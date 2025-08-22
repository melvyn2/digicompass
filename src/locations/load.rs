use alloc::vec;

use core::cmp::min;
use core::num::ParseFloatError;

use embedded_io::{ErrorType, ReadExactError};

use embedded_sdmmc::{
    BlockDevice, File, Mode, RawDirectory, SdCardError, TimeSource, VolumeManager,
};

use hifijson::num::LexWrite;
use hifijson::str::LexAlloc;
use hifijson::token::Lex;
use hifijson::{Error as JsonError, Expect, IterLexer, Token, ignore};
use postcard::Error as PostcardError;

use crate::FAKE_TIMESOURCE;
use crate::locations::ruler::lat_long_to_ecef;
use crate::locations::{
    IoError, LocNode, LocParseError, LocTree, NameData, SOURCE_NAME, STRINGS_CACHE_NAME,
    TREE_CACHE_NAME,
};
use rstar::RTree;
use uom::si::angle::degree;
use uom::si::f64::Angle;

impl From<ReadExactError<IoError>> for LocParseError {
    fn from(value: ReadExactError<IoError>) -> Self {
        match value {
            ReadExactError::UnexpectedEof => Self::InvalidCacheStrLen,
            ReadExactError::Other(ioerr) => Self::IoError(ioerr),
        }
    }
}

impl From<PostcardError> for LocParseError {
    fn from(value: PostcardError) -> Self {
        Self::PostcardError(value)
    }
}

impl From<JsonError> for LocParseError {
    fn from(value: JsonError) -> Self {
        Self::JsonError(value)
    }
}

impl From<Expect> for LocParseError {
    fn from(value: Expect) -> Self {
        Self::JsonError(JsonError::Token(value))
    }
}

impl From<ParseFloatError> for LocParseError {
    fn from(_: ParseFloatError) -> Self {
        Self::InvalidCoords
    }
}
impl From<hifijson::num::Error> for LocParseError {
    fn from(_: hifijson::num::Error) -> Self {
        Self::InvalidCoords
    }
}

fn file_optional<T>(r: Result<T, IoError>) -> Result<Option<T>, IoError> {
    match r {
        Ok(t) => Ok(Some(t)),
        Err(IoError::NotFound) => Ok(None),
        Err(e) => Err(e),
    }
}

pub fn load_tree<D: BlockDevice<Error = SdCardError>, T: TimeSource>(
    vm: &VolumeManager<D, T>,
    dir: RawDirectory,
) -> Result<LocTree, LocParseError> {
    let cache_time = file_optional(vm.find_directory_entry(dir, STRINGS_CACHE_NAME))?
        .map(|e| e.ctime.max(e.mtime));
    let source_time = match vm.find_directory_entry(dir, SOURCE_NAME) {
        Ok(de) => de.ctime.max(de.mtime),
        Err(IoError::NotFound) => return Err(LocParseError::SourceMissing),
        Err(e) => return Err(e.into()),
    };

    defmt::trace!("loaded file times");
    // All file writes will now have mtime == source_time, if the cache needs to be built
    FAKE_TIMESOURCE.set(source_time);

    if cache_time.map(|ct| ct == source_time).unwrap_or(false) {
        load_cached_tree(vm, dir).or_else(|_| build_tree(vm, dir))
    } else {
        build_tree(vm, dir)
    }
}

fn load_cached_tree<D: BlockDevice<Error = SdCardError>, T: TimeSource>(
    vm: &VolumeManager<D, T>,
    dir: RawDirectory,
) -> Result<LocTree, LocParseError> {
    defmt::debug!("Attempting to load cached tree");
    let tree_file = vm
        .open_file_in_dir(dir, TREE_CACHE_NAME, Mode::ReadOnly)?
        .to_file(vm);
    let strs_file = vm.open_file_in_dir(dir, STRINGS_CACHE_NAME, Mode::ReadOnly)?;

    let mut buf: [u8; 512] = [0u8; 512];
    let (rtree, file) = postcard::from_eio((FileEIOWrapper(tree_file), &mut buf))?;
    file.0.0.close()?;

    defmt::info!("Loaded cached tree");
    Ok(LocTree { rtree, strs_file })
}

struct FileByteIter<
    'a,
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> {
    file: File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    // FAT block size
    buf: [u8; 512],
    len: usize,
    idx: usize,
    eof: bool,
}

impl<
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> Iterator for FileByteIter<'_, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Item = Result<u8, LocParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.eof {
            return None;
        }

        if self.idx == self.len {
            self.len = match self.file.read(&mut self.buf) {
                Ok(0) | Err(IoError::EndOfFile) => {
                    self.eof = true;
                    return None;
                }
                Ok(n) => n,
                Err(e) => return Some(Err(e.into())),
            };
            self.idx = 0;
        }

        let res = Some(Ok(self.buf[self.idx]));
        self.idx += 1;
        res
    }
}

impl<
    'a,
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> FileByteIter<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn new(file: File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>) -> Self {
        Self {
            file,
            buf: [0; 512],
            len: 0,
            idx: 0,
            eof: false,
        }
    }
}

// For some reason the embedded-io traits hang
// TODO fix that
// this is a hacky fix in the mean time
struct FileEIOWrapper<
    'a,
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(File<'a, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>);

impl<
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> ErrorType for FileEIOWrapper<'_, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    type Error = embedded_sdmmc::Error<D::Error>;
}

impl<
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> embedded_io::Read for FileEIOWrapper<'_, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        match self.0.read(buf) {
            Ok(n) if n == buf.len() => Ok(()),
            Ok(_) => Err(ReadExactError::UnexpectedEof),
            Err(e) => Err(ReadExactError::Other(e)),
        }
    }
}

impl<
    D: BlockDevice<Error = SdCardError>,
    T: TimeSource,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
> embedded_io::Write for FileEIOWrapper<'_, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).map(|_| buf.len())
    }

    fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush()
    }
}

/// Reads the GeoJSON data from locs.json into an r-tree of points with names
/// To save memory, the names are written to a strings file on the SD card and the points
/// only keep their file offset and string length. Unfortunately the JSON strings can't be used
/// directly because that would require re-parsing them for escapes on every use
pub fn build_tree<D: BlockDevice<Error = SdCardError>, T: TimeSource>(
    vm: &VolumeManager<D, T>,
    dir: RawDirectory,
) -> Result<LocTree, LocParseError> {
    defmt::debug!("Attempting to build tree from JSON");
    let source = match vm.open_file_in_dir(dir, SOURCE_NAME, Mode::ReadOnly) {
        Ok(f) => f.to_file(vm),
        Err(IoError::NotFound) => return Err(LocParseError::SourceMissing),
        Err(e) => return Err(e.into()),
    };
    let strings = vm
        .open_file_in_dir(dir, STRINGS_CACHE_NAME, Mode::ReadWriteCreateOrTruncate)?
        .to_file(vm);

    // fucked up ugly parser
    let mut elems = vec![];
    let mut lexer = IterLexer::new(FileByteIter::new(source));
    lexer.exactly_one(|t, l| match t {
        Token::LCurly => l.seq(Token::RCurly, |t, l| {
            let key = l.str_colon(t, |l| l.str_string().map_err(JsonError::Str))?;
            let t = l.ws_token().ok_or(Expect::Value)?;
            if key != "features" {
                return ignore::parse(t, l).map_err(LocParseError::JsonError);
            }
            match t {
                Token::LSquare => l.seq(Token::RSquare, |t, l| match t {
                    Token::LCurly => {
                        let mut name = None;
                        let mut coords = None;
                        l.seq(Token::RCurly, |t, l| {
                            let key = l.str_colon(t, |l| l.str_string().map_err(JsonError::Str))?;
                            let t = l.ws_token().ok_or(Expect::Value)?;
                            match key.as_str() {
                                "properties" => match t {
                                    Token::LCurly => l.seq(Token::RCurly, |t, l| {
                                        let key = l.str_colon(t, |l| {
                                            l.str_string().map_err(JsonError::Str)
                                        })?;
                                        let t = l.ws_token().ok_or(Expect::Value)?;
                                        if key != "name" {
                                            return ignore::parse(t, l)
                                                .map_err(LocParseError::JsonError);
                                        }
                                        match t {
                                            Token::Quote => {
                                                name =
                                                    Some(l.str_string().map_err(JsonError::Str)?);
                                                // defmt::trace!("got name");
                                                Ok(())
                                            }
                                            _ => Err(Expect::Value.into()),
                                        }
                                    }),
                                    _ => Err(Expect::Value.into()),
                                },
                                "geometry" => match t {
                                    Token::LCurly => l.seq(Token::RCurly, |t, l| {
                                        let key = l.str_colon(t, |l| {
                                            l.str_string().map_err(JsonError::Str)
                                        })?;
                                        let t = l.ws_token().ok_or(Expect::Value)?;
                                        if key != "coordinates" {
                                            return ignore::parse(t, l)
                                                .map_err(LocParseError::JsonError);
                                        }
                                        match t {
                                            Token::LSquare => {
                                                l.eat_whitespace();
                                                // yes, the order is long, lat for some reason
                                                let long = l.num_string()?.0.parse::<f64>()?;
                                                if l.ws_token() != Some(Token::Comma) {
                                                    return Err(Expect::CommaOrEnd.into());
                                                }
                                                l.eat_whitespace();
                                                let lat = l.num_string()?.0.parse::<f64>()?;
                                                if l.ws_token() != Some(Token::RSquare) {
                                                    return Err(Expect::CommaOrEnd.into());
                                                };
                                                coords = Some((lat, long));
                                                // defmt::trace!("got coords");
                                                Ok(())
                                            }
                                            _ => Err(Expect::Value.into()),
                                        }
                                    }),
                                    _ => Err(Expect::Value.into()),
                                },
                                _ => ignore::parse(t, l).map_err(LocParseError::JsonError),
                            }
                        })?;
                        if name.is_none() {
                            defmt::warn!("Skipping location with missing name");
                            Ok(())
                        } else if let Some(name) = name
                            && let Some(coords) = coords
                        {
                            let name_offset = strings.offset();
                            // Truncate to 255 bytes
                            let nb = name.as_bytes();
                            // defmt::trace!("Writing string");
                            let name_len = min(nb.len(), u8::MAX as usize);
                            // strings.write_all(&nb[..name_len])?;
                            strings.write(&nb[..name_len])?;

                            let pos = [
                                Angle::new::<degree>(coords.0),
                                Angle::new::<degree>(coords.1),
                            ];
                            let pos_ecef = lat_long_to_ecef(&pos);
                            elems.push(LocNode::new(
                                [pos_ecef[0].value, pos_ecef[1].value, pos_ecef[2].value],
                                NameData {
                                    name_offset,
                                    name_len: name_len as u8,
                                },
                            ));
                            // defmt::trace!("Pushed entry");
                            Ok(())
                        } else {
                            Err(LocParseError::MissingProperty)
                        }
                    }
                    _ => Err(Expect::Value.into()),
                }),
                _ => Err(Expect::Value.into()),
            }
        }),
        _ => Err(Expect::Value.into()),
    })?;

    strings.close()?;
    defmt::info!("Wrote string cache");
    defmt::flush();
    let strs_file = vm.open_file_in_dir(dir, STRINGS_CACHE_NAME, Mode::ReadOnly)?;

    let tree = LocTree {
        rtree: RTree::bulk_load(elems),
        strs_file,
    };
    defmt::info!("Built tree");

    let cache_file = vm
        .open_file_in_dir(dir, TREE_CACHE_NAME, Mode::ReadWriteCreateOrTruncate)?
        .to_file(vm);
    postcard::to_eio(&tree.rtree, FileEIOWrapper(cache_file))?
        .0
        .close()?;
    defmt::info!("Wrote tree cache");

    Ok(tree)
}
