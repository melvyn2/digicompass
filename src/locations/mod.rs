use alloc::string::String;
use alloc::vec;

use embedded_sdmmc::{BlockDevice, RawFile, SdCardError, TimeSource, VolumeManager};

use hifijson::Error as JsonError;

use postcard::Error as PostcardError;

use rstar::RTree;
use rstar::primitives::GeomWithData;

use serde::{Deserialize, Serialize};

use thiserror::Error;
use uom::si::f64::Length;

pub mod load;
pub mod ruler;

const TREE_CACHE_NAME: &str = "rtree.ch";
const STRINGS_CACHE_NAME: &str = "strings.ch";
const SOURCE_NAME: &str = "locs.jsn";

type IoError = embedded_sdmmc::Error<SdCardError>;

/// ECEF meters
pub type LocNode = GeomWithData<[f64; 3], NameData>;

#[derive(Debug, Serialize, Deserialize)]
pub struct NameData {
    name_offset: u32,
    name_len: u8,
}

pub struct LocTree {
    rtree: RTree<LocNode>,
    strs_file: RawFile,
}

impl LocTree {
    pub fn nearest(&self, pos: &[Length; 3]) -> Option<&LocNode> {
        self.rtree
            .nearest_neighbor(&[pos[0].value, pos[1].value, pos[2].value])
    }

    pub fn name<'a, D: BlockDevice<Error = SdCardError>, T: TimeSource>(
        &self,
        vm: &VolumeManager<D, T>,
        node: &'a LocNode,
    ) -> Result<String, LocParseError> {
        let file = self.strs_file.to_file(vm);
        file.seek_from_start(node.data.name_offset)?;
        let mut name_buf = vec![0u8; node.data.name_len as usize];
        file.read(name_buf.as_mut_slice())?;
        file.to_raw_file();

        let name = String::from_utf8_lossy_owned(name_buf);

        Ok(name)
    }
}

#[derive(Debug, Error)]
pub enum LocParseError {
    #[error("I/O error while loading/saving tree: {0:?}")]
    IoError(IoError),
    #[error("Postcard error while loading/saving tree: {0}")]
    PostcardError(PostcardError),
    #[error("JSON error while loading/saving tree: {0}")]
    JsonError(JsonError),
    #[error("location node is missing a property")]
    MissingProperty,
    #[error("location node coordinates are not valid")]
    InvalidCoords,
    #[error("wrong location name length (past EOF)")]
    InvalidCacheStrLen,
    #[error("missing locations source file")]
    SourceMissing,
}

impl From<IoError> for LocParseError {
    fn from(value: IoError) -> Self {
        Self::IoError(value)
    }
}
