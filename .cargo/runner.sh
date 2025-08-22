#! /usr/bin/env bash
set -euo pipefail

echo "flashing with elf2uf2-rs"
elf2uf2-rs -d $1

#exit
sleep 3

echo "Attaching defmt-print.."
devs=(/dev/ttyACM?)
defmt-print --log-format "{t} {L} {c} {s}" -e $1 serial --dtr --path "${devs[0]}"