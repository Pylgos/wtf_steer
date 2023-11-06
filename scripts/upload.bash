#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..

PIO=/home/teru/.platformio/penv/bin/pio
BIN=$(pwd)/.pio/build/nucleo_f446re/firmware.bin
DST=/run/media/robotclub/NOD_F446RE/
$PIO run

PC=robotclub@robotclub-latitude.local
scp "$BIN" "$PC":"$DST"
