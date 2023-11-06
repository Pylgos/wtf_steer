#!/bin/bash
set -euo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
PC=robotclub@robotclub-latitude.local
ssh -t "$PC" canbusload can0@1000000
