#!/bin/bash
set -euo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
PC=robotclub@robotclub-latitude.local
ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
  -t "$PC" canbusload can0@1000000
