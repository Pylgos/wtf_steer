#!/bin/bash
set -euo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
PC=robotclub@robotclub-latitude.local
ssh -t "$PC" sudo picocom /dev/ttyACM0 -b 115200 --nolock \
  | while read; do printf '%s > %s\n' "$(date '+%H:%M:%S.%3N')" "$REPLY"; done \
  | tee logs/device-monitor-$(date +'%y%m%d-%H%M%S')-ssh.log
