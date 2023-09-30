#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
ssh robotclub@robotclub-latitude.local sudo picocom /dev/ttyACM0 -b 115200 \
  | tee logs/device-monitor-$(date +'%y%m%d-%H%M%S')-ssh.log
