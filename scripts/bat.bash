#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
PC=robotclub@robotclub-latitude.local
ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
  -t "$PC" \
  watch -n30 acpi
