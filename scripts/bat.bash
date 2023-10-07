#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
PC=robotclub@robotclub-latitude.local
ssh -t "$PC" \
  watch acpi
