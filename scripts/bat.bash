#!/bin/bash
set -euxo pipefail
cd "${BASH_SOURCE[0]%/*}"/..
ssh -t robotclub@robotclub-latitude.local \
  watch acpi
