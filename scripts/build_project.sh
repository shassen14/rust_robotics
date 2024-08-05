#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
source ${SCRIPT_DIR}/variables.sh

cargo build --manifest-path ${PROJECT_DIR}/Cargo.toml