#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
source ${SCRIPT_DIR}/variables.sh

docker build ${PROJECT_DIR} -t rust:1.85
