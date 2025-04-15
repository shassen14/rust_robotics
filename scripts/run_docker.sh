#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
source ${SCRIPT_DIR}/variables.sh


xhost +local:root
docker compose -f ${SCRIPT_DIR}/docker-compose.yml run --rm rust-dev
xhost -local:root
