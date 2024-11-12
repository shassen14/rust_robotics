#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
PROJECT_DIR=${SCRIPT_DIR}/..
LOG_DIR=${PROJECT_DIR}/logs
LOG_VALGRIND_DIR=${LOG_DIR}/valgrind
CONFIG_DIR=${PROJECT_DIR}/configs
EXAMPLE_DIR=${PROJECT_DIR}/examples

# function to make the logs directory if it doesn't exist
make_log_dir() {
if [ ! -d ${LOG_DIR} ]; then
  echo "Making the directory, ${LOG_DIR}"
  mkdir ${LOG_DIR}
fi
}

# function to make the logs/valgrind directory if it doesn't exist
make_log_valgrind_dir() {
if [ ! -d ${LOG_VALGRIND_DIR} ]; then
  echo "Making the directory, ${LOG_VALGRIND_DIR}"
  mkdir ${LOG_VALGRIND_DIR}
fi
}