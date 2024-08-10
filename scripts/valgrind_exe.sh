#!/usr/bin/env bash

SCRIPT_DIR=$(realpath $(dirname $0))
source ${SCRIPT_DIR}/variables.sh

helpFunction()
{
   echo ""
   echo "Usage: $0 -f parameterFile" 
   echo -e "\t-f Executable file name"
   exit 1 # Exit script after printing help
}

# function to make the build directory if it doesn't exist
make_log_dir() {
if [ ! -d ${LOG_DIR} ]; then
  echo "Making the directory, ${LOG_DIR}"
  mkdir ${LOG_DIR}
fi
}

while getopts "f:" opt
do
   case "$opt" in
      f ) parameterFile="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

LOG_FILE=${parameterFile}.txt

make_log_dir
cargo build --manifest-path ${PROJECT_DIR}/Cargo.toml
valgrind --leak-check=full \
         --show-leak-kinds=all \
         --track-origins=yes \
         --verbose \
         --log-file=${LOG_DIR}/${LOG_FILE} \
         ${PROJECT_DIR}/target/debug/${parameterFile}

echo "Finished valgrind ${PROJECT_DIR}/target/debug/${parameterFile}"