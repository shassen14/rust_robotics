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

while getopts "f:" opt
do
   case "$opt" in
      f ) parameterFile="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

# lower case the input
parameterFile=$(echo "${parameterFile}" | tr '[:upper:]' '[:lower:]' )

# TODO: find all file names excluding ".rs" in examples, put in a list, and then 
# be able to run those files according to some standard way of organizing files

if [ ${parameterFile} == "dijkstra" ]; then
  cargo run --release --example ${parameterFile} ${CONFIG_DIR}/path_planning/${parameterFile}_animation.toml
elif [ ${parameterFile} == "generate_custom_map" ]; then
    cargo run --release --example ${parameterFile} ${CONFIG_DIR}/utils/${parameterFile}_animation.toml ${CONFIG_DIR}/utils/${parameterFile}_params.toml
else
  echo "$parameterFile"
fi