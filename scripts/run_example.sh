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

# parameterFile = "${parameterFile,,}"
# TODO: find all file names excluding ".rs" in examples, put in a list, and then 
# be able to run those files according to some standard way of organizing files

if [ "${parameterFile,,}" == "dijkstra" ];
then
  cargo run --release --example dijkstra ${CONFIG_DIR}/path_planning/dijkstra_animation.toml
fi