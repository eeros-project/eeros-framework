#!/bin/bash

## Patches EEROS source code to allow ECMasterlib
## 
## Execute this scritp in the EEROS source directory and build EEROS
##
## To revert the patch execute this script again and answer 'y' when asked
## "Reversed (or previously applied) patch detected!  Assume -R? [n]"
##
## Marcel Gehrig	19.12.2017
## ############################################################################

[[ $_ != $0 ]] && script_is_sourced=true || script_is_sourced=false
if [ $script_is_sourced ]; then
    SCRIPT_PATH=$BASH_SOURCE
else
    SCRIPT_PATH="$(readlink -f $0)"
fi
SCRIPT_DIR="$(dirname $SCRIPT_PATH)"
SCRIPT_NAME=$(basename $SCRIPT_PATH)

pushd includes/eeros/core
patch < adECMasterLib.patch


