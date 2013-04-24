#!/bin/sh

if [ ! -d ../build ]; then
    mkdir -p ../build;
fi

cmake -G"Eclipse CDT4 - Unix Makefiles" ..