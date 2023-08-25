#!/bin/bash

path=`pwd`

cd ../..
# tar -cvjSf src.tar.bz2 cpp
if [ -d "cpp" ]; then
    echo "Saving cpp to cpp.tar.bz2"
    rm -fr cpp/build
    tar -cvjSf src.tar.bz2 cpp
    mv src.tar.bz2 ${path}
else
    echo "cpp directory not found"
    exit 1
fi

# cd ${path}