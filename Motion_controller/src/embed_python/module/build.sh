#!/bin/bash

PLAT=/usr/local/aarch64-linux-gnu//bin/aarch64-linux-gnu
export CC="${PLAT}-gcc -std=gnu++11 -pthread"
export CXX="${PLAT}-g++ -std=gnu++11 -pthread"
export LDSHARED="${CC} -shared"
# export PYTHONPATH=/usr/local/roscrosstool/junlong/usr/local/lib/python3.9
# export EXT_SUFFIX=.cpython-39-arm-linux-gnueabihf.so
# export SOABI=cpython-39-arm-linux-gnueabihf
cd ../../../build/embed_python
cd -
cd ../../../install/lib
cd -
# python3 setup.py build --build-base=../../../build/embed_python/module install --prefix=../../../install/lib/module
python3 setup.py build_ext --build-temp=../../../build/embed_python/module --build-lib=../../../install/lib/module/packages --plat-name=arm-linux-gnueabihf
cd ../library
cp -rf * ../../../install/lib/module/
cd -