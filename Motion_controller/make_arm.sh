mkdir -p build install &&
cd build &&
cmake -D CMAKE_TOOLCHAIN_FILE=../configuration/arm.cmake ../src &&
make &&
make install

