mkdir -p build install &&
cd build &&
cmake -D CMAKE_TOOLCHAIN_FILE=../configuration/x86.cmake ../src &&
make &&
make install

