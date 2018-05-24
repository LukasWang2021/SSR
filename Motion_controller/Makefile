PWD=${shell pwd}
all:
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/x86.cmake

arm:
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/toolchain.cmake

x86:
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/x86.cmake

test:
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/toolchain.cmake --cmake-args -DGTEST=1

clean:
	rm -rvf ${PWD}/build ${PWD}/devel ${PWD}/install


