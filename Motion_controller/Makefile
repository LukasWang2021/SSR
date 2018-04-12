PWD=${shell pwd}
all:
	#${shell sh -x ${PWD}/configuration/version.sh 2 0 5}
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/x86.cmake

arm:
	#${shell sh -x ${PWD}/configuration/version.sh 2 0 5}
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/toolchain.cmake

x86:
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/x86.cmake

test:
	${shell sh -x ${PWD}/configuration/version.sh 2 0 5}
	catkin_make install -DCMAKE_TOOLCHAIN_FILE=${PWD}/configuration/toolchain.cmake --cmake-args -DGTEST=1

clean:
	rm -rvf ${PWD}/build ${PWD}/devel ${PWD}/install


