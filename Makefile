LD_LIBRARY_PATH=${PWD}/Aria/lib/:${LD_LIBRARY_PATH}
CXX=/usr/bin/g++-6
CC=/usr/bin/g++-6

CXXFLAGS=-I${PWD}/Aria/include -std=c++11
LDFLAGS=-L${PWD}/Aria/lib -lAria
all: Aria-lib MobileSim-lib app Mapper-bin

Aria-lib:
	make -C Aria

MobileSim-lib:
	make -C MobileSim

Mapper-bin:
	make -C Mapper
