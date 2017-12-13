all: Aria-lib MobileSim-lib

Aria-lib:
	CXX=/usr/bin/g++-6 make -C Aria

MobileSim-lib:
	CXX=/usr/bin/g++-6 make -C MobileSim
