CXX = g++ -std=c++17
DIR = /usr/local
LIBS = -lgecodedriver -lgecodesearch \
       -lgecodeminimodel -lgecodeint \
       -lgecodekernel -lgecodesupport

all: sdp checker

sdp: sdp.cpp
	$(CXX) -I$(DIR)/include -c sdp.cpp
	$(CXX) -L$(DIR)/lib -o sdp sdp.o $(LIBS)

checker: checker.cc
	$(CXX) -o checker checker.cc
