CXX = g++ -std=c++17

all: checker

checker: checker.cc
	$(CXX) -o checker checker.cc

clean:
	rm --force checker