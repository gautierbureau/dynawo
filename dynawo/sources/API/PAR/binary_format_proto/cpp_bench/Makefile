CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
LDLIBS   ?= -lexpat

all: bench

bench: bench.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f bench

.PHONY: all clean
