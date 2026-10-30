CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
SRC = main.cpp $(wildcard ../lib/gc_core/src/*.cpp)

all: gc_host gc_host_f64
gc_host: $(SRC) $(wildcard ../lib/gc_core/src/*.h)
	$(CXX) $(CXXFLAGS) -I../lib/gc_core/src $(SRC) -o $@
gc_host_f64: $(SRC) $(wildcard ../lib/gc_core/src/*.h)
	$(CXX) $(CXXFLAGS) -DGC_REAL=double -I../lib/gc_core/src $(SRC) -o $@
clean:
	rm -f gc_host gc_host_f64 log_*.csv
