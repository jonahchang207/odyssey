CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic

ROOT := ..
TARGET := mcl_tests
BENCHMARK := mcl_benchmark
SOURCES := mcl_tests.cpp $(ROOT)/src/odyssey/mcl.cpp $(ROOT)/src/odyssey/pose.cpp
BENCHMARK_SOURCES := mcl_benchmark.cpp $(ROOT)/src/odyssey/mcl.cpp $(ROOT)/src/odyssey/pose.cpp

.PHONY: test benchmark clean

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/include $(SOURCES) -o $(TARGET)

benchmark: $(BENCHMARK)
	./$(BENCHMARK)

$(BENCHMARK): $(BENCHMARK_SOURCES)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/include $(BENCHMARK_SOURCES) -o $(BENCHMARK)

clean:
	rm -f $(TARGET) $(BENCHMARK)
