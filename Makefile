CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude
BUILD    := build
TARGET   := $(BUILD)/dh_kinematics
SRC      := src/dh_kinematics.cpp

.PHONY: all clean check

all: $(TARGET)

$(TARGET): $(SRC) include/dh_chain.hpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@

check: all
	python scripts/stall_detector.py data/ina219_shunt_capture.csv

clean:
	rm -rf $(BUILD)
