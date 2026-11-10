CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -pthread -Iinclude
TARGET := a200_server

SRC_DIR := src
BUILD_DIR := build

SRCS := \
	$(SRC_DIR)/a200_server.cpp \
	$(SRC_DIR)/crc.cpp \
	$(SRC_DIR)/horizon_legacy_wrapper.cpp \
	$(SRC_DIR)/linux_serial.cpp \
	$(SRC_DIR)/Logger.cpp \
	$(SRC_DIR)/Message_cmd.cpp \
	$(SRC_DIR)/Message.cpp \
	$(SRC_DIR)/Message_data.cpp \
	$(SRC_DIR)/Message_request.cpp \
	$(SRC_DIR)/Number.cpp \
	$(SRC_DIR)/Transport.cpp

OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))

.PHONY: all clean run install

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: all
	./$(BUILD_DIR)/$(TARGET) /dev/husky-base 9000

clean:
	rm -rf $(BUILD_DIR)

install: all
	sudo cp $(BUILD_DIR)/$(TARGET) /usr/local/bin/$(TARGET)
	sudo cp $(SRC_DIR)/husky.py /usr/local/lib/python3/dist-packages/husky.py
