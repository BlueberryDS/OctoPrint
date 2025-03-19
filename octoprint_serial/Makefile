# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -O2 -std=c++17

# Target and source files
TARGET = serial_driver
SRC = serial_driver.cpp
INSTALL_DIR = /usr/local/bin

# Build rules
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

install: $(TARGET)
	install -m 755 $(TARGET) $(INSTALL_DIR)

uninstall:
	rm -f $(INSTALL_DIR)/$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all install uninstall clean