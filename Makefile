.PHONY: all clean native wasm

EMSDK_PATH ?= $(HOME)/src/emsdk

SRC_FILES := $(wildcard src/*)
RESOURCE_FILES := $(wildcard resources/*)
ROM_FILES := $(wildcard roms/*)

all: help

wasm: $(SRC_FILES) $(RESOURCE_FILES) $(ROM_FILES)
	zig build -Dtarget=wasm32-emscripten --sysroot $(EMSDK_PATH)/upstream/emscripten

native: $(SRC_FILES)
	zig build

clean:
	rm -rf zig-out

help:
	@echo "Zig Project Makefile"
	@echo "-------------------"
	@echo "make native : Build for native target"
	@echo "make wasm   : Build for wasm32-emscripten target"
	@echo "make clean  : Remove build artifacts"
