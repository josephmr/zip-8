.PHONY: all clean native wasm run-wasm run-native

EMSDK_PATH ?= $(HOME)/src/emsdk

SRC_FILES := $(wildcard src/*)
RESOURCE_FILES := $(wildcard resources/*)
ROM_FILES := $(wildcard roms/*)

all: help

wasm: $(SRC_FILES) $(RESOURCE_FILES) $(ROM_FILES)
	zig build -Dtarget=wasm32-emscripten --sysroot $(EMSDK_PATH)/upstream/emscripten

run-wasm: wasm
	emrun --no-browser zig-out/htmlout/index.html

zig-out/bin/zip-8: $(SRC_FILES)
	zig build

native: zig-out/bin/zip-8 

run-native: zig-out/bin/zip-8
	./zig-out/bin/zip-8

clean:
	rm -rf zig-out

help:
	@echo "Zig Project Makefile"
	@echo "-------------------"
	@echo "make native      : Build for native target"
	@echo "make wasm        : Build for wasm32-emscripten target"
	@echo "make run-wasm    : Run emrun to host wasm server"
	@echo "make run-native  : Run native binary"
	@echo "make clean       : Remove build artifacts"
