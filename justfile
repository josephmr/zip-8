export EMSDK_PATH := env("EMSDK_PATH", home_directory() + "/src/emsdk")

default:
  @just --list

# Build for wasm32-emscripten target
build-wasm:
  zig build -Dtarget=wasm32-emscripten --sysroot {{EMSDK_PATH}}/upstream/emscripten

alias wasm := run-wasm

# Run emrun to host wasm server
run-wasm: build-wasm
  emrun --no-browser zig-out/htmlout/index.html

# Build for native target
build-native:
  zig build

alias native := run-native

# Run native binary
run-native: build-native
  ./zig-out/bin/zip-8

# Remove build artifacts
clean:
  rm -rf zig-out
