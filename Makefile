SHELL := /bin/bash
BUILD_DIR := build
CONFIG ?= Release
ASAN ?= OFF

.PHONY: all configure build run clean distclean debug release asan

all: build

configure:
	@mkdir -p $(BUILD_DIR)
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CONFIG) -DENABLE_ASAN=$(ASAN)

build: configure
	@cmake --build $(BUILD_DIR) --config $(CONFIG)
	@echo
	@echo "Built binary: ./3drendering"

run: build
	@./3drendering

debug:
	@$(MAKE) CONFIG=Debug ASAN=$(ASAN) build

release:
	@$(MAKE) CONFIG=Release ASAN=$(ASAN) build

asan:
	@$(MAKE) CONFIG=Debug ASAN=ON build

clean:
	@$(RM) -r $(BUILD_DIR)
	@$(RM) ./3drendering
	@echo "Cleaned build artifacts"

distclean: clean
