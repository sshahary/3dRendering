SHELL := /bin/bash
BUILD_DIR := build
CONFIG ?= Release
ASAN ?= OFF

BIN_GUI := render-gui
BIN_CLI := render-to-file
BIN_PATHS := ./$(BIN_GUI) ./$(BIN_CLI) \
             $(BUILD_DIR)/$(BIN_GUI) $(BUILD_DIR)/$(BIN_CLI) \
             $(BUILD_DIR)/$(CONFIG)/$(BIN_GUI) $(BUILD_DIR)/$(CONFIG)/$(BIN_CLI)

.PHONY: all configure build run clean clean-bins distclean debug release asan

all: build

configure:
	@mkdir -p $(BUILD_DIR)
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CONFIG) -DENABLE_ASAN=$(ASAN)

build: configure
	@cmake --build $(BUILD_DIR) --config $(CONFIG)
	@echo
	@{ \
	  found_all=1; \
	  for b in "$(BIN_GUI)" "$(BIN_CLI)"; do \
	    if [ -x "./$$b" ]; then echo "Built: ./$$b ✓"; \
	    elif [ -x "$(BUILD_DIR)/$$b" ]; then echo "Built: $(BUILD_DIR)/$$b ✓"; \
	    elif [ -x "$(BUILD_DIR)/$(CONFIG)/$$b" ]; then echo "Built: $(BUILD_DIR)/$(CONFIG)/$$b ✓"; \
	    else echo "Warning: $$b not found (check CMake RUNTIME_OUTPUT_DIRECTORY)"; found_all=0; fi; \
	  done; \
	  if [ $$found_all -eq 1 ]; then echo "Both executables compiled successfully."; fi; \
	}
	@echo

run: build
	@if [ -n "$(OBJ)" ]; then \
	  if [ -x "./$(BIN_GUI)" ]; then ./$(BIN_GUI) "$(OBJ)"; \
	  elif [ -x "$(BUILD_DIR)/$(BIN_GUI)" ]; then "$(BUILD_DIR)/$(BIN_GUI)" "$(OBJ)"; \
	  else "$(BUILD_DIR)/$(CONFIG)/$(BIN_GUI)" "$(OBJ)"; fi; \
	else \
	  echo "Usage: make run OBJ=path/to/model.obj"; exit 1; \
	fi

debug:
	@$(MAKE) CONFIG=Debug ASAN=$(ASAN) build

release:
	@$(MAKE) CONFIG=Release ASAN=$(ASAN) build

asan:
	@$(MAKE) CONFIG=Debug ASAN=ON build

clean:
	@$(RM) -r "$(BUILD_DIR)"
	@echo "Cleaned build artifacts"

clean-bins:
	@$(RM) -f $(BIN_PATHS)
	@echo "Removed executables: $(BIN_GUI), $(BIN_CLI)"

distclean: clean clean-bins
	@echo "Fully cleaned (build dir and executables removed)"
