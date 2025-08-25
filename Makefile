SHELL := /bin/bash
BUILD_DIR := build
CONFIG ?= Release
ASAN ?= OFF

BIN_GUI  := render-gui
BIN_CLI  := render-to-file
BIN_TEST := core_tests
BINS     := $(BIN_GUI) $(BIN_CLI) $(BIN_TEST)

BIN_PATHS := $(foreach b,$(BINS),./$(b) $(BUILD_DIR)/$(b) $(BUILD_DIR)/$(CONFIG)/$(b))

.PHONY: all configure build run clean clean-bins distclean debug release asan format test

all: build

configure:
	@mkdir -p $(BUILD_DIR)
	@cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(CONFIG) -DENABLE_ASAN=$(ASAN)

build: configure
	@cmake --build $(BUILD_DIR) --config $(CONFIG)
	@echo
	@{ \
	  found_gui=0; found_cli=0; \
	  for b in $(BINS); do \
	    if   [ -x "./$$b" ]; then echo "Built: ./$$b ✓"; path="./$$b"; \
	    elif [ -x "$(BUILD_DIR)/$$b" ]; then echo "Built: $(BUILD_DIR)/$$b ✓"; path="$(BUILD_DIR)/$$b"; \
	    elif [ -x "$(BUILD_DIR)/$(CONFIG)/$$b" ]; then echo "Built: $(BUILD_DIR)/$(CONFIG)/$$b ✓"; path="$(BUILD_DIR)/$(CONFIG)/$$b"; \
	    else echo "Warning: $$b not found (check CMake RUNTIME_OUTPUT_DIRECTORY)"; continue; fi; \
	    [ "$$b" = "$(BIN_GUI)" ] && found_gui=1; \
	    [ "$$b" = "$(BIN_CLI)" ] && found_cli=1; \
	  done; \
	  if [ $$found_gui -eq 1 ] && [ $$found_cli -eq 1 ]; then \
	    echo "Both executables compiled successfully."; \
	  fi; \
	}
	@echo

run: build
	@if [ -n "$(OBJ)" ]; then \
	  if   [ -x "./$(BIN_GUI)" ]; then ./$(BIN_GUI) "$(OBJ)"; \
	  elif [ -x "$(BUILD_DIR)/$(BIN_GUI)" ]; then "$(BUILD_DIR)/$(BIN_GUI)" "$(OBJ)"; \
	  else                               "$(BUILD_DIR)/$(CONFIG)/$(BIN_GUI)" "$(OBJ)"; fi; \
	else \
	  echo "Usage: make run OBJ=path/to/model.obj"; exit 1; \
	fi

format:
	@command -v clang-format >/dev/null || { echo "clang-format not found"; exit 1; }
	@find src include apps tests \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) | xargs clang-format -i

debug:
	@$(MAKE) CONFIG=Debug ASAN=$(ASAN) build

release:
	@$(MAKE) CONFIG=Release ASAN=$(ASAN) build

asan:
	@$(MAKE) CONFIG=Debug ASAN=ON build

test: build
	@ctest --test-dir $(BUILD_DIR) --output-on-failure -C $(CONFIG)

clean:
	@$(RM) -r "$(BUILD_DIR)"
	@echo "Cleaned build artifacts"

clean-bins:
	@$(RM) -f $(BIN_PATHS)
	@echo "Removed executables (if present): $(BINS)"

distclean: clean clean-bins
	@echo "Fully cleaned (build dir and executables removed)"
