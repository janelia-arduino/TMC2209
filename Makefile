CC ?= gcc
CFLAGS ?= -std=c17 -Wall -Wextra -Wpedantic -Werror -O2
CPPFLAGS += -Isrc

BUILD_DIR := build

.PHONY: all
all: $(BUILD_DIR)/libtmc2209.a

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/tmc2209.o: src/tmc2209.c src/tmc2209.h | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/libtmc2209.a: $(BUILD_DIR)/tmc2209.o
	$(AR) rcs $@ $^

$(BUILD_DIR)/test_tmc2209: tests/test_tmc2209.c $(BUILD_DIR)/tmc2209.o | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@

.PHONY: test
test: $(BUILD_DIR)/test_tmc2209
	$(BUILD_DIR)/test_tmc2209

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
