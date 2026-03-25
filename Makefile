CC = cc
CFLAGS = -O2 -std=c11 -I/opt/homebrew/include -Idemod/header
LDFLAGS = -L/opt/homebrew/lib
LDLIBS = -lrtlsdr -lliquid -lportaudio -lusb-1.0 -lm

SRC = wfm_live.c $(wildcard modules/*.c) $(wildcard demod/*.c)
BUILD_DIR = build
OBJ = $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC))

.PHONY: clean

wfm_live: $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) wfm_live
