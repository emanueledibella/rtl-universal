CC = cc
CFLAGS = -O2 -std=c11 -I/opt/homebrew/include -Idemod/header
LDFLAGS = -L/opt/homebrew/lib
LDLIBS = -lrtlsdr -lliquid -lportaudio -lusb-1.0 -lm

SRC = wfm_live.c $(wildcard modules/*.c) $(wildcard demod/*.c)
OBJ = antenna.o

.PHONY: clean

wfm_live: $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(OBJ): $(SRC)
	@tmpdir=$$(mktemp -d); \
	for src in $(SRC); do \
		obj="$$tmpdir/$$(basename "$$src" .c).o"; \
		$(CC) $(CFLAGS) -c "$$src" -o "$$obj" || exit 1; \
	done; \
	$(CC) -r $$tmpdir/*.o -o $@ || exit 1; \
	rm -rf "$$tmpdir"

clean:
	find . -maxdepth 2 -name '*.o' -delete
	rm -f wfm_live
