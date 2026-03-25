CC = cc
CFLAGS = -O2 -std=c11 -I/opt/homebrew/include -Idemod/header
LDFLAGS = -L/opt/homebrew/lib
LDLIBS = -lrtlsdr -lliquid -lportaudio -lusb-1.0 -lm

TARGET = rtl-universal
SRC = rtl-universal.c $(wildcard modules/*.c) $(wildcard demod/*.c)

.PHONY: clean

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) rtl-universal antenna.o
