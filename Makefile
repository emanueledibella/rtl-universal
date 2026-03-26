CC = cc
CPPFLAGS = -isystem /opt/homebrew/include -Idemod/header -Iutility
CFLAGS = -O2 -std=c11
LDFLAGS = -L/opt/homebrew/lib
LDLIBS = -lrtlsdr -lliquid -lportaudio -lusb-1.0 -lm

TARGET = rtl-universal
SRC = rtl-universal.c $(wildcard modules/*.c) $(wildcard demod/*.c) $(wildcard utility/*.c)

.PHONY: clean

$(TARGET): $(SRC)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRC) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) rtl-universal antenna.o
