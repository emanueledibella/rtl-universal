CC = cc
CPPFLAGS = -isystem /opt/homebrew/include -Idemod/header -Iutility
CFLAGS = -O2 -std=c11
LDFLAGS = -L/opt/homebrew/lib
LDLIBS = -lrtlsdr -lliquid -lportaudio -lusb-1.0 -lm

TARGET = rtl-universal
SRC = rtl-universal.c $(wildcard modules/*.c) $(wildcard demod/*.c) $(wildcard utility/*.c)
ANALOG_TEST_TARGET = tests/analog_frontend_test
SPECTRUM_TEST_TARGET = tests/spectrum_test

.PHONY: all clean strict test ui ui-check

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SRC) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET) $(ANALOG_TEST_TARGET) $(SPECTRUM_TEST_TARGET) antenna.o
	rm -rf $(TARGET).dSYM

strict: clean
	$(MAKE) CFLAGS="$(CFLAGS) -Wall -Wextra -Wpedantic -Werror" all $(ANALOG_TEST_TARGET) $(SPECTRUM_TEST_TARGET)

$(ANALOG_TEST_TARGET): tests/analog_frontend_test.c demod/analog_frontend.c demod/header/analog_frontend.h
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/analog_frontend_test.c demod/analog_frontend.c \
		-o $@ $(LDFLAGS) -lliquid -lm

$(SPECTRUM_TEST_TARGET): tests/spectrum_test.c utility/spectrum.c utility/spectrum.h
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/spectrum_test.c utility/spectrum.c \
		-o $@ $(LDFLAGS) -lliquid -lm

test: $(TARGET) $(ANALOG_TEST_TARGET) $(SPECTRUM_TEST_TARGET)
	./$(ANALOG_TEST_TARGET)
	./$(SPECTRUM_TEST_TARGET)
	./$(TARGET) --mode adsb --adsb-test --output log
	./$(TARGET) --mode ais --ais-test --output log
	./$(TARGET) --mode sonde --test --output log
	./$(TARGET) --mode sstv --test --output log
	./$(TARGET) --mode meteor --test --output log
	./$(TARGET) --mode ads-b --adsb-frame 8D4840D6202CC371C32CE0576098 --output log
	@./$(TARGET) --mode adsb --adsb-frame 8D4840D6202CC371C32CE0576098 --output dashboard | grep -q '4840D6'
	./$(TARGET) --ais-nmea '!AIVDM,1,1,,A,13co>HP01p0q=3PGvQd7Dmpt0000,0*7E' --output json
	@printf '%s\n' '*8D4840D6202CC371C32CE0576098;' | \
		./$(TARGET) --input - --input-format avr --output json
	@printf '%s\n' '@1A000000001A8D4840D6202CC371C32CE0576098;' | \
		./$(TARGET) --input - --input-format avr --output beast | \
		./$(TARGET) --input - --input-format beast --output quiet
	@if ./$(TARGET) --mode adsb --adsb-frame 8D4840D6202CC371C32CE0576099 --output log; then \
		echo "invalid ADS-B CRC was accepted"; exit 1; \
	fi

ui: $(TARGET)
	node ui/server.mjs

ui-check:
	npm run check
