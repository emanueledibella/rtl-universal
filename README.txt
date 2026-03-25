# To compile C
```sh
cc -O2 -std=c11 \
  wfm_live.c \
  modules/ais_decoder.c \
  modules/adbs_decoder.c \
  modules/am_demod.c \
  modules/gmsk_demod.c \
  modules/demodulator.c \
  -o wfm_live \
  -I/opt/homebrew/include -L/opt/homebrew/lib \
  -lrtlsdr -lliquid -lusb-1.0 -lm


```

#Examples
./wfm_live 162.0 --mode ais
./wfm_live 162.0 --mode ais --ais-test
./wfm_live 162.025 40 --mode ais --ppm 20 --bw 200000
./wfm_live 1090.0 --mode adbs
./wfm_live 1090.0 --mode adbs --adbs-test

# AIS quick start
# AIS1: 161.975 MHz, AIS2: 162.025 MHz
./wfm_live 161.975 --mode ais
./wfm_live 162.025 --mode ais
# Optional tuner bandwidth and ppm correction
./wfm_live 162.025 --mode ais --bw 25000 --ppm -20


brew install librtlsdr liquid-dsp pkg-config
export PKG_CONFIG_PATH="$(brew --prefix)/lib/pkgconfig:$(brew --prefix)/share/pkgconfig"
