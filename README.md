# Deps on mac
```sh
brew install librtlsdr liquid-dsp portaudio pkg-config
export PKG_CONFIG_PATH="$(brew --prefix)/lib/pkgconfig:$(brew --prefix)/share/pkgconfig"
```

# To compile C
```sh
make
```

# Examples
```sh
./wfm_live 145.500 --mode voice
./wfm_live 145.500 --mode voice --demod fm
./wfm_live 118.300 --mode voice --demod am
./wfm_live 162.0 --mode ais
./wfm_live 162.0 --mode ais --ais-test
./wfm_live 162.025 40 --mode ais --ppm 20 --bw 200000
./wfm_live 1090.0 --mode adsb
./wfm_live 1090.0 --mode adsb --adsb-test

# AIS quick start
# AIS1: 161.975 MHz, AIS2: 162.025 MHz
./wfm_live 161.975 --mode ais
./wfm_live 162.025 --mode ais
# Optional tuner bandwidth and ppm correction
./wfm_live 162.025 --mode ais --bw 25000 --ppm -20
```

