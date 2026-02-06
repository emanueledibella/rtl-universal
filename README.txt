# To compile C
```sh
cc -O2 -std=c11 wfm_live.c modules/ais_decoder.c modules/voice_decoder.c -o wfm_live -I/opt/homebrew/include -L/opt/homebrew/lib -lrtlsdr -lportaudio -lusb-1.0 -lm


```

#Examples
./wfm_live 100.0 20 voice
./wfm_live 162.0 --mode ais

./wfm_live 162.0 --mode ais --ais-test
./wfm_live 162.025 40 --mode ais --ppm 20 --bw 200000
