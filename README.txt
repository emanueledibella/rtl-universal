# To compile C
`
cc -O2 -std=c11 wfm_live.c modules/ais_decoder.c -o wfm_live \
  -I/opt/homebrew/include \
  -L/opt/homebrew/lib \
  -lrtlsdr -lportaudio -lusb-1.0 -lm

`

#Examples
./wfm_live 100.0 20 voice
./wfm_live 162.0 --mode ais
