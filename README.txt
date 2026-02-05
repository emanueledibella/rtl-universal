# To compile C
`
cc -O2 -std=c11 wfm_live.c -o wfm_live \
  -I/opt/homebrew/include \
  -L/opt/homebrew/lib \
  -lrtlsdr -lportaudio -lusb-1.0 -lm
`