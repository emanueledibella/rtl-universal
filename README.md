# rtl-universal

Ricevitore RTL-SDR modulare per voce analogica, AIS navale e ADS-B 1090ES.
La modalità selezionata all'avvio configura frequenza, sample rate,
demodulatore e decoder; sono disponibili anche input offline e output adatti
ad altri programmi.

## Compilazione (macOS)

```sh
brew install librtlsdr liquid-dsp portaudio pkg-config
make strict
make test
```

Il binario generato è `rtl-universal`. `make strict` abilita anche tutti i
warning usati come errori.

## Avvio rapido con RTL-SDR

```sh
# Elenca indice, modello e seriale dei ricevitori collegati
./rtl-universal --list-devices

# Voce: la frequenza è obbligatoria
./rtl-universal 145.500 --mode voice --demod fm
./rtl-universal 118.300 --mode voice --demod am

# Entrambi i canali AIS con un solo dongle, centro a 162.000 MHz
./rtl-universal --mode ais

# Un solo canale: A=161.975 MHz, B=162.025 MHz
./rtl-universal --mode ais --ais-channel A
./rtl-universal --mode ais --ais-channel B

# ADS-B 1090ES; la posizione aiuta la decodifica CPR locale/surface
./rtl-universal --mode adsb --lat 41.9028 --lon 12.4964
```

Per AIS e ADS-B l'output predefinito è una dashboard che si aggiorna sul
posto: ogni MMSI o ICAO occupa una sola riga e i campi vengono completati
quando arrivano nuovi messaggi. La tabella mostra anche età dell'ultimo
messaggio, numero di messaggi ricevuti, stato del dongle e statistiche del
segnale. Per tornare al flusso dettagliato riga per riga si usa
`--output log`:

```sh
./rtl-universal --mode adsb --device 0 --output log
./rtl-universal --mode ais --device 0 --output log
```

La modalità voce continua invece a usare automaticamente l'output testuale.

### Comandi consigliati

Dashboard ADS-B con il dongle numero 0, statistiche ogni 5 secondi e
riconnessione automatica dopo 3 secondi:

```sh
./rtl-universal --mode adsb --device 0 --stats 5 --reconnect 3
```

Dashboard AIS con le stesse impostazioni:

```sh
./rtl-universal --mode ais --device 0 --stats 5 --reconnect 3
```

Per visualizzare il precedente flusso dettagliato, con una nuova riga per
ogni messaggio ricevuto:

```sh
./rtl-universal --mode adsb --device 0 --stats 5 --reconnect 3 --output log
./rtl-universal --mode ais --device 0 --stats 5 --reconnect 3 --output log
```

`--device 0` seleziona il primo dongle, `--stats 5` aggiorna le statistiche
ogni 5 secondi e `--reconnect 3` tenta una nuova connessione dopo 3 secondi
in caso di errore o disconnessione.

Si può scegliere il dongle per indice o seriale, visualizzare le statistiche
ogni due secondi e tentare la riconnessione dopo una disconnessione:

```sh
./rtl-universal --mode adsb --device 1 --stats 2 --reconnect 3
./rtl-universal --mode ais --device 00000001 --ppm -20
```

Nella dashboard le statistiche compaiono nell'intestazione; con gli altri
formati sono scritte su `stderr`. Includono stato del ricevitore, potenza
approssimata in dBFS, clipping e frame/s. In modalità ADS-B mostrano anche
`candidate_rate` (preamboli candidati al secondo), frame validi, CRC errati,
candidati con DF diverso da 17/18 e frame respinti per contrasto
insufficiente. Questi ultimi vengono conteggiati senza riempire il terminale
con una riga per ogni scarto. Con `--stats 0` le statistiche vengono
disabilitate.

Per AIS sono accettati anche gli alias `ship`, `ships`, `navi` e `maritime`;
per ADS-B sono accettati `ads-b` e `adb-s`.

## NMEA AIS

Le frasi NMEA `!AIVDM` contengono messaggi ricevuti da altre navi; `!AIVDO`
contiene normalmente il messaggio della propria stazione. Il programma
verifica il checksum NMEA, ricompone automaticamente i messaggi composti da
più righe, decodifica l'armoring a 6 bit e poi interpreta il payload AIS.

```sh
./rtl-universal \
  --ais-nmea '!AIVDM,1,1,,A,13co>HP01p0q=3PGvQd7Dmpt0000,0*7E'

# File o pipe di frasi !AIVDM/!AIVDO
./rtl-universal --input messaggi.nmea --input-format nmea --output json
cat messaggi.nmea | ./rtl-universal --input - --input-format nmea --output csv
```

Quando si usa una shell, la frase va racchiusa tra apici singoli perché il
carattere `!` può avere un significato speciale.

## AVR e Beast ADS-B

AVR è un formato testuale: un frame è rappresentato come esadecimale, ad
esempio `*8D...;`; la variante che inizia con `@` aggiunge un timestamp a
12 MHz. Beast è il corrispondente flusso binario compatto con timestamp,
livello del segnale ed escaping del byte `0x1A`. Sono supportati i record
Beast Mode A/C (tipo 1, ignorato correttamente) e Mode-S corti/lunghi (tipi 2
e 3); il decoder ADS-B applica poi la validazione CRC e accetta gli extended
squitter DF17/DF18.

```sh
# Input AVR o Beast da file/pipe
./rtl-universal --input frames.avr --input-format avr --output json
./rtl-universal --input frames.beast --input-format beast --output csv

# Output compatibile con programmi esterni
./rtl-universal --mode adsb --output avr > frames.avr
./rtl-universal --mode adsb --output beast > frames.beast
```

I formati di output disponibili sono `dashboard` (predefinito per AIS e
ADS-B), `log`, `json`, `csv`, `avr`, `beast` e `quiet`; `table` resta un alias
di `dashboard`. AVR e Beast sono specifici di ADS-B. Dati decodificati o
binari vanno su `stdout`; con i formati destinati alle pipe, stato, errori e
statistiche vanno su `stderr`, così file e flussi non vengono contaminati.

## Test e input offline

```sh
# Suite deterministiche senza dongle
./rtl-universal --mode ais --ais-test --output log
./rtl-universal --mode adsb --adsb-test --output log

# Frame ADS-B 112 bit con validazione CRC
./rtl-universal --adsb-frame 8D40621D58C382D690C8AC2863A7

# Payload AIS Information già estratto da HDLC, nel formato packed interno
./rtl-universal --ais-payload <hex>

# Replay di una registrazione RTL-SDR CU8: I,Q unsigned a 8 bit interlacciati
./rtl-universal --mode ais --input registrazione.cu8 --input-format iq-u8
./rtl-universal --mode adsb --input registrazione.cu8 --input-format iq-u8
```

`--ais-test` verifica NRZI, HDLC, bit de-stuffing, CRC-16, NMEA e i messaggi
sintetici di posizione/dati viaggio/AtoN. Verifica inoltre il demodulatore con
due segnali GMSK IQ sintetici simultanei a -25 e +25 kHz, inclusi filtro di
canale e recupero del timing. `--adsb-test` usa frame 1090ES noti e un burst
PPM sintetico completo.

Il replay `iq-u8` permette di conservare e aggiungere registrazioni reali di
regressione senza modificare il codice. Il sample rate della registrazione
deve coincidere con quello della modalità: 2.4 MS/s per AIS, 2.0 MS/s per
ADS-B.

## CRC in breve

Il CRC è un controllo d'integrità calcolato sui bit del messaggio. Il mittente
allega il risultato e il ricevitore lo ricalcola: se non coincide, il frame è
quasi certamente alterato da rumore o interferenze e viene scartato. Non è
cifratura e normalmente non ripara i bit. Qui vengono verificati CRC-24 Mode S
per ADS-B e CRC-16 HDLC per AIS; i relativi scarti compaiono nelle statistiche.

## Protocolli implementati

- ADS-B/Mode S: preambolo e PPM a 2 MS/s, frame corti/lunghi, CRC Mode S,
  DF17/DF18, identificazione, categoria, posizione surface/airborne, CPR
  globale e locale, quota barometrica/GNSS, velocità, vertical rate,
  emergency/squawk, target state e operational status.
- AIS: due canali simultanei AIS 1/A e AIS 2/B, mixer e filtri separati,
  recupero del timing, GMSK 9.6 ksym/s, NRZI, HDLC, bit de-stuffing, CRC-16,
  NMEA `!AIVDM`/`!AIVDO` mono e multi-frammento e messaggi AIS 1-28.

Mostrare tutte le opzioni:

```sh
./rtl-universal --help
```

Riferimenti di formato: [ITU-R M.1371](https://www.itu.int/rec/R-REC-m.1371/en),
[NMEA 0183](https://www.nmea.org/nmea-0183.html) e
[Mode-S Beast data output](https://wiki.jetvision.de/wiki/Mode-S_Beast%3AData_Output_Formats).
