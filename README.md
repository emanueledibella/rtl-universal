# rtl-universal

Ricevitore RTL-SDR modulare per voce analogica, AIS navale, ADS-B/Mode S,
radiosonde RS41, SSTV e immagini satellitari Meteor LRPT.
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

## Avvio

Eseguire i comandi seguenti dalla directory principale del progetto, dopo la
compilazione. Collegare il dongle RTL-SDR prima di avviare una ricezione Live.

### Da CLI

Per controllare che il ricevitore sia visibile e conoscere indice e seriale:

```sh
./rtl-universal --list-devices
```

Avvio Voice FM a 145,500 MHz con squelch e statistiche ogni secondo:

```sh
./rtl-universal --mode voice --freq 145.500 --demod fm \
  --filter-width 15000 --filter-type iir --squelch -30 --stats 1
```

Le altre modalità possono essere avviate direttamente con il relativo
protocollo; AIS, ADS-B e RS41 usano la propria frequenza predefinita:

```sh
./rtl-universal --mode ais
./rtl-universal --mode adsb
./rtl-universal --mode sonde
./rtl-universal --mode sstv --freq 145.800 --save-dir immagini-sstv
```

Usare `Ctrl+C` per terminare la ricezione. L'elenco completo degli argomenti è
disponibile con:

```sh
./rtl-universal --help
```

### Con la GUI

La GUI richiede Node.js, ma non è necessario eseguire `npm install` perché il
progetto non usa dipendenze npm. Compilare il backend C e avviare il controller
locale:

```sh
make strict
npm start
```

Aprire quindi [http://127.0.0.1:4173](http://127.0.0.1:4173). Il controller
avvia e arresta automaticamente `rtl-universal`: non serve lanciare il backend
in un secondo terminale. In alternativa, compilazione e avvio della GUI possono
essere eseguiti con un solo comando:

```sh
make ui
```

Se la porta `4173` è già occupata, se ne può scegliere un'altra:

```sh
RTL_UI_PORT=4174 npm start
```

## RTL Universal Studio: Live e Scan

Il progetto include una UI locale moderna per spettro, waterfall, sintonia e
pannelli dedicati ai protocolli. Non richiede pacchetti npm: usa Node.js per il
controller locale e il binario C per acquisizione, DSP e decoder.

La modalità **Live** acquisisce FFT in parallelo alla decodifica, fino a 60 FPS.
Per Voice si può scegliere uno span istantaneo fino a 2,4 MHz, mantenendo il
filtro audio stretto sul canale centrale. Un clic su spettro o waterfall
seleziona la frequenza da decodificare; la barra verticale può anche essere
trascinata. Questa sintonia è digitale: lo spettro rimane centrato sulla
frequenza impostata nel campo superiore e il processo Live non viene riavviato.
La frequenza centrale del tuner cambia soltanto quando si modifica quel campo.

La modalità **Scan** usa `rtl_power` per attraversare un intervallo ampio, per
esempio 50–1700 MHz, e ricompone progressivamente i blocchi in una mappa unica.
La mappa supporta zoom con la rotella, pan tramite trascinamento e doppio clic
per passare a Live sulla frequenza selezionata. Scan e Live sono mutuamente
esclusivi con un singolo dongle, perché entrambi devono controllare il tuner.

I controlli disponibili nella UI includono dispositivo, gain/AGC, PPM, banda
del tuner, frequenza, FFT e refresh. Il pannello contestuale aggiunge AM/FM,
sample rate, filtro e squelch per Voice; coordinate per ADS-B; canale AIS;
formato e cartella di salvataggio per SSTV. Le tabelle ADS-B, AIS e RS41 sono
alimentate dagli eventi JSON già prodotti dal decoder.

In modalità Voice la banda evidenziata sullo spettro rappresenta la larghezza
del filtro RF. Trascinando una delle due maniglie la banda si allarga o si
restringe simmetricamente intorno al canale selezionato; il valore viene
aggiornato anche nel pannello Controlli e applicato al DSP durante la ricezione,
senza dover fermare e riavviare la sessione Live.

In Voice, attivazione e soglia dello squelch vengono aggiornate anche durante
la ricezione. Il pannello mostra sia il livello del canale post-filtro sia lo
stato aperto/chiuso. Come punto di partenza è consigliata una soglia di
`-30 dBFS`; va poi regolata rispetto al rumore realmente indicato (una soglia
più negativa apre più facilmente lo squelch).

### Registrazione audio e I/Q

Il pannello **Registrazione** può avviare e fermare i file durante una sessione
Live, senza riavviare il ricevitore. In modalità Voice l'audio viene prelevato
dopo demodulazione, de-enfasi e squelch, quindi coincide con ciò che viene
riprodotto. La registrazione I/Q è invece presa direttamente dai campioni del
dongle, prima di FFT, filtro di canale e decoder, ed è disponibile per tutti i
protocolli Live.

Per l'audio sono disponibili:

- `wav-s16`: WAV mono PCM 16-bit a 48 kHz, scelta consigliata e più compatibile;
- `wav-f32`: WAV mono IEEE Float 32-bit, utile per elaborazioni successive;
- `s16le` e `f32le`: PCM raw senza header, per pipeline e strumenti DSP.

I formati comuni per le registrazioni raw SDR inclusi nella GUI sono:

| Formato | Byte per campione I/Q | Uso principale |
| --- | ---: | --- |
| `cu8` | 2 | Byte originali RTL-SDR, file compatti e replay diretto |
| `cs16le` | 4 | Ampia interoperabilità con software SDR |
| `cf32le` | 8 | Elaborazione numerica e DSP senza riconversione |
| `wav-iq-s16` | 4 | I e Q nei due canali PCM; compatibile con strumenti che leggono WAV I/Q |
| `sigmf-cu8` | 2 | CU8 più file `.sigmf-meta` con sample-rate, frequenza e cambi di sintonia |

A 2,4 MS/s occupano circa 4,8 MB/s in CU8, 9,6 MB/s in CS16/WAV I/Q e
19,2 MB/s in CF32. Il WAV RIFF standard è limitato a 4 GB ed è quindi indicato
per acquisizioni brevi; per archiviazione e scambio è consigliato SigMF, mentre
CU8 è la scelta più semplice e compatta. Lasciando vuoto il percorso, la GUI
genera un nome con timestamp nella cartella `registrazioni/`.

Le stesse funzioni sono disponibili dalla CLI e possono essere usate insieme:

```sh
./rtl-universal 145.500 --mode voice --demod fm \
  --record-audio registrazioni/voice.wav --audio-format wav-s16 \
  --record-iq registrazioni/canale.sigmf-data --iq-format sigmf-cu8
```

Per un'acquisizione IQ CU8 compatta:

```sh
./rtl-universal 1090 --mode adsb \
  --record-iq registrazioni/adsb.cu8 --iq-format cu8
```

### FFT live dalla CLI

`--spectrum` abilita il tap FFT. Se lo spettro va su stdout occorre usare
`--output quiet`; una GUI può invece fornire un file descriptor separato con
`--spectrum-fd`, evitando qualsiasi interferenza con gli eventi del decoder.

```sh
./rtl-universal 145.500 --mode voice --demod fm \
  --sample-rate 2400000 --filter-width 15000 --filter-type iir \
  --output quiet --spectrum --fft-size 1024 --spectrum-fps 30
```

## Avvio rapido con RTL-SDR

```sh
# Elenca indice, modello e seriale dei ricevitori collegati
./rtl-universal --list-devices

# Voce: la frequenza è obbligatoria
./rtl-universal 145.500 --mode voice --demod fm
./rtl-universal 118.300 --mode voice --demod am

# Filtro RF di canale da 12,5 kHz e squelch a -45 dBFS
./rtl-universal 145.500 --mode voice --demod fm \
  --filter-width 12500 --filter-type fir --squelch -45

# Entrambi i canali AIS con un solo dongle, centro a 162.000 MHz
./rtl-universal --mode ais

# Un solo canale: A=161.975 MHz, B=162.025 MHz
./rtl-universal --mode ais --ais-channel A
./rtl-universal --mode ais --ais-channel B

# ADS-B 1090ES; la posizione aiuta la decodifica CPR locale/surface
./rtl-universal --mode adsb --lat 41.9028 --lon 12.4964

# Radiosonda RS41: sostituire 403.000 con la frequenza realmente osservata
./rtl-universal --mode sonde --freq 403.000 --device 0

# SSTV satellitare/ISS, salva automaticamente le immagini ricevute
./rtl-universal --mode sstv --freq 145.800 --save-dir immagini-sstv

# Meteor LRPT tramite SatDump, passaggio massimo di 15 minuti
./rtl-universal --mode meteor --freq 137.900 --duration 900 \
  --save-dir immagini-meteor
```

Per AIS, ADS-B e radiosonde l'output predefinito è una dashboard che si
aggiorna sul posto: ogni MMSI, ICAO o seriale occupa una sola riga e i campi vengono completati
quando arrivano nuovi messaggi. La tabella mostra anche età dell'ultimo
messaggio, numero di messaggi ricevuti, stato del dongle e statistiche del
segnale. Per tornare al flusso dettagliato riga per riga si usa
`--output log`:

```sh
./rtl-universal --mode adsb --device 0 --output log
./rtl-universal --mode ais --device 0 --output log
```

La modalità voce continua invece a usare automaticamente l'output testuale.

In modalità `voice`, `--filter-width <hz>` imposta la larghezza RF complessiva
del filtro di canale applicato ai campioni I/Q prima della demodulazione. Il
tipo si sceglie con `--filter-type fir|iir|none`: se si specifica soltanto la
larghezza viene usato `fir`; `0`/`none` disabilitano il filtro. Il FIR Kaiser
offre una selettività più netta, mentre l'IIR Butterworth richiede meno CPU.
`--squelch <dbfs>` silenzia l'audio sotto la soglia indicata (`-120..0`). La
potenza viene mediata per circa 10 ms e confrontata direttamente con la
soglia, usando la stessa convenzione dBFS dello squelch semplice di Gqrx;
`--squelch off` lo disabilita. Con `--stats 1` il log mostra `channel`, cioè
il livello post-filtro effettivamente confrontato, e lo stato
`squelch=open|closed`. Questi controlli DSP sono distinti da `--bw`, che
configura invece la banda analogica del tuner RTL-SDR.

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
`candidate_rate` (preamboli candidati al secondo), frame validi, CRC errati e
frame respinti per contrasto insufficiente. Questi ultimi vengono conteggiati senza riempire il terminale
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
e 3); il decoder applica CRC o address parity secondo il Downlink Format.
Sono accettati DF0, 4, 5, 11, 16, 17, 18, 19, 20, 21 e 24, inclusi Comm-B e
i registri BDS più comuni.

```sh
# Input AVR o Beast da file/pipe
./rtl-universal --input frames.avr --input-format avr --output json
./rtl-universal --input frames.beast --input-format beast --output csv

# Output compatibile con programmi esterni
./rtl-universal --mode adsb --output avr > frames.avr
./rtl-universal --mode adsb --output beast > frames.beast
```

I formati di output disponibili sono `dashboard` (predefinito per AIS, ADS-B
e sonde), `log`, `json`, `csv`, `avr`, `beast` e `quiet`; `table` resta un alias
di `dashboard`. AVR e Beast sono specifici di ADS-B. Dati decodificati o
binari vanno su `stdout`; con i formati destinati alle pipe, stato, errori e
statistiche vanno su `stderr`, così file e flussi non vengono contaminati.

## Test e input offline

```sh
# Suite deterministiche senza dongle
./rtl-universal --mode ais --ais-test --output log
./rtl-universal --mode adsb --adsb-test --output log
./rtl-universal --mode sonde --test --output log
./rtl-universal --mode sstv --test --output log
./rtl-universal --mode meteor --test --output log

# Frame ADS-B 112 bit con validazione CRC
./rtl-universal --adsb-frame 8D40621D58C382D690C8AC2863A7

# Payload AIS Information già estratto da HDLC, nel formato packed interno
./rtl-universal --ais-payload <hex>

# Replay di una registrazione RTL-SDR CU8: I,Q unsigned a 8 bit interlacciati
./rtl-universal --mode ais --input registrazione.cu8 --input-format iq-u8
./rtl-universal --mode adsb --input registrazione.cu8 --input-format iq-u8
./rtl-universal --mode sonde --input registrazione-rs41.cu8 --input-format iq-u8
./rtl-universal --mode sstv --input registrazione-sstv.cu8 --input-format iq-u8
```

`--ais-test` verifica NRZI, HDLC, bit de-stuffing, CRC-16, NMEA e i messaggi
sintetici di posizione/dati viaggio/AtoN. Verifica inoltre il demodulatore con
due segnali GMSK IQ sintetici simultanei a -25 e +25 kHz, inclusi filtro di
canale e recupero del timing. `--adsb-test` usa frame 1090ES noti, tutti i
principali Downlink Format Mode S e un burst PPM sintetico completo. Il test
RS41 attraversa sync, inversione, whitening, correzione Reed–Solomon e CRC
introducendo deliberatamente quattro simboli errati. Il test SSTV genera una
trasmissione VIS PD120 e una linea immagine; quello Meteor verifica la scelta
della pipeline senza richiedere SatDump o il dongle.

Il replay `iq-u8` permette di conservare e aggiungere registrazioni reali di
regressione senza modificare il codice. Il sample rate della registrazione
deve coincidere con quello della modalità: 2.4 MS/s per AIS, 2.0 MS/s per
ADS-B, 240 kS/s per RS41/SSTV e 1 MS/s per Meteor LRPT.

## Radiosonde RS41

La modalità `sonde` implementa internamente le Vaisala RS41 a 4800 baud:
sync GFSK, inversione automatica, de-whitening, due codeword interlacciate
RS(255,231), CRC-16 dei blocchi, seriale, numero frame, batteria, posizione
ECEF/GPS, quota, velocità orizzontale, direzione, salita e satelliti. La
dashboard conserva una riga per ogni seriale.

```sh
# Ricezione live; 403 MHz è un valore iniziale, non una frequenza universale
./rtl-universal --mode sonde --freq 403.000 --device 0 --gain 30

# Flusso dettagliato o JSON per altri programmi
./rtl-universal --mode sonde --freq 402.700 --output log
./rtl-universal --mode sonde --freq 402.700 --output json

# Frame RS41 già de-whitened, lungo almeno 320 byte, in esadecimale
./rtl-universal --mode sonde --sonde-frame <hex> --output json
```

Le radiosonde non trasmettono tutte sulla stessa frequenza e “sonde meteo”
non indica un unico protocollo. Questa modalità decodifica RS41; M10/M20,
DFM, RS92 e iMet hanno formati differenti e non vengono etichettati
erroneamente come RS41. Prima della ricezione conviene individuare il segnale
nella porzione 400–406 MHz con uno spettro o una scansione locale.

## SSTV

**Tipo di protocollo:** SSTV (Slow Scan Television) è un protocollo analogico
che trasporta un'immagine attraverso una sequenza di toni audio modulati in
FM. Non è un flusso video: una singola immagine viene costruita lentamente,
riga dopo riga. Il codice VIS trasmesso all'inizio identifica il formato
dell'immagine.

La modalità `sstv` demodula l'audio FM, riconosce l'intestazione VIS e
decodifica `PD120` (640×496, tipico degli eventi ARISS) e `Martin M1`
(320×256). Al termine salva un file PPM; la cartella viene creata
automaticamente. L'autodetect è predefinito, ma si può forzare il modo se il
VIS è debole o manca.

```sh
# Ricezione SSTV automatica: riconosce PD120 o Martin M1 dal codice VIS
./rtl-universal --mode sstv --freq 145.800 --device 0 \
  --save-dir immagini-sstv

# Forza PD120 quando il codice VIS è assente, debole o disturbato
./rtl-universal --mode sstv --freq 145.800 --sstv-mode pd120 \
  --save-dir immagini-sstv --output log
```

Nel primo comando `--mode sstv` attiva il decoder, `--freq 145.800` imposta
la frequenza radio, `--device 0` seleziona il primo dongle e `--save-dir`
indica dove salvare le immagini. La cartella `immagini-sstv` viene creata
automaticamente. Nel secondo comando `--sstv-mode pd120` disabilita il
riconoscimento automatico e `--output log` mostra l'avanzamento riga per riga.

`145.800 MHz` è la frequenza comunemente usata dagli eventi SSTV ARISS, ma
un'immagine arriva solo durante un evento/passaggio realmente attivo. Per
convertire un PPM in PNG su macOS si può usare:

```sh
sips -s format png immagini-sstv/file.ppm --out immagini-sstv/file.png
```

## Meteor LRPT

**Tipo di protocollo:** LRPT (Low Rate Picture Transmission) è un protocollo
satellitare digitale usato dai satelliti meteorologici Meteor-M per inviare i
dati dello strumento MSU-MR. Il segnale radio utilizza QPSK oppure OQPSK; `72k`
e `80k` indicano la velocità in simboli al secondo. Dopo la demodulazione sono
necessari Viterbi, Reed–Solomon/CCSDS e la ricostruzione dei prodotti immagine.

Meteor LRPT richiede QPSK/OQPSK, recupero di portante e clock, Viterbi,
Reed–Solomon/CCSDS e decompressione delle immagini MSU-MR. La modalità
`meteor` delega questa catena al backend SatDump, ma mantiene un solo comando
di avvio e gli passa tutte le opzioni del ricevitore. Occorre installare il
binario CLI `satdump`; per macOS sono disponibili build autonome nella
[pagina ufficiale delle release](https://github.com/SatDump/SatDump/releases).

```sh
# Live: pipeline OQPSK 72 kbaud, rilevamento automatico del satellite
./rtl-universal --mode meteor --freq 137.900 --device 0 --gain 35 \
  --duration 900 --save-dir immagini-meteor

# Meteor M2 originale: protocollo QPSK a 72 kbaud
./rtl-universal --mode meteor --meteor-pipeline m2 --freq 137.100

# Meteor M2-x: protocollo OQPSK a 72 kbaud (impostazione predefinita)
./rtl-universal --mode meteor --meteor-pipeline m2-x --freq 137.900

# Variante Meteor M2-x con protocollo OQPSK a 80 kbaud
./rtl-universal --mode meteor --meteor-pipeline m2-x-80k --freq 137.900

# Forza il satellite M2-4 e indica l'app SatDump quando non è nel PATH
./rtl-universal --mode meteor --satellite M2-4 \
  --satdump /Applications/SatDump.app --save-dir immagini-meteor

# Decodifica una registrazione I/Q CU8 a 1 MS/s
./rtl-universal --mode meteor --input passaggio.cu8 --input-format iq-u8 \
  --save-dir immagini-meteor
```

Le informazioni QPSK/OQPSK e 72/80k vengono inserite scegliendo
`--meteor-pipeline`: non occorre specificare separatamente modulazione e baud
rate. `m2` seleziona QPSK 72k, `m2-x` seleziona OQPSK 72k e `m2-x-80k`
seleziona OQPSK 80k. Se l'opzione viene omessa viene usata `m2-x`.

Nel comando live `--gain 35` imposta il guadagno del dongle, `--duration 900`
limita la ricezione a 900 secondi e `--save-dir immagini-meteor` indica dove
SatDump deve scrivere dati e immagini. Con `--input passaggio.cu8` la sorgente
non è più il dongle: viene elaborata una registrazione I/Q CU8 a 1 MS/s.
Su macOS `--satdump` accetta sia `/Applications/SatDump.app` sia il percorso
completo `/Applications/SatDump.app/Contents/MacOS/satdump`. Il programma
riconosce automaticamente la sintassi CLI di SatDump 1.x e 2.x.

Le pipeline selezionabili sono `m2` (QPSK 72k), `m2-x` (OQPSK 72k,
predefinita) e `m2-x-80k` (OQPSK 80k). La frequenza predefinita è 137.900
MHz, ma trasmettitore, frequenza e operatività vanno verificati per il
satellite e il passaggio scelti. SatDump crea nella cartella di output i
prodotti e le immagini. Il progetto non scarica automaticamente SatDump e
non nasconde un errore se il backend manca.

## CRC in breve

Il CRC è un controllo d'integrità calcolato sui bit del messaggio. Il mittente
allega il risultato e il ricevitore lo ricalcola: se non coincide, il frame è
quasi certamente alterato da rumore o interferenze e viene scartato. Non è
cifratura e normalmente non ripara i bit. Qui vengono verificati CRC-24 Mode S,
CRC-16 HDLC AIS e CRC-16 dei blocchi RS41; i relativi scarti compaiono nelle
statistiche. La correzione Reed–Solomon RS41, invece, può riparare fino a 12
simboli errati per codeword prima del controllo CRC.

## Protocolli implementati

- ADS-B/Mode S: preambolo e PPM a 2 MS/s, frame corti/lunghi, CRC Mode S,
  DF0/4/5/11/16/17/18/19/20/21/24, address/data parity, Comm-B e BDS
  1,0/2,0/3,0/4,0/4,4/4,5/5,0/6,0, identificazione, categoria, posizione surface/airborne, CPR
  globale e locale, quota barometrica/GNSS, velocità, vertical rate,
  emergency/squawk, target state e operational status.
- AIS: due canali simultanei AIS 1/A e AIS 2/B, mixer e filtri separati,
  recupero del timing, GMSK 9.6 ksym/s, NRZI, HDLC, bit de-stuffing, CRC-16,
  NMEA `!AIVDM`/`!AIVDO` mono e multi-frammento e messaggi AIS 1-28.
- RS41: GFSK 4800 baud, sync/inversione/whitening, Reed–Solomon interlacciato,
  CRC dei blocchi, stato e navigazione GPS.
- SSTV: VIS automatico, PD120 e Martin M1, ricostruzione RGB e salvataggio PPM.
- Meteor LRPT: orchestrazione live/offline delle pipeline SatDump QPSK/OQPSK
  72/80 kbaud e prodotti MSU-MR.

Mostrare tutte le opzioni:

```sh
./rtl-universal --help
```

Riferimenti di formato: [ITU-R M.1371](https://www.itu.int/rec/R-REC-m.1371/en),
[NMEA 0183](https://www.nmea.org/nmea-0183.html),
[Mode-S Beast data output](https://wiki.jetvision.de/wiki/Mode-S_Beast%3AData_Output_Formats),
[SatDump pipelines](https://docs.satdump.org/md_docs_2pages_2Pipelines.html) e
[ARISS SSTV](https://www.ariss.org/press-releases/ariss-news-release-ariss-sstv-event-scheduled-for-this-week).
