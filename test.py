import numpy as np
from rtlsdr import RtlSdr
import sounddevice as sd
from scipy.signal import firwin, lfilter, resample_poly

def design_lowpass(cut_hz: float, fs: float, numtaps: int = 129):
    return firwin(numtaps, cut_hz, fs=fs)

def fm_demod(iq: np.ndarray) -> np.ndarray:
    # Quadrature FM discriminator: angle(x[n] * conj(x[n-1]))
    prod = iq[1:] * np.conj(iq[:-1])
    return np.angle(prod)

import numpy as np


# ESEMPIO USO (commenta quando non ti serve):
# t, m, f_inst, f_est, s = _test_fm_instantaneous_frequency(tone_hz=1000, dev_hz=5000, fs=240000, seconds=0.05)
# t, m, f_inst, f_est, s = _test_fm_instantaneous_frequency(tone_hz=1000, dev_hz=75000, fs=240000, seconds=0.02)

def _test_fm_instantaneous_frequency(
    tone_hz: float = 1_000.0,
    dev_hz: float = 75_000.0,
    fs: float = 240_000.0,
    seconds: float = 0.02,
):
    """
    TEST / DIDATTICA (puoi commentarla quando vuoi)

    Genera un tono m(t)=sin(2π f_tone t) e costruisce un segnale FM IQ:
        f_inst(t) = dev_hz * m(t)
        phase(t)  = 2π * ∫ f_inst(t) dt
        s(t)      = exp(j * phase(t))

    Poi stima f_inst(t) dai campioni IQ con:
        f_inst_est[n] = (fs / 2π) * angle( s[n] * conj(s[n-1]) )

    Cosa devi vedere:
    - f_inst_est oscilla tra circa -dev_hz e +dev_hz
    - la velocità dell’oscillazione dipende da tone_hz (es. 1000 cicli/s)
    """
    n = int(fs * seconds)
    t = np.arange(n) / fs

    # 1) Segnale "audio" (tono puro): valori tra -1 e +1
    m = np.sin(2 * np.pi * tone_hz * t)

    # 2) FM: frequenza istantanea (in Hz) = deviazione_massima * audio
    f_inst = dev_hz * m

    # 3) Integro la frequenza per ottenere la fase (in radianti)
    # phase[n] = 2π * sum(f_inst)/fs
    phase = 2 * np.pi * np.cumsum(f_inst) / fs

    # 4) Segnale complesso IQ (ampiezza costante, varia solo la fase)
    s = np.exp(1j * phase).astype(np.complex64)

    # 5) Stima della frequenza istantanea dal segnale IQ (discriminatore FM)
    prod = s[1:] * np.conj(s[:-1])
    f_inst_est = (fs / (2 * np.pi)) * np.angle(prod)  # Hz
    f_inst_est = np.concatenate([[f_inst_est[0]], f_inst_est])  # allinea lunghezze

    print("=== FM TEST ===")
    print(f"tone_hz={tone_hz} Hz | dev_hz={dev_hz} Hz | fs={fs} Hz | seconds={seconds}")
    print(f"Estimated f_inst min/max: {f_inst_est.min():.1f} Hz / {f_inst_est.max():.1f} Hz")
    print("Expected approx min/max:", f"{-dev_hz:.1f} Hz / {dev_hz:.1f} Hz")
    print("Tip: aumenta 'seconds' (es. 0.1) per osservare più cicli del tono.\n")

    # Ritorno i vettori nel caso tu voglia plottarli altrove
    return t, m, f_inst, f_inst_est, s



def main():
    # ====== Parametri ======
    freq = 100.0e6            # cambia con una stazione forte (88-108 MHz)
    sdr_fs = 2_048_000         # campionamento SDR
    audio_fs = 48_000          # uscita audio
    gain = 20                  # prova 10..35
    rf_cut = 120_000           # LPF prima della demod (WFM ~200 kHz)
    audio_cut = 15_000         # LPF audio mono baseband
    block_sdr = 262144         # blocco IQ (più grande = meno drop, più latenza)

    # ====== SDR init ======
    sdr = RtlSdr()
    sdr.sample_rate = sdr_fs
    sdr.center_freq = freq
    sdr.gain = gain

    # Filtri FIR
    rf_lpf = design_lowpass(rf_cut, sdr_fs, numtaps=257)
    audio_lpf = design_lowpass(audio_cut, sdr_fs, numtaps=257)

    # Prepara audio stream
    sd.default.samplerate = audio_fs
    sd.default.channels = 1

    print("START: FM live")
    print(f"  freq={freq/1e6:.3f} MHz | sdr_fs={sdr_fs} | audio_fs={audio_fs} | gain={gain}")
    print("  Ctrl+C per uscire")

    # Stato filtri (per continuità tra blocchi)
    rf_zi = np.zeros(len(rf_lpf) - 1, dtype=np.complex64)
    aud_zi = np.zeros(len(audio_lpf) - 1, dtype=np.float32)

    # Stream audio
    with sd.OutputStream(dtype='float32', samplerate=audio_fs, channels=1, blocksize=0):
        try:
            while True:
                iq = sdr.read_samples(block_sdr).astype(np.complex64)

                # RF low-pass
                iq_f, rf_zi = lfilter(rf_lpf, 1.0, iq, zi=rf_zi)

                # FM demod
                dem = fm_demod(iq_f).astype(np.float32)

                # Audio low-pass
                aud, aud_zi = lfilter(audio_lpf, 1.0, dem, zi=aud_zi)

                # Resample -> 48 kHz
                # (resample_poly vuole interi: up/down)
                aud_48k = resample_poly(aud, up=audio_fs, down=sdr_fs).astype(np.float32)

                # De-emphasis semplice (opzionale, migliora suono FM)
                # 75us per US, 50us per EU. Per EU usa 50e-6
                tau = 50e-6
                alpha = np.exp(-1.0 / (audio_fs * tau))
                # IIR 1° ordine: y[n] = (1-alpha)*x[n] + alpha*y[n-1]
                y = np.empty_like(aud_48k)
                prev = 0.0
                k = 1.0 - alpha
                for i, x in enumerate(aud_48k):
                    prev = k * x + alpha * prev
                    y[i] = prev
                aud_48k = y

                # Normalize soft
                aud_48k -= np.mean(aud_48k)
                m = np.max(np.abs(aud_48k)) + 1e-9
                aud_48k = 0.6 * (aud_48k / m)

                sd.play(aud_48k, samplerate=audio_fs, blocking=True)

        except KeyboardInterrupt:
            print("\nSTOP")
        finally:
            sdr.close()

if __name__ == "__main__":
    main()
