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
