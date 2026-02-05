import numpy as np
import threading, time
from collections import deque

from rtlsdr import RtlSdr
import sounddevice as sd
from scipy.signal import firwin, lfilter


def fir_lowpass(cut_hz: float, fs: float, numtaps: int = 129):
    return firwin(numtaps, cut_hz, fs=fs).astype(np.float32)

def fm_demod(iq: np.ndarray) -> np.ndarray:
    prod = iq[1:] * np.conj(iq[:-1])
    return np.angle(prod).astype(np.float32)

def deemphasis_iir(x: np.ndarray, fs: float, tau: float, zi: float):
    # 1-pole IIR: y[n]=(1-a)x[n]+a y[n-1]
    a = float(np.exp(-1.0 / (fs * tau)))
    b = np.array([1.0 - a], dtype=np.float32)
    a_coeff = np.array([1.0, -a], dtype=np.float32)
    y, zf = lfilter(b, a_coeff, x, zi=[zi])
    return y.astype(np.float32), float(zf[0])


class AudioRingBuffer:
    def __init__(self, max_samples: int):
        self.buf = deque()
        self.max_samples = max_samples
        self.size = 0
        self.lock = threading.Lock()

    def push(self, x: np.ndarray):
        if x.size == 0:
            return
        x = np.asarray(x, dtype=np.float32)
        with self.lock:
            self.buf.append(x)
            self.size += x.size
            while self.size > self.max_samples and self.buf:
                old = self.buf.popleft()
                self.size -= old.size

    def pop(self, n: int) -> np.ndarray:
        out = np.zeros(n, dtype=np.float32)
        with self.lock:
            i = 0
            while i < n and self.buf:
                chunk = self.buf[0]
                take = min(n - i, chunk.size)
                out[i:i+take] = chunk[:take]
                i += take
                if take == chunk.size:
                    self.buf.popleft()
                else:
                    self.buf[0] = chunk[take:]
                self.size -= take
        return out


def main():
    # ===== Parametri =====
    freq = 100.0e6          # cambia con una stazione forte
    sdr_fs = 240_000        # <<<<< chiave: basso e stabile
    audio_fs = 48_000
    decim_audio = 5         # 240k / 5 = 48k (perfetto)
    gain = 20               # prova 10..35

    rf_cut = 100_000        # tieni quasi tutto il canale FM (~200k)
    audio_cut = 15_000
    tau = 50e-6             # de-emphasis EU

    block_sdr = 48_000      # 0.2s di IQ a 240k (puoi aumentare a 96_000)
    callback_frames = 1024  # frames richiesti dall'audio callback

    # Ring buffer: mettiamolo più grande (10s) per assorbire jitter USB
    ring = AudioRingBuffer(max_samples=int(audio_fs * 10.0))

    # ===== SDR init =====
    sdr = RtlSdr()
    sdr.sample_rate = sdr_fs
    sdr.center_freq = freq
    sdr.gain = gain

    # Filtri (tutti a 240k o 48k)
    rf_lpf = fir_lowpass(rf_cut, sdr_fs, numtaps=257)
    audio_lpf_240k = fir_lowpass(audio_cut, sdr_fs, numtaps=257)

    rf_zi = np.zeros(len(rf_lpf) - 1, dtype=np.complex64)
    aud_zi = np.zeros(len(audio_lpf_240k) - 1, dtype=np.float32)
    deemp_zi = 0.0

    stop = threading.Event()

    def producer():
        nonlocal rf_zi, aud_zi, deemp_zi
        try:
            while not stop.is_set():
                iq = sdr.read_samples(block_sdr).astype(np.complex64)

                # 1) RF low-pass
                iq_f, rf_zi = lfilter(rf_lpf, 1.0, iq, zi=rf_zi)

                # 2) FM demod (a 240k)
                dem = fm_demod(iq_f)

                # 3) Audio low-pass (a 240k)
                aud_240k, aud_zi = lfilter(audio_lpf_240k, 1.0, dem, zi=aud_zi)

                # 4) Decima esatta a 48k (prendo 1 campione ogni 5)
                aud_48k = aud_240k[::decim_audio].astype(np.float32)

                # 5) De-emphasis (a 48k)
                aud_48k, deemp_zi = deemphasis_iir(aud_48k, audio_fs, tau, deemp_zi)

                # 6) Normalizzazione soft
                aud_48k -= float(np.mean(aud_48k))
                m = float(np.max(np.abs(aud_48k)) + 1e-9)
                aud_48k = (0.5 / m) * aud_48k

                ring.push(aud_48k)

        except Exception as e:
            print("Producer error:", repr(e))
            stop.set()

    t = threading.Thread(target=producer, daemon=True)
    t.start()

    def callback(outdata, frames, time_info, status):
        # se ring è vuoto, esce silenzio (niente scatto “burst”)
        outdata[:, 0] = ring.pop(frames)

    print("START WFM (240k->48k) — Ctrl+C per uscire")
    print(f"freq={freq/1e6:.3f} MHz | sdr_fs={sdr_fs} | gain={gain}")

    try:
        with sd.OutputStream(
            samplerate=audio_fs,
            channels=1,
            dtype="float32",
            callback=callback,
            blocksize=callback_frames,
            latency="high",
        ):
            while not stop.is_set():
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nSTOP")
    finally:
        stop.set()
        try:
            sdr.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()