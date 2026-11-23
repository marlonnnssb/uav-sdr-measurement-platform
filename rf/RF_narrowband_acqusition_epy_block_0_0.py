import numpy as np
from gnuradio import gr

class blk(gr.sync_block):
    def __init__(self,
                 fs=1.0,
                 nfft=1024,
                 fc=0.0,
                 fcw=0.0,
                 search_bw=5000.0,
                 shifted=True):

        self.fs = float(fs)
        self.nfft = int(nfft)
        self.fc = float(fc)
        self.fcw = float(fcw)
        self.search_bw = float(search_bw)
        self.shifted = bool(shifted)

        gr.sync_block.__init__(
            self,
            name="PeakAroundFCW",
            in_sig=[(np.float32, self.nfft)],  # vetor dB
            out_sig=[np.float32, np.float32]   # peak_db, peak_freq_mhz
        )

    def _freq_axis_hz(self):
        # mapeia bins -> frequencia absoluta (Hz), consistente com FFT shift
        if self.shifted:
            return self.fc + (np.arange(self.nfft) - self.nfft/2.0) * (self.fs / self.nfft)
        else:
            return self.fc + np.arange(self.nfft) * (self.fs / self.nfft)

    def work(self, input_items, output_items):
        X = input_items[0]          # shape: (nvec, nfft) em dB
        out_db = output_items[0]    # shape: (nvec,)
        out_fm = output_items[1]    # shape: (nvec,)

        freqs_hz = self._freq_axis_hz()

        # janela de busca ao redor de fcw
        bw = abs(self.search_bw)
        if bw <= 0:
            # se bw=0, pega o bin mais proximo de fcw
            mask = np.ones(self.nfft, dtype=bool)
        else:
            mask = (freqs_hz >= (self.fcw - bw)) & (freqs_hz <= (self.fcw + bw))

        idxs = np.where(mask)[0]
        if idxs.size == 0:
            # fallback: pega bin mais proximo
            k0 = int(np.argmin(np.abs(freqs_hz - self.fcw)))
            idxs = np.array([k0], dtype=int)

        # processa cada vetor (frame)
        for i in range(X.shape[0]):
            v = X[i, :]

            sub = v[idxs]
            k_rel = int(np.argmax(sub))
            k = int(idxs[k_rel])

            peak_db = float(v[k])
            peak_freq_mhz = float(freqs_hz[k] / 1e6)

            out_db[i] = peak_db
            out_fm[i] = peak_freq_mhz

        return len(out_db)
