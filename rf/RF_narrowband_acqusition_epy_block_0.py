import numpy as np
import os
from datetime import datetime
from gnuradio import gr

class blk(gr.sync_block):
    def __init__(self,
                 csv_path="logs/rf",
                 fs=1000,
                 nfft=1024,
                 fc=2.4e9,
                 shifted=True,
                 freq_unit="Mhz",
                 log_every_n=1,
                 gain_db=0.0,
                 rssi_offset_db=0.0,
                 save_center_hz=None,
                 save_bw_hz=5000.0):

        nfft_i = int(nfft)
        fs_f = float(fs)
        fc_f = float(fc)
        shifted_b = bool(shifted)
        log_every_n_i = int(log_every_n)
        gain_db_f = float(gain_db)
        rssi_offset_db_f = float(rssi_offset_db)
        save_bw_hz_f = float(save_bw_hz)

        if log_every_n_i < 1:
            log_every_n_i = 1

        gr.sync_block.__init__(
            self,
            name="CSV Trace Logger",
            in_sig=[(np.float32, nfft_i)],
            out_sig=None
        )

        self.fs = fs_f
        self.nfft = nfft_i
        self.fc = fc_f
        self.shifted = shifted_b
        self.freq_unit = str(freq_unit)
        self.log_every_n = log_every_n_i
        self._vec_count = 0
        self.gain_db = gain_db_f
        self.rssi_offset_db = rssi_offset_db_f
        self.save_bw_hz = save_bw_hz_f
        self.save_center_hz = self.fc if save_center_hz is None else float(save_center_hz)

        run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trace_{run_tag}.csv"

        csv_path_str = str(csv_path)
        if csv_path_str.lower().endswith(".csv"):
            full_path = csv_path_str
        else:
            out_dir = csv_path_str
            os.makedirs(out_dir, exist_ok=True)
            full_path = os.path.join(out_dir, filename)

        self.csv_path = full_path

        self._rebuild_keep_idx()

        self.fp = open(self.csv_path, "w", buffering=1)

        self.fp.write(f"# Fc_Hz={self.fc:.6f}\n")
        self.fp.write(f"# SaveCenter_Hz={self.save_center_hz:.6f}\n")
        self.fp.write(f"# SaveBW_Hz={self.save_bw_hz:.6f}\n")
        self.fp.write(f"# Fs_Hz={self.fs:.6f}\n")
        self.fp.write(f"# NFFT={self.nfft}\n")
        self.fp.write(f"# Freq_unit={self.freq_unit}\n")
        self.fp.write(f"# Shifted={self.shifted}\n")
        self.fp.write(f"# log_every_n={self.log_every_n}\n")
        self.fp.write(f"# Gain_dB={self.gain_db:.2f}\n")
        self.fp.write(f"# RSSI_Offset_dB={self.rssi_offset_db:.2f}\n")
        self.fp.write(f"# saved_bins={len(self.keep_idx)}\n")

        if self.freq_unit.lower() == "mhz":
            header_freqs = self.freqs_kept_hz / 1e6
            fmt_freq = "{:.6f}"
        else:
            header_freqs = self.freqs_kept_hz
            fmt_freq = "{:.0f}"

        header = "timestamp," + ",".join(fmt_freq.format(f) for f in header_freqs) + "\n"
        self.fp.write(header)

        print(f"[EPY] Logging to: {self.csv_path}")
        print(f"[EPY] log_every_n = {self.log_every_n}")
        print(f"[EPY] saving {len(self.keep_idx)} / {self.nfft} bins around {self.save_center_hz/1e6:.6f} MHz")

    def _freq_axis_hz(self):
        if self.shifted:
            return self.fc + (np.arange(self.nfft) - self.nfft / 2.0) * (self.fs / self.nfft)
        else:
            return self.fc + np.arange(self.nfft) * (self.fs / self.nfft)

    def _rebuild_keep_idx(self):
        freqs_hz = self._freq_axis_hz()

        bw = abs(self.save_bw_hz)
        if bw <= 0:
            mask = np.ones(self.nfft, dtype=bool)
        else:
            mask = (
                (freqs_hz >= (self.save_center_hz - bw)) &
                (freqs_hz <= (self.save_center_hz + bw))
            )

        idxs = np.where(mask)[0]
        if idxs.size == 0:
            k0 = int(np.argmin(np.abs(freqs_hz - self.save_center_hz)))
            idxs = np.array([k0], dtype=int)

        self.keep_idx = idxs
        self.freqs_kept_hz = freqs_hz[self.keep_idx]

    def work(self, input_items, output_items):
        vecs = input_items[0]

        for v in vecs:
            self._vec_count += 1
            if (self._vec_count % self.log_every_n) != 0:
                continue

            v_keep = v[self.keep_idx]
            ts = datetime.now().isoformat(timespec="milliseconds")
            line = ts + "," + ",".join(f"{x:.2f}" for x in v_keep) + "\n"
            self.fp.write(line)

        return len(vecs)

    def stop(self):
        try:
            self.fp.close()
        except Exception:
            pass
        return True
