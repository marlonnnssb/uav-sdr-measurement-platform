import numpy as np
from gnuradio import gr


class blk(gr.sync_block):
    """
    Vector Moving Average

    Performs a temporal moving average independently
    for each FFT frequency bin.

    Input:
        Vector of nfft float32 linear-power samples.

    Output:
        Vector of nfft float32 averaged linear-power samples.
    """

    def __init__(self, nfft=1024, avg_len=10):

        gr.sync_block.__init__(
            self,
            name='Vector Moving Average',
            in_sig=[(np.float32, nfft)],
            out_sig=[(np.float32, nfft)]
        )

        self.nfft = nfft
        self.avg_len = avg_len

        # Circular buffer containing the previous FFT frames.
        self.buffer = np.zeros(
            (avg_len, nfft),
            dtype=np.float64
        )

        # Running sum for each frequency bin.
        self.running_sum = np.zeros(
            nfft,
            dtype=np.float64
        )

        # Current circular-buffer position.
        self.index = 0

        # Number of valid frames currently stored.
        self.count = 0


    def work(self, input_items, output_items):

        x = input_items[0]
        y = output_items[0]

        for i in range(len(x)):

            frame = x[i].astype(np.float64)

            # Initial buffer filling.
            if self.count < self.avg_len:

                self.buffer[self.index] = frame
                self.running_sum += frame

                self.count += 1

                y[i] = (
                    self.running_sum / self.count
                ).astype(np.float32)

            # Normal moving-average operation.
            else:

                # Remove the oldest frame.
                self.running_sum -= self.buffer[self.index]

                # Store the new frame.
                self.buffer[self.index] = frame

                # Add the new frame.
                self.running_sum += frame

                # Average the most recent avg_len frames.
                y[i] = (
                    self.running_sum / self.avg_len
                ).astype(np.float32)

            # Advance circular-buffer index.
            self.index += 1

            if self.index >= self.avg_len:
                self.index = 0

        return len(output_items[0])
