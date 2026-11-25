import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['agg.path.chunksize'] = 10000  # Prevent Agg OverflowError
mpl.rcParams['path.simplify'] = True        # Enable path simplification
mpl.rcParams['path.simplify_threshold'] = 0.1  # Default is 0.111..., adjust if needed


class plots:
    def __init__(self, fs):
        self.fs = fs

    def show_fft(self, signal, label = 'Signal'):
        # Create an FFT plot of a given signal
        fft = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(signal))) + 1e-12)
        f = np.fft.fftshift(np.fft.fftfreq(len(fft), 1/self.fs))
        plt.figure()
        plt.plot(f/1e6, fft)
        plt.xlabel("f [MHz]")
        plt.ylabel("FFT")
        plt.title(f"FFT of {label}")

    def show_spectrogram(self, signal, label = 'Signal'):
        # Create a spectrogram plot of a given signal
        plt.figure()
        plt.specgram(signal, NFFT = 1024, Fs=self.fs, cmap='viridis', scale='dB')
        plt.xlabel('t [s]')
        plt.ylabel('f [Hz]')
        plt.title(f"Spectrogram of {label}") 
    
    def show_constellation(self, signal, order, samps_per_sym, label = 'Signal'):
        signal = signal[::samps_per_sym]
        plt.figure()
        plt.plot(np.real(signal), np.imag(signal), 'o')
        plt.xlabel('I (real)')
        plt.ylabel('Q (imag)')
        plt.ylim(-1, 1)
        plt.xlim(-1, 1)
        plt.title(f"Constellation diagram of {label}")
