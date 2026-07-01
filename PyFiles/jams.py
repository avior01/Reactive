import os
import warnings 
import numpy as np
#import sounddevice as sd
import scipy.signal as sc
from scipy.signal import resample_poly
from scipy.signal import firwin, lfilter
from scipy.io import wavfile
import matplotlib.pyplot as plt
import matplotlib as mpl

WAV_DIR_NAME = r"/home/user/" 

warnings.filterwarnings("ignore")
mpl.rcParams['agg.path.chunksize'] = 10000  # Prevent Agg OverflowError
mpl.rcParams['path.simplify'] = True        # Enable path simplification
mpl.rcParams['path.simplify_threshold'] = 0.1  # Default is 0.111..., adjust if needed

class files:
    def __init__(self, read_files_dir, write_files_dir):
        self.read_files_dir = read_files_dir
        self.write_files_dir = write_files_dir

    def read_from_sc16(self, filename = "signal"):
        if not filename.endswith('.sc16'):
            filename += ".sc16"

        file_path = os.path.join(self.read_files_dir, filename)
        with open(file_path, 'rb') as f:
            data = f.read()
        
        signal = np.frombuffer(data, dtype = np.int16)
        sig_real = signal[0::2]
        sig_imag = signal[1::2]
        signal = sig_real + 1j * sig_imag
        signal = (signal/32767).astype(np.complex128)
        return signal

    def save_to_sc16(self, signal, filename):
        if not filename.endswith('.sc16'):
            filename += ".sc16"

        file_path = os.path.join(self.write_files_dir, filename)
        sig_real = np.real(signal) * 32767
        sig_imag = np.imag(signal) * 32767
        sig_real = sig_real.astype(np.int16)
        sig_imag = sig_imag.astype(np.int16)
        data = np.empty(2 * len(signal), np.int16)
        data[0::2] = sig_real 
        data[1::2] = sig_imag
        data.astype('<i2').tofile(file_path)

    def save_to_fc32(self, signal, filename):
        if not filename.endswith('.bin'):
            filename += ".bin"

        file_path = os.path.join(self.write_files_dir, filename)
        sig_real = np.real(signal)
        sig_real = sig_real.astype(np.float32)
        sig_imag = np.imag(signal)
        sig_imag = sig_imag.astype(np.float32)
        data = np.empty(2 * len(signal), np.float32)
        data[0::2] = sig_real
        data[1::2] = sig_imag
        data.tofile(file_path)

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

class jam:
    def __init__(self, fs, jam_duration):
        self.fs = fs
        self.jam_dur = jam_duration

    @staticmethod
    def normalize_signal(signal):
        factor = np.max([np.max(np.abs(signal.imag)),np.max(np.abs(signal.real))])
        return (signal/factor)
    
    @staticmethod
    def calc_PAPR(signal):
        power = np.abs(signal)**2
        peak = np.max(power)
        avg = np.mean(power)
        PAPR = peak/avg
        PAPR = 10 * np.log10(PAPR)
        return PAPR

    @staticmethod
    def adjust_signal_power(signal, level_dB):
        level_lin = 10**(level_dB/20)
        print(f"Adjusting signal by {level_lin}, {level_dB} dB")
        signal /= level_lin
        return signal 

    @staticmethod        
    def calc_Backoff(signal, number_of_bins_in_signal):
        if (number_of_bins_in_signal <= 10):
            backoff = 20 * np.log10(number_of_bins_in_signal)
        else:
            backoff = 10 + 10 * np.log10(number_of_bins_in_signal)
        
        backoff = 10 ** (backoff/10)
        signal /= backoff
        return signal

    def adjust_length(self, signal): 
        if (len(signal) < int(self.fs * self.jam_dur)):
            reps = int(np.ceil(int(self.fs * self.jam_dur)/len(signal)))
            signal = np.tile(signal, reps)
        
        signal = signal[:int(self.fs * self.jam_dur)]
        return signal

    def low_pass_filter(self, data, cutoff, Fs):
        # Make sure cutoff is within Nyquist frequency limit
        nyquist = Fs / 2
        normalized_cutoff = min(cutoff / nyquist, 0.99)  # Ensure it stays below 1.0
        
        # Design the filter
        b, a = sc.butter(3, normalized_cutoff, btype='low')
        
        # Check if data needs normalization to prevent overflow
        max_amplitude = np.max(np.abs(data))
        if max_amplitude > 1.0:
            # Normalize, filter, then scale back
            scaling_factor = max_amplitude
            normalized_data = data / scaling_factor
            filtered_data = sc.lfilter(b, a, normalized_data)
            result = filtered_data * scaling_factor
        else:
            # Direct filtering if amplitudes are reasonable
            result = sc.lfilter(b, a, data)
        
        # Handle any NaN or Inf values
        result = np.nan_to_num(result, nan=0, posinf=0, neginf=0)
        
        return result

    def create_gaussian_noise(self, num_samps, noise_bw):
        # Create complex gaussian noise and filter it using Butterworth LPF
        noise = np.random.randn(num_samps) + 1j * np.random.randn(num_samps)
        #noise = noise / np.sqrt(np.mean(np.abs(noise)**2))
        #b, a = sc.butter(3, noise_bw/self.fs, btype='low')
        #noise = sc.lfilter(b, a, noise)

        noise = self.low_pass_filter(noise, noise_bw, Fs=self.fs)
        noise = self.low_pass_filter(noise, noise_bw, Fs=self.fs)

        # Step 4: Normalize again (after filtering)
        noise = noise / np.sqrt(np.mean(np.abs(noise)**2))
        
        # Step 5: Force PAR ≈ 0 dB (clip magnitude to fixed value)
        # Use constant envelope (phase preserved)
        noise = noise / np.abs(noise)  # Now magnitude is 1 everywhere
        return noise

    def create_FM(self, fc, k, f_m = None, signal = None):
        # Create an FM modulation for signal
        N = int(self.fs * self.jam_dur)
        t = np.arange(N)/self.fs
        if signal == None:
            # If user provided no signal to modulate - modulate sin wave
            if f_m == None:
                f_m = 1e6
            signal = np.sin(2 * np.pi * f_m * t)

        phy = 2 * np.pi * fc * t + 2 * np.pi * k * np.cumsum(signal)/self.fs
        FM = np.exp(1j * phy)
        return FM

    def complex_bandpass_filter(self, signal, f_start, f_end, numtaps=512):
        # Center frequency and bandwidth
        f_c = (f_start + f_end) / 2
        bw = f_end - f_start

        # Design lowpass filter for bandwidth bw
        lp_cutoff = bw / 2
        lp_filter = firwin(numtaps, lp_cutoff / (self.fs / 2), window='hamming')

        # Modulate to center frequency (complex bandpass)
        t = np.arange(numtaps) / self.fs
        complex_mod = np.exp(1j * 2 * np.pi * f_c * t)
        bp_filter = lp_filter * complex_mod

        # Apply the filter
        filtered = lfilter(bp_filter, 1.0, signal)
        return filtered

    def create_chirp(self, f_start, f_end, chirp_duration, noise_bw):
        # Create chirp
        N = int(self.fs * chirp_duration)
        t = np.arange(N)/self.fs
        c = (f_end - f_start)/chirp_duration
        phy = 2 * np.pi * (c * t **2/2 + f_start * t)
        chirp = np.exp(1j * phy)

        reps = int(np.ceil(self.jam_dur/chirp_duration))
        chirp = np.tile(chirp, reps)[:int(self.jam_dur * self.fs)]

        if (noise_bw > 0):
            noise = self.create_gaussian_noise(int(self.fs * self.jam_dur), noise_bw)
            chirp *= noise
        
        #chirp = self.complex_bandpass_filter(chirp, f_start, f_end)
        chirp = self.normalize_signal(chirp)
        return chirp
    
    def create_multi_cw(self, cw_list, noise_bw):
        # Create multi cw
        N = int(self.fs * self.jam_dur)
        t = np.arange(N)/self.fs
        freqs = np.array(cw_list).reshape(-1, 1) 
        phase = np.random.uniform(low = -np.pi, high = np.pi, size = (len(freqs), 1))
        phy = 2 * np.pi * freqs * t + phase
        multi = np.sum(np.exp(1j * phy), axis = 0)

        if (noise_bw > 0):
            noise = self.create_gaussian_noise(int(self.fs * self.jam_dur), noise_bw)
            multi *= noise        

        #multi = self.complex_bandpass_filter(multi, freqs[0] - 10e3, freqs[-1] + 10e3)
        multi = self.normalize_signal(multi)
        return multi
    
    def create_cyclic(self, cw_list, durations_list, noise_bw):
        # Create cyclic      
        cyclic = np.zeros(0, dtype=complex)

        for cw, dur in zip(cw_list, durations_list):
            n = int(self.fs * dur)
            t_seg = np.arange(n)/self.fs
            phy = 2 * np.pi * cw * t_seg
            cyclic = np.concatenate([cyclic, np.exp(1j * phy)])
        
        reps = int(np.ceil(self.jam_dur/(np.sum(durations_list))))
        cyclic = np.tile(cyclic, reps)[:int(self.jam_dur * self.fs)]

        if (noise_bw > 0):
            noise = self.create_gaussian_noise(int(self.fs * self.jam_dur), noise_bw)
            cyclic = cyclic * noise
        
        #cyclic = self.complex_bandpass_filter(cyclic, cw_list[0] - 10e3, cw_list[-1] + 10e3)
        cyclic = self.normalize_signal(cyclic)
        return cyclic
    
    def create_cyclic_chirp(self, chirp_list, chirp_durations_list, noise_bw):
        # Use chirp function to create cyclic chirp
        cyc_chirp = np.zeros(0, dtype = complex)
        jam_dur_total = 0

        for (f0, f1), (chirp_dur, jam_dur) in zip(chirp_list, chirp_durations_list):
            jam_dur_total += jam_dur
            cyc_chirp = np.concatenate([cyc_chirp, self.create_chirp(f0, f1, chirp_dur, jam_dur, noise_bw)])

        reps = int(np.ceil(self.jam_dur/jam_dur_total))
        cyc_chirp = np.tile(cyc_chirp, reps)[:int(self.fs * self.jam_dur)]

        cyc_chirp = self.normalize_signal(cyc_chirp)
        return cyc_chirp

    def create_barrage(self, freq_ranges, noise_bw):
        # Create a uniform jam in a given range and frequency hops (freq_ranges = [f_start, f_end, freq_hop])
        N = 8192
        t = np.arange(N)/self.fs
        freq = np.fft.fftshift(np.fft.fftfreq(N, 1/self.fs))

        barrage = np.zeros(N, dtype = complex)
        for f_start, f_end, spacing in freq_ranges:
            freq_points = np.arange(f_start, f_end + spacing, spacing)
            freq_points = freq_points[freq_points <= f_end]
            
            for f_point in freq_points:
                idx = np.argmin(np.abs(freq - f_point))
                val = 1 + 1j * np.random.randn()
                val /= np.abs(val)
                barrage[idx] = val

        barrage_in_time = np.fft.ifft(np.fft.ifftshift(barrage))
        if (noise_bw > 0):
            noise = self.create_gaussian_noise(N, noise_bw)
            barrage_in_time *= noise
        
        reps = int(np.ceil(self.jam_dur/len(t) * self.fs))
        barrage_in_time = np.tile(barrage_in_time, reps)[:int(self.fs * self.jam_dur)]
        barrage_in_time = self.normalize_signal(barrage_in_time)
        return barrage_in_time

    def read_WAV(self, filename = "signal"):
        if not filename.endswith('wav'):
            filename += ".wav"
        
        file_path = os.path.join(WAV_DIR_NAME, filename)
        samp_rate, audio = wavfile.read(file_path) 
        length = audio.shape[0] / samp_rate
        print(f"Length: {length}")
        if (np.issubdtype(audio.dtype, np.integer)):
            max_val = np.iinfo(audio.dtype).max
            audio = audio/max_val
        
        if (audio.ndim > 1):
            print(f"Number of Channels: {audio.shape[1]}")
            print("Clipping...")
            signal = signal[:, 0]

        audio_resampled = resample_poly(audio, up=int(self.fs), down=samp_rate)

        if (length > self.jam_dur):
            audio_resampled = audio_resampled[:int(self.jam_dur * self.fs)]
        elif (length < self.jam_dur):
            reps = int(np.ceil(self.jam_dur/length))
            audio_resampled = np.tile(audio_resampled, reps)[:int(self.jam_dur * self.fs)]
        
        audio_resampled = self.normalize_signal(audio_resampled)
        return samp_rate, audio, audio_resampled

    def create_DTMF(self, tone_dur, char_list):
        # Gets a list of chars and durations and produces the desired DTMF
        freq_list = [[1209, 697], [1336, 770], [1447, 852], [1633, 941]]
        row_freqs = [fr[0] for fr in freq_list]
        col_freqs = [fc[1] for fc in freq_list]
        
        row_dic = {
            '1': 0, 
            '2': 1, 
            '3': 2, 
            'A': 3,
            '4': 0, 
            '5': 1, 
            '6': 2, 
            'B': 3,
            '7': 0, 
            '8': 1, 
            '9': 2, 
            'C': 3,
            '*': 0, 
            '0': 1, 
            '#': 2, 
            'D': 3,
        }
        col_dic = {
            '1': 0, 
            '2': 0, 
            '3': 0, 
            'A': 0,
            '4': 1, 
            '5': 1, 
            '6': 1, 
            'B': 1,
            '7': 1, 
            '8': 2, 
            '9': 2, 
            'C': 2,
            '*': 3, 
            '0': 3, 
            '#': 3, 
            'D': 3,
        }
        DTMF = []
        freq_match = []
        t_tone = np.arange(int(self.fs * tone_dur))/self.fs
        t_dtmf = np.arange(int(self.fs * tone_dur * len(char_list)))/self.fs
        t = np.arange(int(self.fs * self.jam_dur))/self.fs
        for c in char_list:
            if c in row_dic and c in col_dic:
                fr = row_freqs[row_dic[c]]
                fc = col_freqs[col_dic[c]]
                freq_match.append((fr, fc))
                dtmf = np.sin(2 * np.pi * fr * t_tone) + np.sin(2 * np.pi * fc * t_tone)
                DTMF.append(dtmf)
            else:
                print(f"Char {c} is not a valid char.")

        DTMF = np.concatenate(DTMF)
        if (len(t_dtmf < len(t))):
            reps = int(np.ceil(len(t)/len(t_dtmf)))
            DTMF = np.tile(DTMF, reps)[:int(self.fs * self.jam_dur)]
        elif (len(t_dtmf > len(t))):
            DTMF = DTMF[:int(self.fs * self.jam_dur)]

        DTMF = self.normalize_signal(DTMF)        
        """
        plt.figure()
        plt.plot(t, DTMF)
        plt.xlabel("t [s]")
        plt.ylabel("Amplitude")
        plt.title("DTMF Signal") 
        """
        """
        DTMF_play = sc.resample(DTMF, int(len(DTMF) * 48000/self.fs))
        sd.play(DTMF_play, samplerate=48000)
        sd.wait()       
        """
        return DTMF

    def create_BPSK(self, bits):
        num_sym = len(bits)
        samps_per_sym = int(self.fs * self.jam_dur / num_sym)
        t_sym = np.arange(samps_per_sym) / self.fs
        t = np.tile(t_sym, num_sym)
        f = 1 / (self.jam_dur / len(bits))

        coding = {
            (0): np.pi, 
            (1): 0 
        }

        phases = np.array([coding[b] for b in bits])
        phases = np.repeat(phases, samps_per_sym)

        BPSK = np.exp(1j * (2 * np.pi * f * t + phases))
        BPSK = self.adjust_length(BPSK)
        return BPSK

    def create_QPSK(self, bits):
        if (len(bits) % 2 != 0):
            print("ERROR: number of bits isn't divisible by 2")
            return -1
        
        symbols = bits.reshape(-1, 2)
        num_sym = symbols.shape[0] #len(bits) / 2 
        samps_per_sym = int(self.fs * self.jam_dur / num_sym)
        t_sym = np.arange(samps_per_sym) / self.fs
        t = np.tile(t_sym, num_sym)
        f = 1 / (2 * self.jam_dur / len(bits))
        
        coding = {
            (0, 0): np.pi/4, # 1 + 1j,
            (0, 1): 3 * np.pi/4, # -1 + 1j,
            (1, 0): 5 * np.pi/4, # -1 - 1j,
            (1, 1): 7 * np.pi/4 # 1 - 1j
        }

        phases = np.array([coding[tuple(s)] for s in symbols])
        phases = np.repeat(phases, samps_per_sym)

        QPSK = np.exp(1j * (2 * np.pi * f * t + phases))
        QPSK = self.adjust_length(QPSK)
        return QPSK

if __name__ == "__main__":  
    fs = 50e6
    spb = 8192
    dur = 0.5 # 2 * 10e3 * spb/fs # 1.5 #sec
    dir = r"/home/user/Projects/files_for_phi/tst"
    jammer = jam(fs, dur)
    plotter = plots(fs)
    filer = files(None, dir)

    noise_bw = 6.25e3
    cw_list = [0]
    cw = jammer.create_multi_cw(cw_list, noise_bw)
    fc = 0
    k = 6.25e3
    fm = jammer.create_FM(fc, k, f_m=2e3)
    signal = cw * fm
    filer.save_to_sc16(cw, "cw_FM")
    #filer.save_to_fc32(cw, "cw")
    plotter.show_fft(signal)
    plotter.show_spectrogram(signal)
    plt.show()

