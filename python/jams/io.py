import os

import numpy as np


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
