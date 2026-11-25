import matplotlib.pyplot as plt

from .io import files
from .jamming import jam
from .plotting import plots

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

