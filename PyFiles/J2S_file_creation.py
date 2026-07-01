import jams

RECORD_DIR_NAME = r"/home/user/Projects/icom"
DEST_DIR_NAME = r"/home/user/Projects/files_for_phi/tst" 

if __name__ == "__main__":
    fs = 15.36e6
    dur = 250e-3
    jammer = jams.jam(fs, dur)
    file = jams.files(RECORD_DIR_NAME, DEST_DIR_NAME)

    rec = jammer.adjust_length(file.read_from_sc16("rate_15.36e6_freq_144e6.sc16"))
    rec = jammer.normalize_signal(rec)
    file.save_to_sc16(rec, "rec")

    signal = jammer.create_multi_cw([0], 0)
    file.save_to_sc16(signal, "cw")

    # For J2S < 0: lower the jammer's power
    for j in range(-10, 1):
        sig_adj = jammer.adjust_signal_power(signal.copy(), -j)
        S = sig_adj + rec
        S = jammer.normalize_signal(S)
        file.save_to_sc16(S, f"J2S{j}")

    # For J2S > 0: lower the record's power
    for j in range(1, 11):
        rec_adj = jammer.adjust_signal_power(rec.copy(), j)
        S = rec_adj + signal
        S = jammer.normalize_signal(S)
        file.save_to_sc16(S, f"J2S{j}")
    