import numpy as np
from scipy.signal.windows import kaiser
from numpy.fft import ifft

def gpr_calc(data, n_freq=8192, freq_min=0.0, freq_max=14.0e9):
    freq = np.linspace(300e3, 14e9, n_freq)

    # Create a mask for the frequencies within the desired range
    ind_ = (freq >= freq_min) & (freq <= freq_max)

    # Filter the frequency array
    freq = freq[ind_]
    data = data[ind_]

    n_freq = len(data)
    ov = 40

    nf = ov * n_freq
    # gpr = np.zeros(nf)
    # time = np.zeros(nf)

    # Apply Kaiser window to the signal
    sig_f = data * kaiser(n_freq, 6)

    # Calculate frequency difference
    df = freq[1] - freq[0]

    # Zero-pad the signal
    sig_f2 = np.zeros(nf, dtype=complex)
    sig_f2[:n_freq] = sig_f
    sig_f = sig_f2

    # Perform inverse FFT
    h_t = ifft(sig_f)*len(sig_f)

    # Time step and time vector
    dt = 1 / (nf * df)
    time = np.arange(nf) * dt

    # Compute GPR values
    gpr = np.abs(2 * ov * h_t)

    return gpr, time, freq

def hrtr_method_fin(s11, freq, dsr, nt, time_min=0.0e-9, time_max=5.0e-9 + 0.01e-9, dt=0.01e-9):

    data_ds = s11[::dsr]
    freq_ds = freq[::dsr]
    freq_ds = freq_ds.reshape(-1, 1)

    time_tr = np.arange(time_min, time_max, dt)

    L = len(data_ds)
    M = L // 2
    N = L - M + 1

    R_ = np.zeros((N, N), dtype=complex)

    r = data_ds
    for j0 in range(1, M + 1):
        H_temp = r[j0 - 1:j0 + N - 1]
        temp = np.outer(H_temp, H_temp.conj())
        R_ += temp

    R_ /= M

    U, S, _ = np.linalg.svd(R_, full_matrices=False)

    En = U[:, nt + 1:]
    p_music3 = np.zeros(len(time_tr), dtype=complex)


    for j0, t_sample in enumerate(time_tr):

        av = np.exp(-1j * 2.0 * np.pi * freq_ds[:N] * t_sample)

        p_music3[j0] = (np.conj(av).T @ av) / (np.conj(av).T @ En @ En.conj().T @ av)

    return p_music3, time_tr


