import serial
import time
import keysight_ktna as vna
import datetime
import numpy as np
from scipy import io
import matplotlib.pyplot as plt
import os

import conv_gpr_v2_Aveen as gpr_

class ArduinoController:
    def __init__(self, port, baudrate, timeout=1):
        # port: COM port of the Arduino.
        # baudrate: Communication baud rate.
        # timeout: Timeout for serial communication in seconds."""
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            print(f"Connected to Arduino on port {port}")
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino on port {port}: {e}")
            self.arduino = None

    def send_order(self, order):
        if self.arduino:
            try:
                self.arduino.write(order.encode())
                time.sleep(0.1)
            except Exception as e:
                print(f"Failed to send order '{order}': {e}")
        else:
            print("Arduino connection is not established.")

    def read_response(self):
        if self.arduino:
            try:
                response = self.arduino.readline().decode('ascii')
                while not response:
                    response = self.arduino.readline().decode('ascii')
                return response
            except Exception as e:
                print(f"Failed to read response: {e}")
                return None
        else:
            print("Arduino connection is not established.")
            return None

    def execute_command(self, order):
        self.send_order(order)
        return self.read_response()

class NetworkAnalyzerController:
    def __init__(self, resource_name, id_query=False, reset=True, options="QueryInstrStatus=True, Simulate=False, Trace=True"):
        self.driver = vna.KtNA(resource_name, id_query, reset, options)
        print("\n Driver Initialized \n")

    def display_identity_info(self):
        """
        Display instrument identity information.
        """
        print('  model:      ', self.driver.identity.instrument_model)
        print('  resource:   ', self.driver.driver_operation.io_resource_descriptor)
        print('  options:    ', self.driver.driver_operation.driver_setup)
        print('\n')

    def perform_basic_operations(self):
        """
        Perform basic operations using classic commands.
        """
        print("Basic Operations using ClassicCommands")
        self.driver.classic_commands.sys_tem.fp_reset()
        self.driver.classic_commands.dis_play.win_dow.window_no = 1
        self.driver.classic_commands.dis_play.win_dow.sta_te = True

    def load_memory_file(self, file_path):
        """
        Load a calibration or memory file into the instrument.
        :param file_path: Path to the file to load.
        """
        self.driver.classic_commands.mme_mory.load(file_name=file_path)

    def trig_channel(self, time_delay_to_read):
        self.driver.classic_commands.sen_se.sw_eep.mode = vna.SweepTriggerMode.SINGLE
        time.sleep(time_delay_to_read)
        self.driver.system.wait_for_operation_complete(datetime.timedelta(0, 2.75, 0))

    def retrieve_data(self):
        self.driver.classic_commands.cal_culate.mea_sure.measurement_no = 1
        data_result1 = self.driver.classic_commands.cal_culate.mea_sure.data_query(vna.MeasurementDataType.COMPLEX_MEAS_DATA)
        f_start = self.driver.classic_commands.sen_se.fre_quency.sta_rt
        f_stop = self.driver.classic_commands.sen_se.fre_quency.stop
        return data_result1, f_start, f_stop

#Todo Do modular the main function please!
def main(file_path_in, file_path_out, n_scan = 40, dsr=40, nt=3, log=0):

    port = 'COM5'
    baudrate = 9600
    timeout = 1

    resource_name = "TCPIP0::scistifrpc19::hislip1,4880::INSTR"
    id_query = False
    reset = True
    options = "QueryInstrStatus=True, Simulate=False, Trace=True"


    # Instantiate the ArduinoController class
    arduino_controller = ArduinoController(port, baudrate, timeout)

    # Check if Arduino is connected before proceeding
    if not arduino_controller.arduino:
        return

    analyzer = NetworkAnalyzerController(resource_name, id_query, reset, options)

    # Check if VNA is connected before proceeding
    if not analyzer.driver:
        return

    # Display identity information
    analyzer.display_identity_info()

    # Perform basic operations
    analyzer.perform_basic_operations()

    # Load a calibration or memory file
    analyzer.load_memory_file(file_path_in)

    order = 'ref'
    response = arduino_controller.execute_command(order)
    if not response:
        return

    # n_scan = 5
    s_ = np.zeros((8192, n_scan), dtype=np.complex128)

    fig, ax = plt.subplots(2, 1, figsize=(15, 6))

    manager = plt.get_current_fig_manager()
    manager.window.wm_geometry("+30+100")  # Replace x_pos and y_pos with actual values

    # Example: Set the position to (100, 200) on the screen
    # manager.window.wm_geometry("+100+200")

    plt.ion()

    h_t_temp = np.zeros((8192, n_scan), dtype=complex)
    TR_      = np.zeros((8192, n_scan), dtype=complex)
    for i0 in range(n_scan):  # step=1000, max 25
        print(f"number of step = {i0}")

        order = 's1'
        response = arduino_controller.execute_command(order)
        if not response:
            return
        # if response:
        #     print(f"Arduino Response: {response}")
        # else:
        #     print("No response from Arduino.")

        time_delay_to_read = 2
        analyzer.trig_channel(time_delay_to_read)

        data_result1, f_start, f_stop = analyzer.retrieve_data()

        # save the data in s_ matrix
        s11 = data_result1[::2] + 1j * data_result1[1::2]
        s_[:, i0] = s11

        if i0 == 0:
            print(f"freq_min = {f_start}")
            print(f"freq_max = {f_stop}")

        gpr, time_, freq = gpr_.gpr_calc(s11, n_freq=8192, freq_min=0.0, freq_max=14.0e9)
        ind_ = time_ < 5.0e-9

        # Use the mask to filter the time and GPR arrays
        time_new = time_[ind_]
        gpr_new = gpr[ind_]

        # plt.plot(time_new, gpr_new, label='gpr')
        # plt.show()

        # h_t_temp[0:len(gpr_new), i0] = 20.0 * np.log10(abs(gpr_new))
        if log == 0:
            h_t_temp[0:len(gpr_new), i0] = gpr_new
        elif log == 1:
            h_t_temp[0:len(gpr_new), i0] = 20*np.log10(abs(gpr_new))

        TR_temp, time_tr = gpr_.hrtr_method(s11, freq, dsr, nt, time_min=0.0e-9, time_max=5.0e-9 + 0.01e-9, dt=0.01e-9)
        if log == 0:
            TR_[0:len(time_tr), i0] = TR_temp
        elif log == 1:
            TR_[0:len(time_tr), i0] = 20*np.log10(TR_temp)

        # x_temp = range(i0 + 1)
        # y_temp = time_new

        if i0 > 0:
            ax[0].clear()
            ax[1].clear()
            # ax.pcolormesh(x_temp, y_temp, abs(h_t_temp[0:len(gpr_new), 0:i0+1]), cmap='turbo')
            # X, Y = np.meshgrid(x_temp, y_temp)
            # ax.contourf(X, Y, abs(h_t_temp[0:len(gpr_new), 0:i0 + 1]), cmap='jet')
            # ax.pcolormesh(x_temp, y_temp, 20*np.log10(abs(h_t_temp[ind_[0]:0:-1, 0:i0+1])), cmap='turbo')
            # ax.contour(X, Y, abs(h_t_temp[0:len(gpr_new), 0:i0 + 1]))

            # plt.subplot(2, 1, 1)
            im1 = ax[0].imshow(abs(h_t_temp[0:len(gpr_new), 0:i0 + 1]), aspect='auto', cmap='jet', interpolation='none',
                       extent=(0, i0 + 1.5, time_new[-1] * 1.0e9, time_new[0] * 1.0e9))

            if i0 == 1:
                plt.colorbar(im1, ax=ax[0])

            ax[0].grid(True)
            ax[0].set_xlabel('Number of Scans')
            ax[0].set_ylabel('Time (ns)')
            ax[0].set_title('Conv-GPR')
            ax[0].tick_params(labelsize=15)


            # plt.subplot(2, 1, 2)
            im2 = ax[1].imshow(abs(TR_[0:len(time_tr),  0:i0 + 1]), aspect='auto', cmap='jet', interpolation='none',
                       extent=(0.5, i0 + 1.5, time_tr[-1] * 1.0e9, time_tr[0] * 1.0e9))

            if i0 == 1:
                plt.colorbar(im2, ax=ax[1])

            ax[1].grid(True)
            ax[1].set_xlabel('Number of Scans')
            ax[1].set_ylabel('Time (ns)')
            ax[1].set_title('TR-GPR')
            ax[1].tick_params(labelsize=15)

            ax[0].set_xlim(0.5, n_scan + 0.5)
            ax[1].set_xlim(0.5, n_scan + 0.5)

            plt.pause(0.1)


    # save in file
    io.savemat(file_path_out, {"data": s_})
    print('wrote!')

    folder_name = 'saved_figures'
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"Folder '{folder_name}' created successfully.")
    else:
        print(f"Folder '{folder_name}' already exists.")

    # Get the current date and time
    current_time = time.strftime("%Y-%m-%d_%H-%M-%S")
    # Construct the filename with date and time
    filename = f"./saved_figures/figure_{current_time}.png"
    plt.ioff()
    fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
    plt.tight_layout()
    # Save the figure
    plt.savefig(filename)
    plt.show()

    # if analyzer.driver is not None:  # Skip close() if constructor failed
    analyzer.driver.close()
    arduino_controller.arduino.close()

def main_source_loc(file_path_in, file_path_out, n_scan = 40, d = 0.1, n_run = 20, freq_min = 3e9, freq_max = 5.5e9):

    port = 'COM13'
    baudrate = 9600
    timeout = 1

    resource_name = "TCPIP0::scistifrpc19::hislip1,4880::INSTR"
    id_query = False
    reset = True
    options = "QueryInstrStatus=True, Simulate=False, Trace=True"


    # Instantiate the ArduinoController class
    arduino_controller = ArduinoController(port, baudrate, timeout)

    # Check if Arduino is connected before proceeding
    if not arduino_controller.arduino:
        return

    analyzer = NetworkAnalyzerController(resource_name, id_query, reset, options)

    # Check if VNA is connected before proceeding
    if not analyzer.driver:
        return

    # Display identity information
    analyzer.display_identity_info()

    # Perform basic operations
    analyzer.perform_basic_operations()

    # Load a calibration or memory file
    analyzer.load_memory_file(file_path_in)

        # n_scan = 5
    nf = 1024 # 8192
    s_ = np.zeros((nf, n_scan), dtype=np.complex128)
    switch_ids = ['1', '2', '3']

    fig = plt.figure(figsize=(3,3))
    plt.ion()

    for j0 in range(n_run):
        print(f"number of step = {j0}")

        for i0 in range(n_scan):  # step=1000, max 25

            order = switch_ids[i0]
            response = arduino_controller.execute_command(order)
            if not response:
                return
            # if response:
            #     print(f"Arduino Response: {response}")
            # else:
            #     print("No response from Arduino.")

            time_delay_to_read = 0.0
            analyzer.trig_channel(time_delay_to_read)

            data_result1, f_start, f_stop = analyzer.retrieve_data()

            # save the data in s_ matrix
            s11 = data_result1[::2] + 1j * data_result1[1::2]
            s_[:, i0] = s11

        if j0 == 0:
            print(f"freq_min = {f_start}")
            print(f"freq_max = {f_stop}")

        freq = np.linspace(f_start, f_stop, nf)

        ind_ = (freq >= freq_min) & (freq <= freq_max)
        freq = freq[ind_]
        s_2 = s_[ind_, :]

        c = 3e8
        lambda_ = c / freq

        phi1 = np.angle(s_2[:, 1])  # Phase of S[:,2] for Antenna 1 (adjusted to Python's zero-based indexing)
        phi2 = np.angle(s_2[:, 0])  # Phase of S[:,1] for Antenna 2 (adjusted to Python's zero-based indexing)

        delta_phi = phi2 - phi1

        delta_phi_unwrapped = (delta_phi + np.pi) % (2 * np.pi) - np.pi

        theta = np.arcsin((delta_phi_unwrapped * lambda_) / (2 * np.pi * d)) * (180 / np.pi)

        ax = plt.subplot(111, polar=True)
        ax.clear()
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_thetamin(-90)
        ax.set_thetamax(90)
        ax.set_rlim(bottom=-20, top=0)
        ax.set_yticklabels([])
        ax.vlines(np.mean(theta)/180*np.pi, -20,0)
        # ax.text(-2, -14, "{} deg".format(np.mean(theta)))
        plt.draw()
        plt.pause(0.1)
        # plt.show()
    # plt.figure()
    # plt.plot(freq, theta)
    # plt.xlabel('Frequency')  # Add axis labels if needed
    # plt.ylabel('Theta')  # Add axis labels if needed
    # plt.title('Theta vs. Frequency')  # Add a title if needed
    # plt.grid(True)  # Optional: Adds a grid to the plot

        # Display the mean of theta
        print(np.mean(theta))

    plt.ioff()
    # plt.show()

    # save in file
    io.savemat(file_path_out, {"data": s_})
    print('wrote!')

    # if analyzer.driver is not None:  # Skip close() if constructor failed
    analyzer.driver.close()
    arduino_controller.arduino.close()

def main_source_loc_MUSIC_01(file_path_in, file_path_out, n_scan = 4, d = 0.1, N = 3, n_run = 20, freq_min = 2.0e9, freq_max = 2.1e9, K_obj = 1):

    port = 'COM13'
    baudrate = 9600
    timeout = 1

    resource_name = "TCPIP0::scistifrpc19::hislip1,4880::INSTR"
    id_query = False
    reset = True
    options = "QueryInstrStatus=True, Simulate=False, Trace=True"


    # Instantiate the ArduinoController class
    arduino_controller = ArduinoController(port, baudrate, timeout)

    # Check if Arduino is connected before proceeding
    if not arduino_controller.arduino:
        return

    analyzer = NetworkAnalyzerController(resource_name, id_query, reset, options)

    # Check if VNA is connected before proceeding
    if not analyzer.driver:
        return

    # Display identity information
    analyzer.display_identity_info()

    # Perform basic operations
    analyzer.perform_basic_operations()

    # Load a calibration or memory file
    analyzer.load_memory_file(file_path_in)

        # n_scan = 5
    nf = 1024 # 8192
    s_ = np.zeros((nf, n_scan), dtype=np.complex128)
    switch_ids = ['1', '2', '3', '4']

    ax = plt.figure(figsize=(12, 3))
    plt.ion()

    for j0 in range(n_run):
        print(f"number of step = {j0}")

        for i0 in range(n_scan):  # step=1000, max 25

            order = switch_ids[i0]
            response = arduino_controller.execute_command(order)
            if not response:
                return
            # if response:
            #     print(f"Arduino Response: {response}")
            # else:
            #     print("No response from Arduino.")

            time_delay_to_read = 0.0
            analyzer.trig_channel(time_delay_to_read)

            data_result1, f_start, f_stop = analyzer.retrieve_data()

            # save the data in s_ matrix
            s11 = data_result1[::2] + 1j * data_result1[1::2]
            s_[:, i0] = s11

        if j0 == 0:
            print(f"freq_min = {f_start}")
            print(f"freq_max = {f_stop}")

        freq = np.linspace(f_start, f_stop, nf)

        ind_ = (freq >= freq_min) & (freq <= freq_max)
        freq = freq[ind_]
        s_2 = s_[ind_, :]
        # s_2 = s_2/np.abs(s_2)
        # nf_2 = len(freq)

        # c = 3e8
        # lambda_ = c / freq
        # k = 2.0*np.pi/lambda_

        s_2 = s_2.T  # Transpose of S
        # Covariance matrix
        R = (s_2 @ s_2.conj().T) / len(freq)

        # Eigen-decomposition of R
        D, Q = np.linalg.eig(R)  # D: Eigenvalues, Q: Eigenvectors

        # Sort eigenvalues in descending order and reorder eigenvectors
        idx = np.argsort(D)[::-1]  # Indices to sort D in descending order
        D = D[idx]  # Sort eigenvalues
        # print(abs(D))
        Q = Q[:, idx]  # Reorder eigenvectors accordingly

        # Noise subspace
        Qn = Q[:, K_obj:]  # Equivalent to Q(:, K+1:end) in MATLAB

        # Parameters
        theta = np.arange(-90, 90, 0.1)  # Angle range from -90 to 90 degrees
        Pmusic = np.zeros(len(theta), dtype=float)  # Initialize MUSIC spectrum

        # MUSIC Spectrum Calculation
        for i, angle in enumerate(theta):
            a = np.exp(-1j * 2 * np.pi * d * np.arange(N).reshape(-1, 1) * np.sin(np.radians(angle)))
            Pmusic[i] = 1 / np.abs(a.conj().T @ (Qn @ Qn.conj().T) @ a)

        # Normalize and convert the MUSIC spectrum to dB scale
        Pmusic_dB = 20 * np.log10(Pmusic / np.max(Pmusic))

        # Plot the MUSIC spectrum
        ax.clear()
        plt.plot(theta, Pmusic_dB)
        plt.xlabel('Angle (degrees)')
        plt.ylabel('Spatial Spectrum (dB)')
        plt.title('MUSIC Spectrum')
        plt.grid(True)
        plt.pause(0.1)

    plt.ioff()
    # plt.show()

    # save in file
    io.savemat(file_path_out, {"data": s_})
    print('wrote!')

    # if analyzer.driver is not None:  # Skip close() if constructor failed
    analyzer.driver.close()
    arduino_controller.arduino.close()

def main_source_loc_GCC_01(file_path_in, file_path_out, n_scan = 4, d = 0.1, N = 4, n_run = 20, freq_min = 1.2e9, freq_max = 5.0e9, K_obj = 1):

    port = 'COM13'
    baudrate = 9600
    timeout = 1

    resource_name = "TCPIP0::scistifrpc19::hislip1,4880::INSTR"
    id_query = False
    reset = True
    options = "QueryInstrStatus=True, Simulate=False, Trace=True"


    # Instantiate the ArduinoController class
    arduino_controller = ArduinoController(port, baudrate, timeout)

    # Check if Arduino is connected before proceeding
    if not arduino_controller.arduino:
        return

    analyzer = NetworkAnalyzerController(resource_name, id_query, reset, options)

    # Check if VNA is connected before proceeding
    if not analyzer.driver:
        return

    # Display identity information
    analyzer.display_identity_info()

    # Perform basic operations
    analyzer.perform_basic_operations()

    # Load a calibration or memory file
    analyzer.load_memory_file(file_path_in)

        # n_scan = 5
    nf = 1024 # 8192
    s_ = np.zeros((nf, n_scan), dtype=np.complex128)
    switch_ids = ['1', '2', '3', '4']

    ax = plt.figure(figsize=(12, 3))
    plt.ion()

    for j0 in range(n_run):
        print(f"number of step = {j0}")

        for i0 in range(n_scan):  # step=1000, max 25

            order = switch_ids[i0]
            response = arduino_controller.execute_command(order)
            if not response:
                return
            # if response:
            #     print(f"Arduino Response: {response}")
            # else:
            #     print("No response from Arduino.")

            time_delay_to_read = 0.0
            analyzer.trig_channel(time_delay_to_read)

            data_result1, f_start, f_stop = analyzer.retrieve_data()

            # save the data in s_ matrix
            s11 = data_result1[::2] + 1j * data_result1[1::2]
            s_[:, i0] = s11

        if j0 == 0:
            print(f"freq_min = {f_start}")
            print(f"freq_max = {f_stop}")

        freq = np.linspace(f_start, f_stop, nf)

        ind_ = (freq >= freq_min) & (freq <= freq_max)
        freq = freq[ind_]
        s_2 = s_[ind_, :]
        # s_2 = s_2/np.abs(s_2)
        # nf_2 = len(freq)

        # c = 3e8
        # lambda_ = c / freq
        # k = 2.0*np.pi/lambda_

        s_2 = s_2.T  # Transpose of S
        # Covariance matrix
        R = (s_2 @ s_2.conj().T) / len(freq)

        # Eigen-decomposition of R
        D, Q = np.linalg.eig(R)  # D: Eigenvalues, Q: Eigenvectors

        # Sort eigenvalues in descending order and reorder eigenvectors
        idx = np.argsort(D)[::-1]  # Indices to sort D in descending order
        D = D[idx]  # Sort eigenvalues
        # print(abs(D))
        Q = Q[:, idx]  # Reorder eigenvectors accordingly

        # Noise subspace
        Qn = Q[:, K_obj:]  # Equivalent to Q(:, K+1:end) in MATLAB

        # Parameters
        theta = np.arange(-90, 90, 0.1)  # Angle range from -90 to 90 degrees
        Pmusic = np.zeros(len(theta), dtype=float)  # Initialize MUSIC spectrum

        # MUSIC Spectrum Calculation
        for i, angle in enumerate(theta):
            a = np.exp(-1j * 2 * np.pi * d * np.arange(N).reshape(-1, 1) * np.sin(np.radians(angle)))
            Pmusic[i] = 1 / np.abs(a.conj().T @ (Qn @ Qn.conj().T) @ a)

        # Normalize and convert the MUSIC spectrum to dB scale
        Pmusic_dB = 20 * np.log10(Pmusic / np.max(Pmusic))

        # Plot the MUSIC spectrum
        ax.clear()
        plt.plot(theta, Pmusic_dB)
        plt.xlabel('Angle (degrees)')
        plt.ylabel('Spatial Spectrum (dB)')
        plt.title('MUSIC Spectrum')
        plt.grid(True)
        plt.pause(0.1)

    plt.ioff()
    # plt.show()

    # save in file
    io.savemat(file_path_out, {"data": s_})
    print('wrote!')

    # if analyzer.driver is not None:  # Skip close() if constructor failed
    analyzer.driver.close()
    arduino_controller.arduino.close()




if __name__ == "__main__":

    # Landmine
    file_path_ = 'C:\\Users\\Public\\Documents\\Network Analyzer\\cal_01_11_2024_2_port_8192_0_14GHz.csa'
    # file_path_ = 'C:\\Users\\Public\\Documents\\Network Analyzer\\cal_01_11_2024_2_port_8192_0_14GHz_s21.csa'
    # file_path_ = 'C:\\Users\\Public\\Documents\\Network Analyzer\\cal_01_11_2024_2_port_8192_0_14GHz_s22.csa'

    file_path_mat = 'test_s11_disk_under_ground_2.mat'

    main(file_path_, file_path_mat, n_scan=40, dsr=40, nt=21, log=1)

    # # Source Localization
    # file_path_ = 'C:\\Users\\Public\\Documents\\Network Analyzer\\cal_01_11_2024_port1_Switch_s21_RBW_5k.csa'
    #
    # file_path_mat = 'new_temp.mat'
    #
    # main_source_loc(file_path_, file_path_mat, n_scan = 3, d = 0.1, n_run = 100, freq_min = 2.0e9, freq_max = 2.01e9)

    # # Source Localization
    # file_path_ = 'C:\\Users\\Public\\Documents\\Network Analyzer\\cal_01_11_2024_port1_Switch_s21_TG.csa'
    #
    # file_path_mat = 'new_temp.mat'
    #
    # main_source_loc_MUSIC_01(file_path_, file_path_mat, n_scan = 4, d = 0.1, N = 4, n_run = 10, freq_min = 2.0e9, freq_max = 2.01e9, K_obj = 1)
