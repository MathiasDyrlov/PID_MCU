import serial
import threading
import tkinter as tk
from tkinter import ttk, filedialog
import datetime
import csv
import sys
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation


class SerialMonitorApp:
    def __init__(self, master, port="COM4", baudrate=115200):
        self.master = master
        self.master.title("SCADA Monitor")
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.logging = False
        self.data = {"ADC0": "0", "ADC1": "0", "ADC2": "0", "DUTY0": "0", "DUTY1": "0", "ADC3": "0", "ADC4": "0"}
        self.ser = None
        self.log_file = None
        self.csv_writer = None
        self.buffer = ""
        self.trim_value = tk.IntVar(value=512)

        self.plot_data = {
            "ADC0": [], "ADC1": [], "ADC2": [],
            "DUTY0": [], "DUTY1": [],
            "ADC3": [], "ADC4": [],
            "Power": [], "Load": [],
            "time": []
        }
        self.start_time = datetime.datetime.now()

        self.setup_gui()
        self.connect_serial()
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def setup_gui(self):
        frame = ttk.Frame(self.master, padding="10")
        frame.grid()

        self.labels = {}
        for i, key in enumerate(["ADC0", "ADC1", "ADC2", "ADC3", "ADC4", "DUTY0", "DUTY1"]):
            label = ttk.Label(frame, text=f"{key}: {self.data[key]}", font=("Arial", 14))
            label.grid(row=i, column=0, sticky=tk.W, pady=5)
            self.labels[key] = label

        # Trim slider
        ttk.Label(frame, text="Trim (Setpoint):", font=("Arial", 12, "bold")).grid(row=7, column=0, sticky=tk.W, pady=(10, 0))

        trim_slider = tk.Scale(
            frame,
            from_=492,
            to=532,
            orient=tk.VERTICAL,
            variable=self.trim_value,
            resolution=1,
            showvalue=0,
            length=300,
            width=40,
            sliderlength=30,
            command=self.send_trim
        )
        trim_slider.grid(row=8, column=0, sticky=tk.W, pady=(0, 5))

        self.trim_value_label = ttk.Label(frame, text="512", font=("Arial", 12))
        self.trim_value_label.grid(row=9, column=0, sticky=tk.W)

        # Reset Trim Button
        self.reset_trim_btn = ttk.Button(frame, text="Reset Setpoint", command=self.reset_trim)
        self.reset_trim_btn.grid(row=10, column=0, pady=(5, 10), sticky=tk.W)

        self.status_label = ttk.Label(frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=11, column=0, sticky=tk.W)

        self.start_btn = ttk.Button(frame, text="Start", command=self.send_start)
        self.start_btn.grid(row=12, column=0, pady=5)

        self.stop_btn = ttk.Button(frame, text="Stop", command=self.send_stop)
        self.stop_btn.grid(row=13, column=0, pady=5)

        self.toggle_btn = ttk.Button(frame, text="Start Logging", command=self.toggle_logging)
        self.toggle_btn.grid(row=14, column=0, pady=10)

        self.quit_btn = ttk.Button(frame, text="Quit", command=self.quit_app)
        self.quit_btn.grid(row=15, column=0, pady=5)

        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(411)  # Live Voltage Data (ADC0-2)
        self.ax2 = self.fig.add_subplot(412)  # PWM Duty Cycles
        self.ax3 = self.fig.add_subplot(413)  # Power Output
        self.ax4 = self.fig.add_subplot(414)  # Load Resistance

        # Plot lines for first graph (ADC0, ADC1, ADC2)
        self.line_adc0, = self.ax1.plot([], [], label="Rectifier Output")
        self.line_adc1, = self.ax1.plot([], [], label="ADC1")
        self.line_adc2, = self.ax1.plot([], [], label="ADC2")
        self.ax1.set_title("Live Voltage Data")
        self.ax1.set_ylabel("Voltage (V)")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        # Plot lines for second graph (DUTY0, DUTY1)
        self.line_duty0, = self.ax2.plot([], [], label="Generator", color="orange")
        self.line_duty1, = self.ax2.plot([], [], label="MPPT", color="purple")
        self.ax2.set_title("PWM Duty Cycles")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Duty (%)")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        # Plot line for third graph (Power)
        self.line_power, = self.ax3.plot([], [], label="Power", color="green")
        self.ax3.set_title("Power Output (W)")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("Power (W)")
        self.ax3.legend(loc="upper right")
        self.ax3.grid(True)

        # Plot line for fourth graph (Load Resistance)
        self.line_load, = self.ax4.plot([], [], label="Load Resistance", color="red")
        self.ax4.set_title("Load Resistance (Ohms)")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("Resistance (Ω)")
        self.ax4.legend(loc="upper right")
        self.ax4.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().grid(row=0, column=1, rowspan=16, padx=10)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000, cache_frame_data=False)
        self.fig.tight_layout(pad=3.0)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.status_label.config(text=f"Connected to {self.port}", foreground="green")
        except serial.SerialException:
            self.status_label.config(text="Serial connection failed", foreground="red")

    def read_serial(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    self.buffer += chunk

                    while 'A' in self.buffer and 'B' in self.buffer:
                        start = self.buffer.find('A') + 1
                        end = self.buffer.find('B')
                        if start < end:
                            message = self.buffer[start:end].strip()
                            self.parse_message(message)
                            self.buffer = self.buffer[end + 1:]
                        else:
                            self.buffer = self.buffer[end + 1:]
            except Exception as e:
                print(f"Serial read error: {e}")

    def parse_message(self, msg):
        try:
            parts = msg.split(';')
            if len(parts) != 7:
                print(f"Invalid message: {msg}")
                return

            adc0, adc1, adc2, duty0, duty1, adc3, adc4 = map(int, parts)

            self.data["ADC0"] = str(adc0)
            self.data["ADC1"] = str(adc1)
            self.data["ADC2"] = str(adc2)
            self.data["DUTY0"] = str(duty0)
            self.data["DUTY1"] = str(duty1)
            self.data["ADC3"] = str(adc3)
            self.data["ADC4"] = str(adc4)

            # Schedule GUI label update on main thread
            self.master.after(0, self.update_labels)

            now = (datetime.datetime.now() - self.start_time).total_seconds()
            self.plot_data["time"].append(now)

            # Convert ADC to voltage (example scales, adjust as needed)
            voltage_adc0 = adc0 * (15.0 / 512)
            voltage_adc1 = adc1 * (5.0 / 1023)
            voltage_adc2 = adc2 * (5.0 / 1023)
            voltage_batt = adc3 * (15.0 / 512)  # battery voltage
            current_load = adc4 * (20.0 / 1023)  # current from hall effect sensor

            # Power calculation (P = V * I)
            power = voltage_batt * current_load

            # Load resistance calculation R = V / I (avoid div by zero)
            if current_load > 0:
                load_resistance = voltage_batt / current_load
            else:
                load_resistance = None  # or float('inf')

            self.plot_data["ADC0"].append(voltage_adc0)
            self.plot_data["ADC1"].append(voltage_adc1)
            self.plot_data["ADC2"].append(voltage_adc2)
            self.plot_data["DUTY0"].append(duty0)
            self.plot_data["DUTY1"].append(duty1)
            self.plot_data["ADC3"].append(voltage_batt)
            self.plot_data["ADC4"].append(current_load)
            self.plot_data["Power"].append(power)
            self.plot_data["Load"].append(load_resistance)

            if self.logging:
                timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                self.csv_writer.writerow([timestamp, adc0, adc1, adc2, duty0, duty1, adc3, adc4, power, load_resistance])

        except ValueError:
            print(f"Value error parsing: {msg}")

    def update_labels(self):
        for key in self.labels:
            self.labels[key].config(text=f"{key}: {self.data[key]}")

    def update_plot(self, i):
        time_data = self.plot_data["time"][-100:]

        # Update voltage plot
        self.line_adc0.set_data(time_data, self.plot_data["ADC0"][-100:])
        self.line_adc1.set_data(time_data, self.plot_data["ADC1"][-100:])
        self.line_adc2.set_data(time_data, self.plot_data["ADC2"][-100:])
        self.ax1.relim()
        self.ax1.autoscale_view()

        # Update duty cycle plot
        self.line_duty0.set_data(time_data, self.plot_data["DUTY0"][-100:])
        self.line_duty1.set_data(time_data, self.plot_data["DUTY1"][-100:])
        self.ax2.relim()
        self.ax2.autoscale_view()

        # Update power plot
        self.line_power.set_data(time_data, self.plot_data["Power"][-100:])
        self.ax3.relim()
        self.ax3.autoscale_view()

        # Update load resistance plot (replace None with NaN for plotting)
        load_data = [r if r is not None else math.nan for r in self.plot_data["Load"][-100:]]
        self.line_load.set_data(time_data, load_data)
        self.ax4.relim()
        self.ax4.autoscale_view()

        self.canvas.draw()

    def toggle_logging(self):
        if not self.logging:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV Files", "*.csv")])
            if filepath:
                self.log_file = open(filepath, mode='w', newline='')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow(["Time", "ADC0", "ADC1", "ADC2", "DUTY0", "DUTY1", "ADC3", "ADC4", "Power (W)", "Load Resistance (Ω)"])
                self.logging = True
                self.toggle_btn.config(text="Stop Logging")
        else:
            self.logging = False
            if self.log_file:
                self.log_file.close()
            self.toggle_btn.config(text="Start Logging")

    def send_start(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b'S')
            print("Sent: S")

    def send_stop(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b'X')
            print("Sent: X")

    def send_trim(self, val):
        try:
            if self.ser and self.ser.is_open:
                value = int(float(val))
                self.ser.write(f"T{value:04d}".encode())
                self.trim_value_label.config(text=str(value))
        except Exception as e:
            print(f"Failed to send trim: {e}")

    def reset_trim(self):
        self.trim_value.set(512)
        self.send_trim(512)

    def quit_app(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.logging and self.log_file:
            self.log_file.close()
        self.master.destroy()


# Run the GUI
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = SerialMonitorApp(root, port="COM4", baudrate=115200)
        root.protocol("WM_DELETE_WINDOW", app.quit_app)
        root.mainloop()
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")
        sys.exit(0)
