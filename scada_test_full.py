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
    def __init__(self, master, port="COM3", baudrate=115200):
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
        self.trim2_value = tk.IntVar(value=512)
        self.trim3_value = tk.IntVar(value=512)

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
        main_frame = ttk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.labels = {}
        for key in ["ADC0", "ADC1", "ADC2", "ADC3", "ADC4", "DUTY0", "DUTY1"]:
            label = ttk.Label(control_frame, text=f"{key}: {self.data[key]}", font=("Arial", 14))
            label.pack(anchor=tk.W, pady=2)
            self.labels[key] = label

        ttk.Label(control_frame, text="Setpoints:", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=(10, 0))
        trim_frame = ttk.Frame(control_frame)
        trim_frame.pack(anchor=tk.W)

        self.trim_labels = []

        def make_slider(parent, label_text, var, cmd, label_var):
            frame = ttk.Frame(parent)
            frame.pack(side=tk.LEFT, padx=5)
            ttk.Label(frame, text=label_text, font=("Arial", 10)).pack()
            slider = tk.Scale(frame, from_=492, to=532, orient=tk.VERTICAL,
                              variable=var, resolution=1, showvalue=0,
                              length=200, width=30, sliderlength=25,
                              command=cmd)
            slider.pack()
            lbl = ttk.Label(frame, text="512", font=("Arial", 10))
            lbl.pack()
            label_var.append(lbl)

        make_slider(trim_frame, "Trim 1\n(Txxxx)", self.trim_value,
                    lambda val: self.send_trim(val, start_char='T'),
                    self.trim_labels)
        make_slider(trim_frame, "Trim 2\n(Uxxxx)", self.trim2_value,
                    lambda val: self.send_trim(val, start_char='U'),
                    self.trim_labels)
        make_slider(trim_frame, "Trim 3\n(Vxxxx)", self.trim3_value,
                    lambda val: self.send_trim(val, start_char='V'),
                    self.trim_labels)

        ttk.Button(control_frame, text="Reset Setpoints", command=self.reset_trim).pack(anchor=tk.W, pady=5)

        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red")
        self.status_label.pack(anchor=tk.W, pady=5)

        ttk.Button(control_frame, text="Start", command=self.send_start).pack(anchor=tk.W, pady=2)
        ttk.Button(control_frame, text="Stop", command=self.send_stop).pack(anchor=tk.W, pady=2)
        self.toggle_btn = ttk.Button(control_frame, text="Start Logging", command=self.toggle_logging)
        self.toggle_btn.pack(anchor=tk.W, pady=10)
        ttk.Button(control_frame, text="Quit", command=self.quit_app).pack(anchor=tk.W, pady=5)

        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax1 = self.fig.add_subplot(411)
        self.ax2 = self.fig.add_subplot(412)
        self.ax3 = self.fig.add_subplot(413)
        self.ax4 = self.fig.add_subplot(414)

        self.line_adc0, = self.ax1.plot([], [], label="Rectifier Output")
        self.line_adc1, = self.ax1.plot([], [], label="Buck Output")
        self.line_adc2, = self.ax1.plot([], [], label="Boost Output")
        self.ax1.set_title("Live Voltage Data")
        self.ax1.set_ylabel("Voltage (V)")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        self.line_duty0, = self.ax2.plot([], [], label="Generator", color="orange")
        self.line_duty1, = self.ax2.plot([], [], label="Buck", color="purple")
        self.ax2.set_title("PWM Duty Cycles")
        self.ax2.set_ylabel("Duty (%)")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        self.line_power, = self.ax3.plot([], [], label="Power", color="green")
        self.ax3.set_title("Power Output (W)")
        self.ax3.set_ylabel("Power (W)")
        self.ax3.legend(loc="upper right")
        self.ax3.grid(True)

        self.line_load, = self.ax4.plot([], [], label="Load Resistance", color="red")
        self.ax4.set_title("Load Resistance (Ohms)")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("Resistance (Ω)")
        self.ax4.legend(loc="upper right")
        self.ax4.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000, cache_frame_data=False)
        self.fig.tight_layout(pad=2.0)

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
                return
            adc0, adc1, adc2, duty0, duty1, adc3, adc4 = map(int, parts)

            self.data.update({"ADC0": str(adc0), "ADC1": str(adc1), "ADC2": str(adc2),
                              "DUTY0": str(duty0), "DUTY1": str(duty1),
                              "ADC3": str(adc3), "ADC4": str(adc4)})

            self.master.after(0, self.update_labels)
            now = (datetime.datetime.now() - self.start_time).total_seconds()
            self.plot_data["time"].append(now)

            voltage_adc0 = adc0 * (15.0 / 512)
            voltage_adc1 = adc1 * (5.0 / 512)
            voltage_adc2 = adc2 * (15.0 / 512)
            voltage_batt = adc3 * (15.0 / 512)
            current_load = adc4 * (20.0 / 1023)
            power = voltage_batt * current_load
            load_resistance = voltage_batt / current_load if current_load > 0 else None

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
            pass

    def update_labels(self):
        for key in self.labels:
            self.labels[key].config(text=f"{key}: {self.data[key]}")

    def update_plot(self, i):
        time_data = self.plot_data["time"][-100:]
        self.line_adc0.set_data(time_data, self.plot_data["ADC0"][-100:])
        self.line_adc1.set_data(time_data, self.plot_data["ADC1"][-100:])
        self.line_adc2.set_data(time_data, self.plot_data["ADC2"][-100:])
        self.ax1.relim(); self.ax1.autoscale_view()

        self.line_duty0.set_data(time_data, self.plot_data["DUTY0"][-100:])
        self.line_duty1.set_data(time_data, self.plot_data["DUTY1"][-100:])
        self.ax2.relim(); self.ax2.autoscale_view()

        self.line_power.set_data(time_data, self.plot_data["Power"][-100:])
        self.ax3.relim(); self.ax3.autoscale_view()

        load_data = [r if r is not None else math.nan for r in self.plot_data["Load"][-100:]]
        self.line_load.set_data(time_data, load_data)
        self.ax4.relim(); self.ax4.autoscale_view()

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

    def send_stop(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b'X')

    def send_trim(self, val, start_char='T'):
        try:
            if self.ser and self.ser.is_open:
                value = int(float(val))
                self.ser.write(f"{start_char}{value:04d}".encode())
                index = {'T': 0, 'U': 1, 'V': 2}.get(start_char, 0)
                self.trim_labels[index].config(text=str(value))
        except Exception as e:
            print(f"Failed to send trim ({start_char}): {e}")

    def reset_trim(self):
        self.trim_value.set(512)
        self.trim2_value.set(512)
        self.trim3_value.set(512)
        self.send_trim(512, 'T')
        self.send_trim(512, 'U')
        self.send_trim(512, 'V')

    def quit_app(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.logging and self.log_file:
            self.log_file.close()
        self.master.destroy()


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = SerialMonitorApp(root, port="COM3", baudrate=115200)
        root.protocol("WM_DELETE_WINDOW", app.quit_app)
        root.mainloop()
    except KeyboardInterrupt:
        sys.exit(0)
