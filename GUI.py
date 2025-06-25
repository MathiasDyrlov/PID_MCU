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
    def __init__(self, master, port="COM5", baudrate=115200):
        self.master = master
        self.master.title("SCADA Monitor")
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.logging = False
        self.ser = None
        self.buffer = ""
        self.log_file = None
        self.csv_writer = None

        self.data = {
            "ADC_GEN": "0", "ADC_BUCK": "0", "ADC_BOOST": "0", "ADC_PV": "0", "ADC_OUTPUT": "0",
            "DUTY0": "0", "DUTY1": "0", "DUTY2": "0", "DUTY3": "0", "DUTY4": "0"
        }

        self.trim_value = tk.IntVar(value=512)
        self.trim2_value = tk.IntVar(value=490)
        self.trim3_value = tk.IntVar(value=512)
        self.trim4_value = tk.IntVar(value=410)

        self.resistance_value = tk.DoubleVar(value=300.0)

        self.plot_data = {
            "ADC_GEN": [], "ADC_BUCK": [], "ADC_BOOST": [], "ADC_PV": [], "ADC_OUTPUT": [],
            "DUTY0": [], "DUTY1": [], "DUTY2": [], "DUTY3": [], "DUTY4": [],
            "Power": [], "time": []
        }

        self.start_time = datetime.datetime.now()

        self.setup_gui()
        self.connect_serial()
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

    def setup_gui(self):
        outer_frame = ttk.Frame(self.master)
        outer_frame.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(outer_frame)
        scrollbar = ttk.Scrollbar(outer_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        main_frame = ttk.Frame(scrollable_frame)
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.labels = {}
        for key in self.data:
            label = ttk.Label(control_frame, text=f"{key}: {self.data[key]}", font=("Arial", 12))
            label.pack(anchor=tk.W, pady=2)
            self.labels[key] = label

        ttk.Label(control_frame, text="Setpoints:", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=(10, 0))
        trim_frame = ttk.Frame(control_frame)
        trim_frame.pack(anchor=tk.W)

        self.trim_labels = []

        def make_slider(parent, label_text, var, cmd, label_var_list, min_val, max_val):
            frame = ttk.Frame(parent)
            frame.pack(side=tk.LEFT, padx=5)
            ttk.Label(frame, text=label_text, font=("Arial", 10)).pack()
            slider = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.VERTICAL,
                              variable=var, resolution=1, showvalue=0,
                              length=200, width=30, sliderlength=25,
                              command=cmd)
            slider.pack()
            lbl = ttk.Label(frame, text=str(var.get()), font=("Arial", 10))
            lbl.pack()

            def update_label(*args):
                lbl.config(text=str(var.get()))
            var.trace_add("write", update_label)

            label_var_list.append(lbl)

        make_slider(trim_frame, "Generator Trim", self.trim_value,
                    lambda val: self.send_trim(val, start_char='T'), self.trim_labels,
                    min_val=492, max_val=532)

        make_slider(trim_frame, "Buck Trim", self.trim2_value,
                    lambda val: self.send_trim(val, start_char='U'), self.trim_labels,
                    min_val=470, max_val=510)

        make_slider(trim_frame, "Boost Trim", self.trim3_value,
                    lambda val: self.send_trim(val, start_char='V'), self.trim_labels,
                    min_val=492, max_val=532)

        make_slider(trim_frame, "PV Trim", self.trim4_value,
                    lambda val: self.send_trim(val, start_char='W'), self.trim_labels,
                    min_val=390, max_val=430)

        ttk.Button(control_frame, text="Reset Setpoints", command=self.reset_trim).pack(anchor=tk.W, pady=5)

        resistance_frame = ttk.Frame(control_frame)
        resistance_frame.pack(anchor=tk.W, pady=10)
        ttk.Label(resistance_frame, text="Load Resistance (Ω):", font=("Arial", 10)).pack(side=tk.LEFT)
        resistance_entry = ttk.Entry(resistance_frame, textvariable=self.resistance_value, width=7)
        resistance_entry.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red")
        self.status_label.pack(anchor=tk.W, pady=5)

        ttk.Button(control_frame, text="Start", command=self.send_start).pack(anchor=tk.W, pady=2)
        ttk.Button(control_frame, text="Stop", command=self.send_stop).pack(anchor=tk.W, pady=2)
        self.toggle_btn = ttk.Button(control_frame, text="Start Logging", command=self.toggle_logging)
        self.toggle_btn.pack(anchor=tk.W, pady=10)
        ttk.Button(control_frame, text="Quit", command=self.quit_app).pack(anchor=tk.W, pady=5)

        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312)
        self.ax3 = self.fig.add_subplot(313)

        self.line_adc_gen, = self.ax1.plot([], [], label="ADC_GEN")
        self.line_adc_buck, = self.ax1.plot([], [], label="ADC_BUCK")
        self.line_adc_boost, = self.ax1.plot([], [], label="ADC_BOOST")
        self.line_adc_pv, = self.ax1.plot([], [], label="ADC_PV")
        self.ax1.set_title("Voltage Inputs")
        self.ax1.set_ylabel("Voltage (V)")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        self.line_duty0, = self.ax2.plot([], [], label="GEN", color="orange")
        self.line_duty1, = self.ax2.plot([], [], label="BUCK", color="purple")
        self.line_duty2, = self.ax2.plot([], [], label="BOOST", color="blue")
        self.line_duty3, = self.ax2.plot([], [], label="PV_PID", color="green")
        self.line_duty4, = self.ax2.plot([], [], label="PV_MPPT", color="red")
        self.ax2.set_title("PWM Duty Cycles")
        self.ax2.set_ylabel("Duty (%)")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        self.line_power, = self.ax3.plot([], [], label="Power", color="darkgreen")
        self.ax3.set_title("Power Output")
        self.ax3.set_ylabel("Watts")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.legend(loc="upper right")
        self.ax3.grid(True)

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
            parts = [p.strip() for p in msg.strip().split(';') if p.strip() != '']
            if len(parts) != 10:
                print(f"Ignoring malformed message (expected 10 parts, got {len(parts)}): {msg}")
                return

            adc_gen, adc_buck, adc_boost, adc_pv, adc_output, duty0, duty1, duty2, duty3, duty4 = map(int, parts[:10])

            self.data.update({
                "ADC_GEN": str(adc_gen),
                "ADC_BUCK": str(adc_buck),
                "ADC_BOOST": str(adc_boost),
                "ADC_PV": str(adc_pv),
                "ADC_OUTPUT": str(adc_output),
                "DUTY0": str(duty0),
                "DUTY1": str(duty1),
                "DUTY2": str(duty2),
                "DUTY3": str(duty3),
                "DUTY4": str(duty4),
            })

            self.master.after(0, self.update_labels)
            now = (datetime.datetime.now() - self.start_time).total_seconds()
            self.plot_data["time"].append(now)

            voltage_output = adc_output * (15.0 / 512)
            R_load = self.resistance_value.get()
            power = (voltage_output ** 2) / R_load if R_load > 0 else 0

            duty_converted = [
                (duty0 * 100) / 1023,
                (duty1 * 100) / 1023,
                (duty2 * 100) / 1023,
                (duty3 * 100) / 1023,
                (duty4 * 100) / 255
            ]

            self.plot_data["ADC_GEN"].append(adc_gen * (15.0 / 512))
            self.plot_data["ADC_BUCK"].append(adc_buck * (5.0 / 512))
            self.plot_data["ADC_BOOST"].append(adc_boost * (15.0 / 512))
            self.plot_data["ADC_PV"].append(adc_pv * 0.0122)
            self.plot_data["ADC_OUTPUT"].append(voltage_output)

            self.plot_data["DUTY0"].append(duty_converted[0])
            self.plot_data["DUTY1"].append(duty_converted[1])
            self.plot_data["DUTY2"].append(duty_converted[2])
            self.plot_data["DUTY3"].append(duty_converted[3])
            self.plot_data["DUTY4"].append(duty_converted[4])
            self.plot_data["Power"].append(power)

            if self.logging and self.csv_writer:
                row = [
                    now,
                    self.plot_data["ADC_GEN"][-1],
                    self.plot_data["ADC_BUCK"][-1],
                    self.plot_data["ADC_BOOST"][-1],
                    self.plot_data["ADC_PV"][-1],
                    self.plot_data["ADC_OUTPUT"][-1],
                    self.plot_data["DUTY0"][-1],
                    self.plot_data["DUTY1"][-1],
                    self.plot_data["DUTY2"][-1],
                    self.plot_data["DUTY3"][-1],
                    self.plot_data["DUTY4"][-1],
                    power
                ]
                self.csv_writer.writerow(row)

        except Exception as e:
            print(f"Failed to parse message: {e}")

    def update_labels(self):
        for k, lbl in self.labels.items():
            lbl.config(text=f"{k}: {self.data[k]}")

    def update_plot(self, i):
        t = self.plot_data["time"][-100:]
        self.line_adc_gen.set_data(t, self.plot_data["ADC_GEN"][-100:])
        self.line_adc_buck.set_data(t, self.plot_data["ADC_BUCK"][-100:])
        self.line_adc_boost.set_data(t, self.plot_data["ADC_BOOST"][-100:])
        self.line_adc_pv.set_data(t, self.plot_data["ADC_PV"][-100:])
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.line_duty0.set_data(t, self.plot_data["DUTY0"][-100:])
        self.line_duty1.set_data(t, self.plot_data["DUTY1"][-100:])
        self.line_duty2.set_data(t, self.plot_data["DUTY2"][-100:])
        self.line_duty3.set_data(t, self.plot_data["DUTY3"][-100:])
        self.line_duty4.set_data(t, self.plot_data["DUTY4"][-100:])
        self.ax2.relim()
        self.ax2.autoscale_view()

        self.line_power.set_data(t, self.plot_data["Power"][-100:])
        self.ax3.relim()
        self.ax3.autoscale_view()

        self.canvas.draw()

    def toggle_logging(self):
        if not self.logging:
            path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV Files", "*.csv")])
            if path:
                self.log_file = open(path, mode='w', newline='')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow([
                    "Time",
                    "ADC_GEN (V)", "ADC_BUCK (V)", "ADC_BOOST (V)", "ADC_PV (V)", "ADC_OUTPUT (V)",
                    "DUTY0 (%)", "DUTY1 (%)", "DUTY2 (%)", "DUTY3 (%)", "DUTY4 (%)",
                    "Power (W)"
                ])
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
                index = {'T': 0, 'U': 1, 'V': 2, 'W': 3}.get(start_char, 0)
                self.trim_labels[index].config(text=str(value))
        except Exception as e:
            print(f"Failed to send trim ({start_char}): {e}")

    def reset_trim(self):
        self.trim_value.set(512)
        self.trim2_value.set(490)
        self.trim3_value.set(512)
        self.trim4_value.set(410)
        self.send_trim(512, 'T')
        self.send_trim(490, 'U')
        self.send_trim(512, 'V')
        self.send_trim(410, 'W')

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
        app = SerialMonitorApp(root)
        root.protocol("WM_DELETE_WINDOW", app.quit_app)
        root.mainloop()
    except KeyboardInterrupt:
        sys.exit(0)
