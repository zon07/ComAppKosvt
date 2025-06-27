import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from threading import Thread
import time

class SensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Sensor Monitor")
        
        self.serial_port = None
        self.sensors_count = 0
        self.sensor_data = {}
        self.sensor_widgets = {}
        self.current_sensor_index = 0
        self.sensor_order = []
        self.sensor_names = [
            "Датчик Dust",
            "Датчик Temp 1",
            "Датчик Humid 1",
            "Датчик Temp 2",
            "Датчик Humid 2"
        ]
        
        self.create_widgets()
        self.update_ports_list()
        
        self.running = True
        self.read_thread = Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        
        self.update_data()

    def create_widgets(self):
        # Control panel
        control_frame = tk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # COM port selection
        tk.Label(control_frame, text="COM порт:").pack(side=tk.LEFT)
        
        self.port_combobox = ttk.Combobox(control_frame, state="readonly", width=15)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(control_frame, text="Подключиться", command=self.connect_to_port)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.refresh_btn = tk.Button(control_frame, text="Обновить список", command=self.update_ports_list)
        self.refresh_btn.pack(side=tk.LEFT)
        
        self.status_label = tk.Label(control_frame, text="Статус: Не подключено")
        self.status_label.pack(side=tk.RIGHT)
        
        # Notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # System tab
        self.system_tab = tk.Frame(self.notebook)
        self.notebook.add(self.system_tab, text="Система")
        
        self.system_text = tk.Text(self.system_tab, wrap=tk.WORD)
        self.system_text.pack(fill=tk.BOTH, expand=True)
        self.system_text.insert(tk.END, "Ожидание подключения к устройству...\n")
        self.system_text.config(state=tk.DISABLED)
        
        # Context menu
        self.context_menu = tk.Menu(self.root, tearoff=0)
        self.context_menu.add_command(label="Копировать", command=self.copy_from_log)
        self.context_menu.add_command(label="Очистить лог", command=self.clear_log)
        self.system_text.bind("<Button-3>", self.show_context_menu)

    def show_context_menu(self, event):
        try:
            self.context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.context_menu.grab_release()

    def copy_from_log(self):
        try:
            if self.system_text.tag_ranges(tk.SEL):
                selected_text = self.system_text.get(tk.SEL_FIRST, tk.SEL_LAST)
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
        except Exception as e:
            self.log_message(f"Ошибка копирования: {str(e)}")

    def clear_log(self):
        self.system_text.config(state=tk.NORMAL)
        self.system_text.delete(1.0, tk.END)
        self.system_text.config(state=tk.DISABLED)
    
    def update_ports_list(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.set(ports[0])
        else:
            self.port_combobox.set('')
            self.status_label.config(text="Статус: COM-порты не найдены")
    
    def connect_to_port(self):
        port_name = self.port_combobox.get()
        if not port_name:
            messagebox.showerror("Ошибка", "Не выбран COM-порт")
            return
            
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        try:
            self.serial_port = serial.Serial(port_name, baudrate=9600, timeout=1)
            self.status_label.config(text=f"Статус: Подключено к {port_name}")
            self.log_message(f"Успешное подключение к {port_name}")
            self.request_sensor_count()
        except Exception as e:
            self.status_label.config(text="Статус: Ошибка подключения")
            self.log_message(f"Ошибка подключения к {port_name}: {str(e)}")
            messagebox.showerror("Ошибка", f"Не удалось подключиться к {port_name}:\n{str(e)}")
    
    def log_message(self, message):
        self.system_text.config(state=tk.NORMAL)
        self.system_text.insert(tk.END, message + "\n")
        self.system_text.see(tk.END)
        self.system_text.config(state=tk.DISABLED)
    
    def send_request_with_delay(self, request):
        if not self.serial_port or not self.serial_port.is_open:
            return False
            
        for byte in request:
            self.serial_port.write(bytes([byte]))
            time.sleep(0.005)
            
        self.serial_port.flush()
        return True
    
    def request_sensor_count(self):
        request = bytes([0x41, 0x41, 0x02, 0x00, 0x01])
        if self.send_request_with_delay(request):
            self.log_message("Отправлен запрос количества датчиков: 41 41 02 00 01")
    
    def request_sensor_data(self, sensor_index):
        if not 0 <= sensor_index < self.sensors_count:
            return
            
        self.current_sensor_index = sensor_index
        
        request = bytes([0x41, 0x41, 0x04, 0x00, 0x02, 0x00, sensor_index])
        if self.send_request_with_delay(request):
            self.log_message(f"Запрос данных датчика {sensor_index} отправлен")
    
    def read_serial_data(self):
        while self.running:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(1)
                    continue
                    
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.process_received_data(data)
                    
            except Exception as e:
                self.log_message(f"Ошибка в потоке чтения: {str(e)}")
                time.sleep(1)
                
            time.sleep(0.1)
    
    def process_received_data(self, data):
        self.log_message(f"Получены данные: {data.hex(' ')}")
        
        pos = 0
        while pos <= len(data) - 4:
            if data[pos] == 0x41 and data[pos+1] == 0x41:
                payload_len = data[pos+2]
                message_end = pos + 3 + payload_len
                
                if message_end > len(data):
                    break
                    
                # Sensor count response
                if payload_len == 1:
                    self.sensors_count = data[pos+3]
                    self.log_message(f"Получено количество датчиков: {self.sensors_count}")
                    self.sensor_order = list(range(self.sensors_count))
                    self.root.after(0, self.create_sensor_tabs_in_order)
                    pos = message_end
                    continue
                    
                # Sensor data response
                if payload_len == 8 and len(data[pos:message_end]) >= 11:
                    try:
                        sensor_index = self.current_sensor_index
                        sensor_type = data[pos+3]
                        
                        value = data[pos+4] | (data[pos+5] << 8)
                        gain = data[pos+6] | (data[pos+7] << 8)
                        offset = data[pos+8] | (data[pos+9] << 8)
                        is_valid = data[pos+10]
                        
                        self.sensor_data[sensor_index] = {
                            'type': sensor_type,
                            'value': value,
                            'gain': gain,
                            'offset': offset,
                            'is_valid': is_valid
                        }
                        
                        self.log_message(
                            f"Данные датчика {sensor_index}:\n"
                            f"Тип: {self.get_sensor_type_name(sensor_type)}\n"
                            f"Значение: {value}\n"
                            f"Усиление: {gain}\n"
                            f"Смещение: {offset}\n"
                            f"Статус: {'VALID' if is_valid else 'INVALID'}"
                        )
                        
                        # Update display immediately
                        self.root.after(0, lambda idx=sensor_index: self.update_sensor_display(idx))
                        
                        # Request next sensor in fixed order
                        next_idx = (self.sensor_order.index(sensor_index) + 1) % self.sensors_count
                        self.current_sensor_index = self.sensor_order[next_idx]
                        self.request_sensor_data(self.current_sensor_index)
                        
                    except Exception as e:
                        self.log_message(f"Ошибка обработки данных: {str(e)}")
                    
                    pos = message_end
                    continue
                    
            pos += 1

    def create_sensor_tabs_in_order(self):
        """Create tabs in fixed order 0-1-2-3-4 with predefined names"""
        if not hasattr(self, 'notebook') or self.sensors_count <= 0:
            return
            
        # Remove old tabs
        for tab in self.notebook.tabs()[1:]:
            self.notebook.forget(tab)
        
        self.sensor_widgets = {}
        
        # Create tabs in fixed order
        for i in range(self.sensors_count):
            tab = ttk.Frame(self.notebook)
            
            # Use predefined name if available
            tab_name = self.sensor_names[i] if i < len(self.sensor_names) else f"Датчик {i}"
            self.notebook.add(tab, text=tab_name)
            
            # Header
            ttk.Label(tab, text=f"Данные {tab_name}", 
                    font=('Arial', 12, 'bold')).pack(pady=5)
            
            self.create_sensor_display(tab, i)
            ttk.Button(tab, text="Обновить",
                    command=lambda idx=i: self.update_single_sensor(idx)).pack(pady=5)
            
            # Request data immediately
            self.request_sensor_data(i)

    def get_sensor_type_name(self, type_code):
        sensor_types = {
            0: "UNDEFINED",
            1: "TEMPERATURE",
            2: "PRESSURE",
            3: "HUMIDITY",
            4: "DUST",
            5: "COUNT"
        }
        return sensor_types.get(type_code, f"UNKNOWN ({type_code})")
    
    def update_single_sensor(self, sensor_index):
        if 0 <= sensor_index < self.sensors_count:
            prev_index = self.current_sensor_index
            self.current_sensor_index = sensor_index
            self.request_sensor_data(sensor_index)
            self.current_sensor_index = prev_index
            self.log_message(f"Ручной запрос данных датчика {sensor_index}")
   
    def get_sensor_units(self, type_code):
        units = {
            0: "",
            1: "°C",
            2: "kPa",
            3: "%",
            4: "μg/m³",
            5: ""
        }
        return units.get(type_code, "")

    def calculate_processed_value(self, data):
        try:
            if data['gain'] == 0:
                return 0
            return (data['value'] / data['gain']) - data['offset']
        except:
            return 0   
   
    def create_sensor_display(self, parent, sensor_index):
        frame = tk.Frame(parent)
        frame.pack(fill=tk.X, padx=10, pady=5)
        
        sensor_widgets = {}
        
        labels_info = [
            ("Тип:", 'type'),
            ("Сырое значение:", 'value'),
            ("Усиление:", 'gain'),
            ("Смещение:", 'offset'),
            ("Результат:", 'processed'),
            ("Статус:", 'status')
        ]
        
        for row, (label_text, key) in enumerate(labels_info):
            tk.Label(frame, text=label_text).grid(row=row, column=0, sticky=tk.W, pady=2)
            sensor_widgets[key] = tk.Label(frame, text="---")
            sensor_widgets[key].grid(row=row, column=1, sticky=tk.W)
        
        self.sensor_widgets[sensor_index] = sensor_widgets
    
    def update_sensor_display(self, sensor_index):
        if sensor_index in self.sensor_widgets and sensor_index in self.sensor_data:
            widgets = self.sensor_widgets[sensor_index]
            data = self.sensor_data[sensor_index]
            
            units = self.get_sensor_units(data['type'])
            processed_value = self.calculate_processed_value(data)
            
            widgets['type'].config(text=self.get_sensor_type_name(data['type']))
            widgets['value'].config(text=f"{data['value']} (raw)")
            widgets['gain'].config(text=f"{data['gain']}")
            widgets['offset'].config(text=f"{data['offset']}")
            widgets['processed'].config(text=f"{processed_value:.2f} {units}")
            widgets['status'].config(
                text="VALID" if data['is_valid'] else "INVALID",
                fg="green" if data['is_valid'] else "red"
            )
    
    def update_data(self):
        if hasattr(self, 'sensor_widgets'):
            for sensor_index in self.sensor_widgets:
                if sensor_index in self.sensor_data:
                    self.update_sensor_display(sensor_index)
        
        self.root.after(500, self.update_data)
    
    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()