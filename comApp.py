import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from threading import Thread
import time

class SensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("KOSTVT UART Monitor")
        
        # Фиксируем размер окна
        self.root.geometry("700x450")  # Ширина x Высота (увеличено для Alive индикатора)
        self.root.minsize(700, 450)
        self.root.maxsize(700, 450)
        
        # Serial communication
        self.serial_port = None
        self.sensors_count = 0
        self.sensor_data = {}
        self.sensor_widgets = {}
        
        # Sensor polling
        self.current_sensor_index = 0
        self.polling_interval = 200  # 200ms between sensor polls
        self.polling_active = False
        self.connected = False
        
        # Alive monitoring
        self.last_alive_time = 0
        self.alive_timeout = 3000  # 3 seconds timeout
        self.alive_check_interval = 1000  # Check every 1 second
        
        # Log buffer settings
        self.max_log_lines = 1000  # Максимальное количество строк в логе
        self.log_buffer_size = 100  # Количество строк для удаления при переполнении
        
        self.create_widgets()
        self.update_ports_list()
        
        self.running = True
        self.read_thread = Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        
        # Start alive monitoring
        self.root.after(self.alive_check_interval, self.check_alive_status)

    def create_widgets(self):
        # Main container frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control panel - вертикальное расположение
        control_frame = tk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=5)
        
        # COM port selection - первая строка
        com_frame = tk.Frame(control_frame)
        com_frame.pack(fill=tk.X, pady=2)
        
        tk.Label(com_frame, text="COM порт:").pack(side=tk.LEFT)
        
        self.port_combobox = ttk.Combobox(com_frame, state="readonly", width=15)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(com_frame, text="Подключиться", command=self.connect_to_port)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = tk.Button(com_frame, text="Закрыть порт", 
                                      command=self.disconnect_port, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        self.refresh_btn = tk.Button(com_frame, text="Обновить список", command=self.update_ports_list)
        self.refresh_btn.pack(side=tk.LEFT)
        
        # Status label - вторая строка (под COM портом)
        status_frame = tk.Frame(control_frame)
        status_frame.pack(fill=tk.X, pady=2)
        
        self.status_label = tk.Label(status_frame, text="Статус: Не подключено", anchor='w')
        self.status_label.pack(side=tk.LEFT)
        
        # Alive status indicator
        self.alive_status = tk.Label(status_frame, text="[ALIVE: ---]", fg="gray")
        self.alive_status.pack(side=tk.RIGHT)
        
        # Notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
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

    def check_alive_status(self):
        """Check if device is still alive (responding)"""
        if self.connected and self.serial_port and self.serial_port.is_open:
            current_time = time.time() * 1000  # Convert to milliseconds
            time_diff = current_time - self.last_alive_time
            
            if time_diff > self.alive_timeout:
                # Device is not responding
                self.alive_status.config(text="[ALIVE: NO RESPONSE]", fg="red")
                self.log_message("Ошибка: Устройство не отвечает (Alive timeout)")
                self.reset_sensor_values()  # Сбрасываем значения датчиков
            else:
                # Device is responding
                self.alive_status.config(text="[ALIVE: OK]", fg="green")
        
        self.root.after(self.alive_check_interval, self.check_alive_status)

    def reset_sensor_values(self):
        """Reset all sensor values to default/undefined state"""
        for sensor_index in self.sensor_widgets:
            widgets = self.sensor_widgets[sensor_index]
            
            # Обновляем все виджеты на вкладках датчиков
            widgets['type'].config(text="---")
            widgets['location'].config(text="---")
            widgets['value'].config(text="---")
            widgets['gain'].config(text="---")
            widgets['offset'].config(text="---")
            widgets['processed'].config(text="---")
            widgets['status'].config(text="---", fg="black")
            widgets['fault_detection'].config(text="---", fg="black")
            widgets['fault_level'].config(text="---")
        
        # Также очищаем данные в памяти
        self.sensor_data = {}
        self.log_message("Значения датчиков сброшены из-за потери связи")

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
        # Добавляем небольшую задержку перед повторным подключением
        time.sleep(0.5)
        
        port_name = self.port_combobox.get()
        if not port_name:
            messagebox.showerror("Ошибка", "Не выбран COM-порт")
            return
            
        if self.serial_port and hasattr(self.serial_port, 'is_open'):
            try:
                self.serial_port.close()
            except:
                pass
                
        try:
            # Создаем новый экземпляр Serial вместо повторного использования старого
            self.serial_port = serial.Serial(port_name, baudrate=9600, timeout=1)
            self.connected = True
            self.running = True  # Убедимся, что флаг running установлен
            self.last_alive_time = time.time() * 1000  # Initialize alive timer
            self.status_label.config(text=f"Статус: Подключено к {port_name}")
            self.alive_status.config(text="[ALIVE: ---]", fg="gray")
            self.log_message(f"Успешное подключение к {port_name}")
            
            # Update UI
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.port_combobox.config(state='disabled')
            
            # Перезапускаем поток чтения, если он завершился
            if not self.read_thread.is_alive():
                self.read_thread = Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
            
            # Start communication
            self.request_sensor_count()
            
        except Exception as e:
            self.status_label.config(text="Статус: Ошибка подключения")
            self.log_message(f"Ошибка подключения к {port_name}: {str(e)}")
            messagebox.showerror("Ошибка", f"Не удалось подключиться к {port_name}:\n{str(e)}")

    def disconnect_port(self):
        """Close serial port and stop polling"""
        self.polling_active = False
        self.connected = False
        
        # Останавливаем поток чтения
        self.running = False
        
        # Даем время потоку завершиться
        time.sleep(0.1)
        
        try:
            if self.serial_port and hasattr(self.serial_port, 'is_open'):
                # Очищаем буферы перед закрытием
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.serial_port.close()
                self.log_message("COM порт закрыт")
        except Exception as e:
            self.log_message(f"Ошибка при закрытии порта: {str(e)}")
        
        # Обновляем интерфейс
        self.status_label.config(text="Статус: Не подключено")
        self.alive_status.config(text="[ALIVE: ---]", fg="gray")
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.port_combobox.config(state='readonly')

        # Очищаем данные
        self.sensor_data = {}
        self.sensors_count = 0
        self.current_sensor_index = 0
        
        # Удаляем вкладки датчиков
        for tab in self.notebook.tabs()[1:]:
            self.notebook.forget(tab)

    def log_message(self, message):
        self.system_text.config(state=tk.NORMAL)
        
        # Проверяем, не превышен ли лимит строк
        line_count = int(self.system_text.index('end-1c').split('.')[0])
        if line_count >= self.max_log_lines:
            # Удаляем старые 10% сообщений
            delete_lines = min(self.log_buffer_size, line_count)
            self.system_text.delete(1.0, f"{delete_lines}.0")
            self.system_text.insert(tk.END, f"... удалено {delete_lines} старых строк ...\n")
        
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
        # New protocol format: 0x41 0x41 <len> <cmd_lsb> <cmd_msb> <params...>
        request = bytes([0x41, 0x41, 0x02, 0x02, 0x00])  # GET_SENSORS_COUNT = 0x0002
        if self.send_request_with_delay(request):
            self.log_message("Отправлен запрос количества датчиков: 41 41 02 02 00")
    
    def request_sensor_data(self, sensor_index):
        # New protocol format: 0x41 0x41 <len> <cmd_lsb> <cmd_msb> <params...>
        # GET_SENSOR_DATA = 0x0003, параметр - индекс датчика (2 байта)
        request = bytes([
            0x41, 0x41,  # Header
            0x04,         # Length (cmd + param)
            0x03, 0x00,   # GET_SENSOR_DATA (0x0003)
            (sensor_index & 0xFF), ((sensor_index >> 8) & 0xFF)  # Sensor index (2 bytes)
        ])
        if self.send_request_with_delay(request):
            self.log_message(f"Запрос данных датчика {sensor_index} отправлен")
    
    def read_serial_data(self):
        while self.running:
            try:
                # Проверяем, что порт существует и открыт
                if not self.serial_port or not hasattr(self.serial_port, 'is_open') or not self.serial_port.is_open:
                    time.sleep(1)
                    continue
                    
                try:
                    # Безопасная проверка наличия данных
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        self.process_received_data(data)
                except (serial.SerialException, OSError) as e:
                    if self.running:  # Логируем только если не было запроса на закрытие
                        self.log_message(f"Ошибка чтения порта: {str(e)}")
                    time.sleep(1)
                    
            except Exception as e:
                if self.running:  # Логируем только если не было запроса на закрытие
                    self.log_message(f"Ошибка в потоке чтения: {str(e)}")
                time.sleep(1)
                
            time.sleep(0.1)
    
    def process_received_data(self, data):
        self.log_message(f"Получены данные: {data.hex(' ')}")
        
        pos = 0
        while pos <= len(data) - 4:  # Минимум 4 байта (заголовок + длина)
            if data[pos] == 0x41 and data[pos+1] == 0x41:
                payload_len = data[pos+2]
                message_end = pos + 3 + payload_len
                
                if message_end > len(data):
                    self.log_message(f"Неполное сообщение, ожидается {payload_len} байт")
                    break
                    
                cmd_code = data[pos+3] | (data[pos+4] << 8)
                self.last_alive_time = time.time() * 1000
                
                # Обработка ответа с количеством датчиков (0x0002)
                if cmd_code == 0x0002 and payload_len >= 1:
                    self.sensors_count = data[pos+5]
                    self.log_message(f"Получено количество датчиков: {self.sensors_count}")
                    self.initialize_sensor_system()
                    pos = message_end
                    continue
                    
                # Обработка данных датчика (0x0003)
                if cmd_code == 0x0003 and payload_len >= 13:
                    try:
                        # Проверяем, что в сообщении достаточно данных (3 заголовок + 13 payload)
                        if len(data) >= pos + 16:
                            sensor_index = data[pos+5] | (data[pos+6] << 8)
                            sensor_type = data[pos+7]
                            
                            value = data[pos+8] | (data[pos+9] << 8)
                            gain = data[pos+10] | (data[pos+11] << 8)
                            offset = data[pos+12] | (data[pos+13] << 8)
                            is_valid = data[pos+14]
                            location = data[pos+15]
                            is_fault_detection = data[pos+16]
                            fault_level = data[pos+17] | (data[pos+18] << 8)
                            
                            self.sensor_data[sensor_index] = {
                                'type': sensor_type,
                                'value': value,
                                'gain': gain,
                                'offset': offset,
                                'is_valid': is_valid,
                                'location': location,
                                'is_fault_detection': is_fault_detection,
                                'fault_level': fault_level
                            }
                            
                            self.log_message(
                                f"Данные датчика {sensor_index}:\n"
                                f"Тип: {self.get_sensor_type_name(sensor_type)}\n"
                                f"Значение: {value}\n"
                                f"Усиление: {gain}\n"
                                f"Смещение: {offset}\n"
                                f"Статус: {'VALID' if is_valid else 'INVALID'}\n"
                                f"Расположение: {location}\n"
                                f"Детекция ошибок: {'ON' if is_fault_detection else 'OFF'}\n"
                                f"Уровень ошибки: {fault_level}"
                            )
                            
                            self.root.after(0, lambda idx=sensor_index: self.update_sensor_display(idx))
                        else:
                            self.log_message("Ошибка: Недостаточно данных в сообщении датчика")
                    except IndexError as e:
                        self.log_message(f"Ошибка обработки данных датчика: {str(e)}")
                    finally:
                        pos = message_end
                        continue
                        
                # Обработка Alive-сообщения (0x0001)
                if cmd_code == 0x0001:
                    self.log_message("Получен Alive ответ от устройства")
                    pos = message_end
                    continue
                    
            pos += 1

    def initialize_sensor_system(self):
        """Initialize sensor tabs and start polling"""
        # Create tabs for each sensor
        self.create_sensor_tabs()
        
        # Start polling cycle
        self.polling_active = True
        self.poll_next_sensor()

    def poll_next_sensor(self):
        """Poll next sensor in sequence"""
        if not self.polling_active or not self.connected or not self.serial_port.is_open:
            return
            
        if self.sensors_count > 0:
            self.request_sensor_data(self.current_sensor_index)
            
            # Move to next sensor (circular)
            self.current_sensor_index = (self.current_sensor_index + 1) % self.sensors_count
            
            # Schedule next poll
            self.root.after(self.polling_interval, self.poll_next_sensor)

    def create_sensor_tabs(self):
        """Create tabs for each sensor"""
        if not hasattr(self, 'notebook'):
            return
            
        # Remove old tabs
        for tab in self.notebook.tabs()[1:]:
            self.notebook.forget(tab)
        
        self.sensor_widgets = {}
        
        # Create tabs for each sensor
        for i in range(self.sensors_count):
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=f"Датчик {i}")
            
            # Header
            ttk.Label(tab, text=f"Данные датчика {i}", 
                    font=('Arial', 12, 'bold')).pack(pady=5)
            
            self.create_sensor_display(tab, i)

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

    def get_location_name(self, location_code):
        locations = {
            0: "SENSOR_LOCATION_UNDEF",
            1: "SENSOR_LOCATION_1",
            2: "SENSOR_LOCATION_2",
            3: "SENSOR_LOCATION_3",
            4: "SENSOR_LOCATION_4",
            5: "SENSOR_LOCATION_5",
            6: "SENSOR_LOCATION_6",
            7: "SENSOR_LOCATION_7",
            8: "SENSOR_LOCATION_COUNT"
        }
        return locations.get(location_code, f"UNKNOWN_LOCATION ({location_code})")

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
            ("Расположение:", 'location'),
            ("Сырое значение:", 'value'),
            ("Усиление:", 'gain'),
            ("Смещение:", 'offset'),
            ("Результат:", 'processed'),
            ("Статус:", 'status'),
            ("Детекция ошибок:", 'fault_detection'),
            ("Уровень ошибки:", 'fault_level')
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
            
            sensor_type = data['type']
            unit = self.get_sensor_units(sensor_type)
            processed_value = self.calculate_processed_value(data)
            
            # Обработка fault_level
            if data['fault_level'] == 0xFFFF:
                fault_level_text = "---"
            else:
                processed_fault_level = self.calculate_processed_value({
                    'value': data['fault_level'],
                    'gain': data['gain'],
                    'offset': data['offset']
                })
                fault_level_text = f"{processed_fault_level:.2f} {unit}"
            
            widgets['type'].config(text=f"{self.get_sensor_type_name(sensor_type)}")
            widgets['location'].config(text=f"{self.get_location_name(data['location'])}")
            widgets['value'].config(text=f"{data['value']} (raw)")
            widgets['gain'].config(text=f"{data['gain']}")
            widgets['offset'].config(text=f"{data['offset']}")
            widgets['processed'].config(text=f"{processed_value:.2f} {unit}")
            widgets['status'].config(
                text="VALID" if data['is_valid'] else "INVALID",
                fg="green" if data['is_valid'] else "red"
            )
            widgets['fault_detection'].config(
                text="ON" if data['is_fault_detection'] else "OFF",
                fg="green" if data['is_fault_detection'] else "red"
            )
            widgets['fault_level'].config(text=fault_level_text)
    
    def update_data(self):
        if hasattr(self, 'sensor_widgets'):
            for sensor_index in self.sensor_widgets:
                if sensor_index in self.sensor_data:
                    self.update_sensor_display(sensor_index)
        
        self.root.after(500, self.update_data)
    
    def on_closing(self):
        """Cleanup on window close"""
        self.polling_active = False
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()