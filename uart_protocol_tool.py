import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
from threading import Thread
import time
import struct

class SimpleUARTApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Protocol Tool")
        self.root.geometry("800x800")
        
        # Serial communication
        self.serial_port = None
        self.connected = False
        self.running = True
        
        # Alive monitoring - НАСТРОЙКИ ТАЙМАУТА
        self.last_alive_time = 0
        self.alive_timeout = 10000  # 10 секунд таймаут
        self.alive_check_interval = 1000  # 1 секунда - интервал проверки
        
        # Protocol constants from uart_protocol.h
        self.UART_COMMANDS = {
            # --- Команды чтения (GET) ---
            "UART_CMD_GET_EEPROM_VERSION": 0x0001,      # Запрос версии ПО EEPROM
            "UART_CMD_GET_VERSION": 0x0002,             # Запрос версии ПО FLASH
            "UART_CMD_GET_SENSOR_COUNT": 0x0003,        # Запрос количества датчиков
            "UART_CMD_GET_SENSORS_INFO": 0x1000,        # Запрос информацию о всех датчиков
            "UART_CMD_GET_SENSOR_VALUE_BASE": 0x2000,   # Базовый адрес команд Датчик 0
            "UART_CMD_GET_SENSOR_VALUE_1": 0x2001,      # Датчик 1
            "UART_CMD_GET_SENSOR_VALUE_2": 0x2002,      # Датчик 2
            "UART_CMD_GET_SENSOR_VALUE_3": 0x2003,      # Датчик 3
            "UART_CMD_GET_SENSOR_VALUE_4": 0x2004,      # Датчик 4
            "UART_CMD_GET_SENSORS_VALUE": 0x3000,       # Запрос значений всех датчиков
            "UART_CMD_GET_FAULTS_INFO": 0x4000,         # Запрос текущей информации об уставках
            "UART_CMD_GET_SERIAL": 0xF500,              # Запрос серийного номера

            # --- Команды записи (SET) ---
            "UART_CMD_SET_FAULT_VALUE": 0x5000,         # Базовый адрес установки аварийного уровня
            "UART_CMD_SET_SERIAL": 0xF505,              # Записать серийный номер

            # --- Старые команды (сохранены для обратной совместимости) ---
            "UART_CMD_NACK": 0xE000,                    # Команда отрицательного подтверждения
            "UART_CMD_ALIVE": 0xE001,                   # Команда проверки активности системы
            "UART_CMD_GET_RAW_ADC_CAL": 0xE008,         # Получить Raw-калибровку АЦП
            "UART_CMD_GET_MODE": 0xE003,                # Получить текущий режим работы
            "UART_CMD_SET_MODE": 0xE063,                # Установить режим работы

            # --- Максимальное значение ---
            "UART_CMD_MAX": 0xFFFF                      # Максимальное значение команды
        }

        self.command_descriptions = {
            # --- Команды чтения (GET) ---
            "UART_CMD_GET_EEPROM_VERSION": "Запрос версии ПО EEPROM",
            "UART_CMD_GET_VERSION": "Запрос версии ПО FLASH", 
            "UART_CMD_GET_SENSOR_COUNT": "Запрос количества датчиков",
            "UART_CMD_GET_SENSORS_INFO": "Запрос информацию о всех датчиков",
            "UART_CMD_GET_SENSOR_VALUE_BASE": "Базовый адрес команд Датчик 0",
            "UART_CMD_GET_SENSOR_VALUE_1": "Датчик 1",
            "UART_CMD_GET_SENSOR_VALUE_2": "Датчик 2",
            "UART_CMD_GET_SENSOR_VALUE_3": "Датчик 3",
            "UART_CMD_GET_SENSOR_VALUE_4": "Датчик 4",
            "UART_CMD_GET_SENSORS_VALUE": "Запрос значений всех датчиков",
            "UART_CMD_GET_FAULTS_INFO": "Запрос текущей информации об уставках",
            "UART_CMD_GET_SERIAL": "Запрос серийного номера",

            # --- Команды записи (SET) ---
            "UART_CMD_SET_FAULT_VALUE": "Базовый адрес установки аварийного уровня",
            "UART_CMD_SET_SERIAL": "Записать серийный номер",

            # --- Старые команды (сохранены для обратной совместимости) ---
            "UART_CMD_NACK": "Команда отрицательного подтверждения",
            "UART_CMD_ALIVE": "Команда проверки активности системы",
            "UART_CMD_GET_RAW_ADC_CAL": "Получить Raw-калибровку АЦП",
            "UART_CMD_GET_MODE": "Получить текущий режим работы", 
            "UART_CMD_SET_MODE": "Установить режим работы",

            # --- Максимальное значение ---
            "UART_CMD_MAX": "Максимальное значение команды"
        }
        
        # Protocol settings from uart_transport_parser.c
        self.PROTOCOL_SYNC_BYTES = [0x41, 0x41]  # 'A', 'A'
        self.MAX_PAYLOAD_SIZE = 64
        
        # Parser state
        self.rx_buffer = bytearray()
        self.parser_state = "WAIT_SYNC"
        self.expected_length = 0
        
        # Status variables
        self.connection_ok = False
        self.data_received = False
        
        # Thread management
        self.read_thread = None
        
        self.create_widgets()
        self.update_ports_list()
        
        # Start alive monitoring
        self.root.after(self.alive_check_interval, self.check_alive_status)

    def crc8_calculate(self, data):
        """Вычисление CRC8 согласно алгоритму из устройства"""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) & 0xFF) ^ 0x07
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def build_packet(self, payload):
        """Построение пакета согласно uart_transport_parser.c"""
        if len(payload) > self.MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(payload)} bytes")
        
        packet = bytearray()
        packet.extend([0x41, 0x41])  # Sync bytes
        packet.append(len(payload))  # Length
        
        # Add payload
        packet.extend(payload)
        
        # Calculate CRC (only length + payload, without sync bytes)
        crc_data = bytearray()
        crc_data.append(len(payload))  # Length
        crc_data.extend(payload)       # Payload
        
        crc = self.crc8_calculate(crc_data)
        packet.append(crc)  # Add CRC
        
        return bytes(packet)

    def parse_packet(self, packet_data):
        """Разбор входящего пакета"""
        if len(packet_data) < 4:
            return None, "Packet too short"
        
        # Check sync bytes
        if packet_data[0] != 0x41 or packet_data[1] != 0x41:
            return None, "Invalid sync bytes"
        
        # Get payload length
        payload_length = packet_data[2]
        
        # Check total packet length
        expected_packet_length = payload_length + 4
        if len(packet_data) != expected_packet_length:
            return None, f"Length mismatch: expected {expected_packet_length}, got {len(packet_data)}"
        
        # Verify CRC
        received_crc = packet_data[-1]
        crc_data = bytearray()
        crc_data.append(payload_length)
        crc_data.extend(packet_data[3:3 + payload_length])
        
        calculated_crc = self.crc8_calculate(crc_data)
        
        if received_crc != calculated_crc:
            return None, f"CRC error: received 0x{received_crc:02X}, calculated 0x{calculated_crc:02X}"
        
        # Extract payload
        payload = packet_data[3:3 + payload_length]
        return payload, None

    def process_received_byte(self, byte):
        """Обработка входящего байта (автомат состояний)"""
        if self.parser_state == "WAIT_SYNC":
            if byte == 0x41:
                self.rx_buffer = bytearray([byte])
                self.parser_state = "WAIT_SYNC2"
            return None
            
        elif self.parser_state == "WAIT_SYNC2":
            if byte == 0x41:
                self.rx_buffer.append(byte)
                self.parser_state = "WAIT_LENGTH"
            else:
                self.parser_state = "WAIT_SYNC"
            return None
            
        elif self.parser_state == "WAIT_LENGTH":
            self.rx_buffer.append(byte)
            payload_length = byte
            self.expected_length = payload_length + 4
            
            if payload_length > self.MAX_PAYLOAD_SIZE:
                self.parser_state = "WAIT_SYNC"
                self.rx_buffer = bytearray()
                return None
            elif payload_length > 0:
                self.parser_state = "WAIT_DATA"
            else:
                self.parser_state = "WAIT_CRC"
            return None
            
        elif self.parser_state == "WAIT_DATA":
            self.rx_buffer.append(byte)
            if len(self.rx_buffer) >= self.expected_length - 1:
                self.parser_state = "WAIT_CRC"
            return None
            
        elif self.parser_state == "WAIT_CRC":
            self.rx_buffer.append(byte)
            packet_data = bytes(self.rx_buffer)
            self.parser_state = "WAIT_SYNC"
            self.rx_buffer = bytearray()
            
            payload, error = self.parse_packet(packet_data)
            if error:
                self.log_message(f"❌ Packet error: {error}")
                self.log_message(f"   Raw: {packet_data.hex(' ')}")
                return None
            else:
                return payload
                
        return None

    def create_widgets(self):
        """Создание интерфейса"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Подключение", padding="5")
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="COM порт:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.port_combobox = ttk.Combobox(conn_frame, state="readonly", width=15)
        self.port_combobox.grid(row=0, column=1, padx=5)
        
        ttk.Label(conn_frame, text="Скорость:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.baud_combobox = ttk.Combobox(conn_frame, values=["9600", "19200", "38400", "57600", "115200"], 
                                         state="readonly", width=10)
        self.baud_combobox.set("9600")
        self.baud_combobox.grid(row=0, column=3, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Подключиться", command=self.connect_to_port)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        self.disconnect_btn = ttk.Button(conn_frame, text="Отключиться", 
                                       command=self.disconnect_port, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=5, padx=5)
        
        self.refresh_btn = ttk.Button(conn_frame, text="Обновить", command=self.update_ports_list)
        self.refresh_btn.grid(row=0, column=6, padx=5)
        
        # Status indicator frame
        status_frame = ttk.LabelFrame(main_frame, text="Статус связи", padding="5")
        status_frame.pack(fill=tk.X, pady=5)
        
        # Status indicator
        self.status_canvas = tk.Canvas(status_frame, width=30, height=30, bg="white", relief="sunken", bd=2)
        self.status_canvas.pack(side=tk.LEFT, padx=5)
        self.status_indicator = self.status_canvas.create_oval(5, 5, 25, 25, fill="gray", outline="black")
        
        self.status_label = ttk.Label(status_frame, text="Нет подключения", font=("Arial", 10))
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # Command frame
        cmd_frame = ttk.LabelFrame(main_frame, text="Команды UART", padding="5")
        cmd_frame.pack(fill=tk.X, pady=5)
        
        # Используем grid для точного контроля размеров
        cmd_frame.columnconfigure(1, weight=1)  # Комбобокс расширяется
        
        ttk.Label(cmd_frame, text="Команда:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        
        # ШИРОКИЙ комбобокс
        command_list = [cmd for cmd in self.UART_COMMANDS.keys() 
                       if cmd not in ["UART_CMD_NACK", "UART_CMD_MAX"]]
        self.cmd_combobox = ttk.Combobox(cmd_frame, values=command_list, state="readonly")
        self.cmd_combobox.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        self.cmd_combobox.bind('<<ComboboxSelected>>', self.on_command_selected)
        
        ttk.Label(cmd_frame, text="Параметр:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        self.param_entry = ttk.Entry(cmd_frame, width=20)
        self.param_entry.grid(row=0, column=3, padx=5, pady=2, sticky="ew")
        
        self.send_btn = ttk.Button(cmd_frame, text="Отправить команду", 
                                 command=self.send_command, state=tk.DISABLED)
        self.send_btn.grid(row=0, column=4, padx=5, pady=2)
        
        # Command description
        self.cmd_desc_label = ttk.Label(cmd_frame, text="Выберите команду для описания", 
                                       foreground="blue", wraplength=600)
        self.cmd_desc_label.grid(row=1, column=0, columnspan=5, sticky=tk.W, padx=5, pady=2)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Лог коммуникации", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create text widget with scrollbar
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, width=80, height=20)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Context menu for log
        self.log_context_menu = tk.Menu(self.root, tearoff=0)
        self.log_context_menu.add_command(label="Копировать", command=self.copy_log_selection)
        self.log_context_menu.add_command(label="Копировать все", command=self.copy_log_all)
        self.log_context_menu.add_separator()
        self.log_context_menu.add_command(label="Очистить лог", command=self.clear_log)
        
        # Bind right-click to context menu
        self.log_text.bind("<Button-3>", self.show_log_context_menu)
        
        # Log controls
        log_controls = ttk.Frame(log_frame)
        log_controls.pack(fill=tk.X, pady=5)
        
        self.clear_btn = ttk.Button(log_controls, text="Очистить лог", command=self.clear_log)
        self.clear_btn.pack(side=tk.LEFT, padx=5)
        
        self.copy_btn = ttk.Button(log_controls, text="Копировать все", command=self.copy_log_all)
        self.copy_btn.pack(side=tk.LEFT, padx=5)
        
        # Filter checkbox
        self.filter_alive_var = tk.BooleanVar(value=True)
        self.filter_checkbox = ttk.Checkbutton(log_controls, text="Скрывать Alive сообщения", 
                                             variable=self.filter_alive_var)
        self.filter_checkbox.pack(side=tk.RIGHT, padx=5)

    def show_log_context_menu(self, event):
        """Показать контекстное меню для лога"""
        try:
            self.log_context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.log_context_menu.grab_release()

    def copy_log_selection(self):
        """Копировать выделенный текст из лога"""
        try:
            if self.log_text.tag_ranges(tk.SEL):
                selected_text = self.log_text.get(tk.SEL_FIRST, tk.SEL_LAST)
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
                self.log_message("✅ Выделенный текст скопирован")
            else:
                self.log_message("⚠️ Не выделен текст для копирования")
        except Exception as e:
            self.log_message(f"❌ Ошибка копирования: {str(e)}")

    def copy_log_all(self):
        """Копировать весь лог в буфер обмена"""
        try:
            text = self.log_text.get(1.0, tk.END)
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.log_message("✅ Весь лог скопирован в буфер обмена")
        except Exception as e:
            self.log_message(f"❌ Ошибка копирования: {str(e)}")

    def update_status_indicator(self):
        """Обновление индикатора статуса"""
        if not self.connected:
            color = "gray"
            text = "Не подключено"
        elif not self.connection_ok:
            color = "red"
            text = "Потеря связи"
        elif self.data_received:
            color = "blue"
            text = "Данные получены"
            # Сбрасываем флаг получения данных до следующего пакета
            self.root.after(300, self.reset_data_received_flag)
        else:
            color = "green"
            text = "Связь OK"
        
        self.status_canvas.itemconfig(self.status_indicator, fill=color)
        self.status_label.config(text=text)

    def reset_data_received_flag(self):
        """Сброс флага получения данных (возврат к зеленому)"""
        self.data_received = False
        if self.connection_ok:
            self.update_status_indicator()

    def check_alive_status(self):
        """Проверка статуса связи"""
        if self.connected and self.serial_port and self.serial_port.is_open:
            current_time = time.time() * 1000
            time_diff = current_time - self.last_alive_time
            
            if time_diff > self.alive_timeout:
                self.connection_ok = False
                if hasattr(self, 'status_label'):
                    self.log_message("⚠️ Потеря связи с устройством", force_log=True)
            else:
                self.connection_ok = True
        else:
            self.connection_ok = False
        
        self.update_status_indicator()
        self.root.after(self.alive_check_interval, self.check_alive_status)

    def on_command_selected(self, event):
        """Обновление описания команды при выборе"""
        selected_cmd = self.cmd_combobox.get()
        if selected_cmd in self.command_descriptions:
            self.cmd_desc_label.config(text=self.command_descriptions[selected_cmd])

    def build_command_payload(self, command_name, parameter=None):
        """Построение payload для команды"""
        if command_name not in self.UART_COMMANDS:
            raise ValueError(f"Unknown command: {command_name}")
        
        command_code = self.UART_COMMANDS[command_name]
        
        # Big-endian command code (MSB first, LSB second)
        payload = bytearray([
            (command_code >> 8) & 0xFF,    # MSB
            command_code & 0xFF            # LSB
        ])
        
        # Add parameter if needed
        if parameter is not None and parameter != "":
            try:
                # Try to parse as hex or decimal
                if isinstance(parameter, str) and parameter.startswith("0x"):
                    param_value = int(parameter, 16)
                else:
                    param_value = int(parameter)
                
                # Add parameter as big-endian (MSB first, LSB second)
                payload.extend([
                    (param_value >> 8) & 0xFF,    # MSB
                    param_value & 0xFF            # LSB
                ])
                
            except ValueError as e:
                raise ValueError(f"Invalid parameter: {parameter}") from e
        
        return bytes(payload)

    def send_command(self):
        """Отправка выбранной команды"""
        if not self.connected or not self.serial_port:
            self.log_message("❌ Нет подключения к устройству")
            return
        
        try:
            command_name = self.cmd_combobox.get()
            parameter = self.param_entry.get().strip()
            
            if not command_name:
                self.log_message("❌ Не выбрана команда")
                return
            
            # Build payload
            payload = self.build_command_payload(command_name, parameter if parameter else None)
            
            # Build and send packet
            packet = self.build_packet(payload)
            
            # Логируем отправку ПЕРЕД отправкой
            self.log_message("📤 ОТПРАВЛЕНО:")
            self.log_message(f"   Длина пакета: {len(packet)} байт")
            self.log_message(f"   Команда: {command_name} (0x{self.UART_COMMANDS[command_name]:04X})")
            if parameter:
                self.log_message(f"   Параметр: {parameter}")
            self.log_message(f"   Payload: {payload.hex(' ').upper()}")
            self.log_message(f"   Полный пакет: {packet.hex(' ').upper()}")
            self.log_message(f"   CRC: 0x{packet[-1]:02X}")
            self.log_message("")
            
            # ОБНОВЛЯЕМ ВРЕМЯ АКТИВНОСТИ ПРИ ОТПРАВКЕ
            self.last_alive_time = time.time() * 1000
            
            # Отправляем пакет
            self.serial_port.write(packet)
            self.serial_port.flush()
            
        except Exception as e:
            self.log_message(f"❌ Ошибка отправки: {str(e)}")

    def read_serial_data(self):
        """Чтение данных из порта с улучшенной обработкой ошибок"""
        while self.running and self.connected:
            try:
                if not self.serial_port or not hasattr(self.serial_port, 'is_open') or not self.serial_port.is_open:
                    time.sleep(1)
                    continue
                    
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    
                    # Process each byte through state machine
                    for byte in data:
                        payload = self.process_received_byte(byte)
                        if payload is not None:
                            self.process_received_packet(payload)
                            
            except (serial.SerialException, OSError) as e:
                if self.running and self.connected:
                    error_msg = str(e)
                    self.log_message(f"❌ Ошибка порта: {error_msg}")
                    
                    # Проверяем, является ли ошибка критической
                    if any(msg in error_msg for msg in ["FileNotFoundError", "AccessDenied", "disconnected", "device not found"]):
                        self.log_message("🔌 Устройство отключено")
                        self.safe_disconnect()
                    else:
                        # Для других ошибок ждем и продолжаем
                        time.sleep(2)
                continue
            except Exception as e:
                if self.running and self.connected:
                    self.log_message(f"⚠️ Ошибка в потоке чтения: {str(e)}")
                time.sleep(2)
                continue
                
            time.sleep(0.01)

    def safe_disconnect(self):
        """Безопасное отключение с полной очисткой состояния"""
        self.log_message("🔌 Безопасное отключение...")
        self.connected = False
        self.connection_ok = False
        self.running = False
        
        # Даем время потоку чтения завершиться
        if self.read_thread and self.read_thread.is_alive():
            time.sleep(0.1)
        
        try:
            if self.serial_port and hasattr(self.serial_port, 'is_open'):
                self.serial_port.close()
                self.log_message("📴 Порт закрыт")
        except Exception as e:
            self.log_message(f"Ошибка при закрытии порта: {str(e)}")
        
        # Сбрасываем состояние парсера
        self.parser_state = "WAIT_SYNC"
        self.rx_buffer = bytearray()
        
        # Восстанавливаем UI в главном потоке
        self.root.after(0, self.reset_connection_ui)

    def reset_connection_ui(self):
        """Сброс UI для повторного подключения"""
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.send_btn.config(state=tk.DISABLED)
        self.port_combobox.config(state='readonly')
        self.baud_combobox.config(state='readonly')
        
        self.update_status_indicator()

    def process_received_packet(self, payload):
        """Обработка принятого пакета"""
        # Обновляем время последней активности
        self.last_alive_time = time.time() * 1000
        
        # Устанавливаем флаг получения данных
        self.data_received = True
        self.update_status_indicator()
        
        # Проверяем, является ли пакет Alive сообщением
        if len(payload) >= 2:
            # Big-endian command code (MSB first, LSB second)
            cmd_code = (payload[0] << 8) | payload[1]
            if cmd_code == self.UART_COMMANDS["UART_CMD_ALIVE"]:
                # Не логируем Alive сообщения если включен фильтр
                if not self.filter_alive_var.get():
                    self.log_message("💓 Получен Alive-ответ")
                return
        
        # Логируем все остальные пакеты
        self.log_message("📨 ПРИНЯТО:")
        self.log_message(f"   Длина payload: {len(payload)} байт")
        self.log_message(f"   Payload: {payload.hex(' ').upper()}")
        
        # Parse command code (big-endian)
        if len(payload) >= 2:
            cmd_code = (payload[0] << 8) | payload[1]
            
            # Find command name
            cmd_name = "UNKNOWN"
            for name, code in self.UART_COMMANDS.items():
                if code == cmd_code:
                    cmd_name = name
                    break
            
            self.log_message(f"   Команда: {cmd_name} (0x{cmd_code:04X})")
            
            # Log parameters if present
            if len(payload) > 2:
                self.log_message(f"   Параметры: {payload[2:].hex(' ').upper()}")
        
        self.log_message("")

    def log_message(self, message, force_log=False):
        """Добавление сообщения в лог"""
        # Проверяем, не является ли сообщение Alive и включен ли фильтр
        if "Alive" in message and self.filter_alive_var.get() and not force_log:
            return
            
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.update()

    def clear_log(self):
        """Очистка лога"""
        self.log_text.delete(1.0, tk.END)

    def update_ports_list(self):
        """Обновление списка COM портов"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.set(ports[0])
        else:
            self.port_combobox.set('')

    def connect_to_port(self):
        """Подключение к выбранному порту с улучшенной обработкой ошибок"""
        port_name = self.port_combobox.get()
        baud_rate = self.baud_combobox.get()
        
        if not port_name:
            self.log_message("❌ Не выбран COM-порт")
            return
        
        # Останавливаем предыдущее соединение если есть
        if self.connected:
            self.safe_disconnect()
            time.sleep(0.5)  # Даем время на закрытие
            
        try:
            # Закрываем предыдущее соединение если есть
            if self.serial_port and hasattr(self.serial_port, 'is_open'):
                try:
                    self.serial_port.close()
                except:
                    pass
            
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=int(baud_rate),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # Принудительно устанавливаем начальное состояние RTS
            self.serial_port.rts = False
            time.sleep(0.1)  # Даем время на установку состояния
                 
            # Тестируем соединение
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connected = True
            self.running = True  # ВКЛЮЧАЕМ флаг running
            self.last_alive_time = time.time() * 1000
            self.connection_ok = True
            
            self.log_message(f"✅ Подключено к {port_name} ({baud_rate} бод)")
            
            # Сбрасываем состояние парсера
            self.parser_state = "WAIT_SYNC"
            self.rx_buffer = bytearray()
            
            # Запускаем новый поток чтения
            self.read_thread = Thread(target=self.read_serial_data, daemon=True)
            self.read_thread.start()
            
            # Обновляем UI
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.send_btn.config(state=tk.NORMAL)
            self.port_combobox.config(state='disabled')
            self.baud_combobox.config(state='disabled')
            
            self.update_status_indicator()
            
        except Exception as e:
            self.log_message(f"❌ Ошибка подключения к {port_name}: {str(e)}")
            # Сбрасываем состояние при ошибке подключения
            self.connected = False
            self.connection_ok = False
            self.update_status_indicator()

    def disconnect_port(self):
        """Отключение от порта с полной очисткой состояния"""
        self.safe_disconnect()

    def on_closing(self):
        """Обработчик закрытия приложения"""
        self.running = False
        self.connected = False
        
        # Даем время потоку завершиться
        if self.read_thread and self.read_thread.is_alive():
            time.sleep(0.2)
            
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleUARTApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()