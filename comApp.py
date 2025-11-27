import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from threading import Thread
import time

class SensorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("KOSTVT UART Monitor v2.0")
        
        # Фиксируем размер окна
        self.root.geometry("800x600")
        self.root.minsize(800, 600)
        self.root.maxsize(800, 600)
        
        # Serial communication
        self.serial_port = None
        self.sensors_count = 0
        self.sensor_data = {}
        self.sensor_widgets = {}
        self.sensor_info = {}  # Для хранения информации о датчиках
        
        # Protocol settings from uart_protocol_tool.py
        self.HEADER_SEQUENCE = [0x1F, 0x8B, 0xE2, 0x74]
        self.HEADER_LENGTH = len(self.HEADER_SEQUENCE)
        self.MAX_PAYLOAD_SIZE = 64
        
        # UART commands from uart_protocol.h
        self.UART_COMMANDS = {
            "UART_CMD_GET_SENSOR_COUNT": 0x0003,
            "UART_CMD_GET_SENSORS_INFO": 0x1000,
            "UART_CMD_GET_SENSORS_VALUE": 0x3000,
            "UART_CMD_GET_FAULTS_INFO": 0x4000,
            "UART_CMD_SET_FAULT_VALUE": 0x5000,
            "UART_CMD_NACK": 0xE000,
            "UART_CMD_ALIVE": 0xE001,
        }
        
        # Sensor types
        self.sensor_types = {
            0: "UNDEFINED",
            1: "TEMPERATURE", 
            2: "PRESSURE",
            3: "HUMIDITY",
            4: "DUST",
            5: "COUNT"
        }
        
        # Sensor locations
        self.sensor_locations = {
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
        
        # Sensor polling
        self.polling_interval = 500
        self.polling_active = False
        self.connected = False
        
        # Alive monitoring
        self.last_alive_time = 0
        self.alive_timeout = 5000
        self.alive_check_interval = 1000
        
        # Parser state
        self.rx_buffer = bytearray()
        self.parser_state = "WAIT_HEADER"
        self.expected_length = 0
        self.header_index = 0
        
        # Log buffer settings
        self.max_log_lines = 1000
        self.log_buffer_size = 100
        
        self.create_widgets()
        self.update_ports_list()
        
        self.running = True
        self.read_thread = Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        
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
        """Построение пакета с новым заголовком"""
        if len(payload) > self.MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(payload)} bytes")
        
        packet = bytearray()
        packet.extend(self.HEADER_SEQUENCE)  # Sync bytes
        packet.append(len(payload))  # Length
        
        # Add payload
        packet.extend(payload)
        
        # Calculate CRC (only length + payload, without header bytes)
        crc_data = bytearray()
        crc_data.append(len(payload))  # Length
        crc_data.extend(payload)       # Payload
        
        crc = self.crc8_calculate(crc_data)
        packet.append(crc)  # Add CRC
        
        return bytes(packet)

    def parse_packet(self, packet_data):
        """Разбор входящего пакета"""
        min_packet_length = self.HEADER_LENGTH + 2  # header + length + crc
        if len(packet_data) < min_packet_length:
            return None, "Packet too short"
        
        # Check header bytes
        for i in range(self.HEADER_LENGTH):
            if packet_data[i] != self.HEADER_SEQUENCE[i]:
                return None, f"Invalid header byte at position {i}"
        
        # Get payload length
        payload_length = packet_data[self.HEADER_LENGTH]
        
        # Check total packet length
        expected_packet_length = self.HEADER_LENGTH + payload_length + 2
        if len(packet_data) != expected_packet_length:
            return None, f"Length mismatch"
        
        # Verify CRC
        received_crc = packet_data[-1]
        crc_data = bytearray()
        crc_data.append(payload_length)  # Length byte
        payload_start = self.HEADER_LENGTH + 1
        crc_data.extend(packet_data[payload_start:payload_start + payload_length])
        
        calculated_crc = self.crc8_calculate(crc_data)
        
        if received_crc != calculated_crc:
            return None, f"CRC error"
        
        # Extract payload
        payload = packet_data[payload_start:payload_start + payload_length]
        return payload, None

    def process_received_byte(self, byte):
        """Обработка входящего байта"""
        if self.parser_state == "WAIT_HEADER":
            if byte == self.HEADER_SEQUENCE[self.header_index]:
                self.rx_buffer.append(byte)
                self.header_index += 1
                
                if self.header_index >= self.HEADER_LENGTH:
                    self.parser_state = "WAIT_LENGTH"
                    self.header_index = 0
            else:
                self.header_index = 0
                self.rx_buffer = bytearray()
                if byte == self.HEADER_SEQUENCE[0]:
                    self.rx_buffer.append(byte)
                    self.header_index = 1
            return None
            
        elif self.parser_state == "WAIT_LENGTH":
            self.rx_buffer.append(byte)
            payload_length = byte
            self.expected_length = self.HEADER_LENGTH + payload_length + 2
            
            if payload_length > self.MAX_PAYLOAD_SIZE:
                self.parser_state = "WAIT_HEADER"
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
            
            self.parser_state = "WAIT_HEADER"
            self.rx_buffer = bytearray()
            
            payload, error = self.parse_packet(packet_data)
            if error:
                self.log_message(f"❌ Ошибка пакета: {error}")
                return None
            else:
                self.log_message(f"✅ Принят пакет: {packet_data.hex(' ')}")
                return payload
                
        return None

    def send_packet(self, payload):
        """Отправка пакета"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
            
        try:
            packet = self.build_packet(payload)
            self.serial_port.write(packet)
            self.serial_port.flush()
            
            self.log_message(f"📤 Отправлен пакет: {packet.hex(' ')}")
            self.log_message(f"   Payload: {payload.hex(' ')}")
            
            return True
        except Exception as e:
            self.log_message(f"❌ Ошибка отправки: {str(e)}")
            return False

    def build_command_payload(self, command_name, parameter=None):
        """Построение payload для команды (Big-endian)"""
        if command_name not in self.UART_COMMANDS:
            raise ValueError(f"Unknown command: {command_name}")
        
        command_code = self.UART_COMMANDS[command_name]
        
        # Big-endian command code (MSB first, LSB second)
        payload = bytearray([
            (command_code >> 8) & 0xFF,    # MSB
            command_code & 0xFF            # LSB
        ])
        
        # Add parameter if needed
        if parameter is not None:
            payload.extend(parameter)
        
        return bytes(payload)

    def request_sensor_count(self):
        """Запрос количества датчиков"""
        payload = self.build_command_payload("UART_CMD_GET_SENSOR_COUNT")
        if self.send_packet(payload):
            self.log_message("📊 Запрос количества датчиков")

    def request_sensors_info(self):
        """Запрос информации о датчиках"""
        payload = self.build_command_payload("UART_CMD_GET_SENSORS_INFO")
        if self.send_packet(payload):
            self.log_message("📋 Запрос информации о датчиках")

    def request_sensors_value(self):
        """Запрос значений всех датчиков"""
        payload = self.build_command_payload("UART_CMD_GET_SENSORS_VALUE")
        if self.send_packet(payload):
            self.log_message("📡 Запрос значений датчиков")

    def request_faults_info(self):
        """Запрос информации об уставках"""
        payload = self.build_command_payload("UART_CMD_GET_FAULTS_INFO")
        if self.send_packet(payload):
            self.log_message("⚡ Запрос информации об уставках")

    def send_fault_value(self, sensor_index, fault_value, is_fault_on):
        """Отправка уставки для датчика"""
        # Формируем параметры: [флаг_включения, MSB_значения, LSB_значения]
        parameter = bytearray([
            int(is_fault_on),  # Флаг включения (0/1)
            (fault_value >> 8) & 0xFF,  # MSB значения
            fault_value & 0xFF          # LSB значения
        ])
        
        # Команда SET_FAULT_VALUE для конкретного датчика
        command_code = self.UART_COMMANDS["UART_CMD_SET_FAULT_VALUE"] + sensor_index
        command_name = f"UART_CMD_SET_FAULT_VALUE_{sensor_index}"
        
        payload = self.build_command_payload("UART_CMD_SET_FAULT_VALUE", parameter)
        if self.send_packet(payload):
            self.log_message(f"⚡ Отправка уставки для датчика {sensor_index}: значение={fault_value}, включено={is_fault_on}")

    def process_received_packet(self, payload):
        """Обработка принятого пакета"""
        self.last_alive_time = time.time() * 1000
        
        if len(payload) < 2:
            self.log_message("⚠️ Слишком короткий payload")
            return
            
        # Big-endian command code (MSB first, LSB second)
        cmd_code = (payload[0] << 8) | payload[1]
        
        self.log_message(f"🔍 Обработка команды: 0x{cmd_code:04X}")
        
        # Обработка NACK
        if cmd_code == self.UART_COMMANDS["UART_CMD_NACK"]:
            self.process_nack(payload)
            return
            
        # Обработка Alive
        if cmd_code == self.UART_COMMANDS["UART_CMD_ALIVE"]:
            self.log_message("💓 Получен Alive ответ")
            return
            
        # Обработка количества датчиков
        if cmd_code == self.UART_COMMANDS["UART_CMD_GET_SENSOR_COUNT"]:
            self.process_sensor_count(payload)
            return
            
        # Обработка информации о датчиках
        if cmd_code == self.UART_COMMANDS["UART_CMD_GET_SENSORS_INFO"]:
            self.process_sensors_info(payload)
            return
            
        # Обработка значений датчиков
        if cmd_code == self.UART_COMMANDS["UART_CMD_GET_SENSORS_VALUE"]:
            self.process_sensors_value(payload)
            return
            
        # Обработка информации об уставках
        if cmd_code == self.UART_COMMANDS["UART_CMD_GET_FAULTS_INFO"]:
            self.process_faults_info(payload)
            return
            
        # Обработка SET_FAULT_VALUE ответа
        if cmd_code >= self.UART_COMMANDS["UART_CMD_SET_FAULT_VALUE"] and \
           cmd_code < (self.UART_COMMANDS["UART_CMD_SET_FAULT_VALUE"] + self.sensors_count):
            self.process_set_fault_response(payload, cmd_code)
            return
            
        self.log_message(f"⚠️ Неизвестная команда: 0x{cmd_code:04X}")

    def process_nack(self, payload):
        """Обработка NACK"""
        if len(payload) >= 4:
            nack_code = (payload[2] << 8) | payload[3]
            nack_messages = {
                0: "Неопределенная ошибка",
                1: "Неверная команда",
                2: "Устройство занято", 
                3: "Неверный параметр",
                4: "Внутренняя ошибка"
            }
            error_msg = nack_messages.get(nack_code, f"Неизвестная ошибка (код: {nack_code})")
            self.log_message(f"❌ NACK: {error_msg}")
        else:
            self.log_message("❌ NACK: некорректный формат")

    def process_sensor_count(self, payload):
        """Обработка ответа с количеством датчиков"""
        if len(payload) >= 4:
            new_sensors_count = (payload[2] << 8) | payload[3]
            self.log_message(f"📊 Получено количество датчиков: {new_sensors_count}")
            
            if new_sensors_count != self.sensors_count:
                self.sensors_count = new_sensors_count
                self.initialize_sensor_system()
            else:
                self.log_message("ℹ️ Количество датчиков не изменилось")
        else:
            self.log_message("⚠️ Неверный формат ответа количества датчиков")

    def process_sensors_info(self, payload):
        """Обработка информации о датчиках"""
        if len(payload) < 3:
            self.log_message("⚠️ Неверный формат информации о датчиках")
            return
            
        # Пропускаем 2 байта команды
        data = payload[2:]
        sensor_count = len(data) // 6  # 6 байт на датчик: location(1) + type(1) + gain(2) + offset(2)
        
        self.log_message(f"📋 Получена информация о {sensor_count} датчиках")
        
        for i in range(sensor_count):
            start_idx = i * 6
            if start_idx + 5 < len(data):
                location = data[start_idx]
                sensor_type = data[start_idx + 1]
                gain = (data[start_idx + 2] << 8) | data[start_idx + 3]
                offset = (data[start_idx + 4] << 8) | data[start_idx + 5]
                
                self.sensor_info[i] = {
                    'location': location,
                    'type': sensor_type,
                    'gain': gain,
                    'offset': offset
                }
                
                self.log_message(f"   Датчик {i}: тип={self.sensor_types.get(sensor_type, 'UNKNOWN')}, " +
                               f"расположение={self.sensor_locations.get(location, 'UNKNOWN')}, " +
                               f"усиление={gain}, смещение={offset}")
        
        # Обновляем отображение
        self.root.after(0, self.update_sensor_displays)

    def process_sensors_value(self, payload):
        """Обработка значений датчиков"""
        if len(payload) < 3:
            self.log_message("⚠️ Неверный формат значений датчиков")
            return
            
        # Пропускаем 2 байта команды
        data = payload[2:]
        sensor_count = len(data) // 2  # 2 байта на значение
        
        self.log_message(f"📡 Получены значения {sensor_count} датчиков")
        
        for i in range(sensor_count):
            start_idx = i * 2
            if start_idx + 1 < len(data):
                value = (data[start_idx] << 8) | data[start_idx + 1]
                
                # Маркер невалидных данных
                if value == 0xFFFF:
                    self.log_message(f"   Датчик {i}: НЕВАЛИДНЫЕ ДАННЫЕ")
                    if i in self.sensor_data:
                        self.sensor_data[i]['is_valid'] = False
                else:
                    if i not in self.sensor_data:
                        self.sensor_data[i] = {}
                    
                    self.sensor_data[i]['value'] = value
                    self.sensor_data[i]['is_valid'] = True
                    
                    # Добавляем информацию из sensor_info если есть
                    if i in self.sensor_info:
                        self.sensor_data[i].update(self.sensor_info[i])
                    
                    self.log_message(f"   Датчик {i}: значение={value}")
        
        # Обновляем отображение
        self.root.after(0, self.update_sensor_displays)

    def process_faults_info(self, payload):
        """Обработка информации об уставках"""
        if len(payload) < 3:
            self.log_message("⚠️ Неверный формат информации об уставках")
            return
            
        # Пропускаем 2 байта команды
        data = payload[2:]
        sensor_count = len(data) // 4  # 4 байта на датчик: пустой(1) + флаг(1) + уровень(2)
        
        self.log_message(f"⚡ Получена информация об уставках {sensor_count} датчиков")
        
        for i in range(sensor_count):
            start_idx = i * 4
            if start_idx + 3 < len(data):
                is_fault_detection = data[start_idx + 1]  # Пропускаем пустой байт
                fault_level = (data[start_idx + 2] << 8) | data[start_idx + 3]
                
                if i not in self.sensor_data:
                    self.sensor_data[i] = {}
                
                self.sensor_data[i]['is_fault_detection'] = bool(is_fault_detection)
                self.sensor_data[i]['fault_level'] = fault_level
                
                # Добавляем информацию из sensor_info если есть
                if i in self.sensor_info:
                    self.sensor_data[i].update(self.sensor_info[i])
                
                self.log_message(f"   Датчик {i}: детекция={'ON' if is_fault_detection else 'OFF'}, уровень={fault_level}")
        
        # Обновляем отображение
        self.root.after(0, self.update_sensor_displays)
        
        # Автоматически запускаем опрос после получения всей информации
        self.root.after(500, self.start_sensor_polling)

    def update_sensor_displays(self):
        """Обновление отображения всех датчиков"""
        for sensor_index in range(self.sensors_count):
            self.update_sensor_display(sensor_index)

    def update_sensor_display(self, sensor_index):
        """Обновление отображения конкретного датчика"""
        if sensor_index in self.sensor_widgets and sensor_index in self.sensor_data:
            widgets = self.sensor_widgets[sensor_index]
            data = self.sensor_data[sensor_index]
            
            # Basic info
            sensor_type = data.get('type', 0)
            location = data.get('location', 0)
            value = data.get('value', 0)
            gain = data.get('gain', 1)
            offset = data.get('offset', 0)
            is_valid = data.get('is_valid', False)
            is_fault_detection = data.get('is_fault_detection', False)
            fault_level = data.get('fault_level', 0)
            
            # Units for display
            units = {
                1: "°C",  # TEMPERATURE
                2: "kPa", # PRESSURE  
                3: "%",   # HUMIDITY
                4: "μg/m³" # DUST
            }
            unit = units.get(sensor_type, "")
            
            # Calculate processed value
            if gain != 0:
                processed_value = (value / gain) - offset
            else:
                processed_value = 0
            
            # Update widgets
            widgets['type'].config(text=self.sensor_types.get(sensor_type, "UNKNOWN"))
            widgets['location'].config(text=self.sensor_locations.get(location, "UNKNOWN"))
            widgets['value'].config(text=f"{value} (raw)")
            widgets['gain'].config(text=f"{gain}")
            widgets['offset'].config(text=f"{offset}")
            widgets['processed'].config(text=f"{processed_value:.2f} {unit}")
            widgets['status'].config(
                text="VALID" if is_valid else "INVALID",
                fg="green" if is_valid else "red"
            )
            widgets['fault_detection'].config(
                text="ON" if is_fault_detection else "OFF",
                fg="green" if is_fault_detection else "red"
            )
            
            # Update fault level with processed value if possible
            if gain != 0:
                processed_fault = (fault_level / gain) - offset
                widgets['fault_level'].config(text=f"{processed_fault:.2f} {unit}")
            else:
                widgets['fault_level'].config(text=f"{fault_level} (raw)")
            
            # Update fault control
            if 'fault_detection_var' in widgets:
                widgets['fault_detection_var'].set(is_fault_detection)
                widgets['fault_entry'].delete(0, tk.END)
                widgets['fault_entry'].insert(0, str(fault_level))

    def start_sensor_polling(self):
        """Запуск опроса датчиков"""
        if not self.connected:
            return
            
        self.polling_active = True
        self.start_poll_btn.config(state=tk.DISABLED)
        self.stop_poll_btn.config(state=tk.NORMAL)
        self.log_message("🔁 Запуск опроса датчиков")
        self.poll_sensors()

    def poll_sensors(self):
        """Опрос датчиков"""
        if not self.polling_active or not self.connected:
            return
            
        self.request_sensors_value()
        self.root.after(self.polling_interval, self.poll_sensors)

    def process_sensors_value(self, payload):
        """Обработка значений датчиков"""
        if len(payload) < 3:
            self.log_message("⚠️ Неверный формат значений датчиков")
            return
            
        # Пропускаем 2 байта команды
        data = payload[2:]
        sensor_count = len(data) // 2  # 2 байта на значение
        
        self.log_message(f"📡 Получены значения {sensor_count} датчиков")
        
        for i in range(sensor_count):
            start_idx = i * 2
            if start_idx + 1 < len(data):
                value = (data[start_idx] << 8) | data[start_idx + 1]
                
                # Маркер невалидных данных
                if value == 0xFFFF:
                    self.log_message(f"   Датчик {i}: НЕВАЛИДНЫЕ ДАННЫЕ")
                    if i in self.sensor_data:
                        self.sensor_data[i]['is_valid'] = False
                else:
                    if i not in self.sensor_data:
                        self.sensor_data[i] = {}
                    
                    self.sensor_data[i]['value'] = value
                    self.sensor_data[i]['is_valid'] = True
                    
                    # Добавляем информацию из sensor_info если есть
                    if i in self.sensor_info:
                        self.sensor_data[i].update(self.sensor_info[i])
                    
                    self.log_message(f"   Датчик {i}: значение={value}")
        
        # Обновляем отображение
        self.root.after(0, self.update_sensor_displays)

    def process_set_fault_response(self, payload, cmd_code):
        """Обработка ответа на установку уставки"""
        sensor_index = cmd_code - self.UART_COMMANDS["UART_CMD_SET_FAULT_VALUE"]
        
        if len(payload) >= 3 and payload[2] == 1:  # Статус успеха
            self.log_message(f"✅ Уставка для датчика {sensor_index} успешно установлена")
        else:
            self.log_message(f"❌ Ошибка установки уставки для датчика {sensor_index}")

    def create_widgets(self):
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        control_frame = tk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=5)
        
        # COM port selection
        com_frame = tk.Frame(control_frame)
        com_frame.pack(fill=tk.X, pady=2)
        
        tk.Label(com_frame, text="COM порт:").pack(side=tk.LEFT)
        
        self.port_combobox = ttk.Combobox(com_frame, state="readonly", width=15)
        self.port_combobox.pack(side=tk.LEFT, padx=5)
        
        tk.Label(com_frame, text="Скорость:").pack(side=tk.LEFT, padx=(10,0))
        self.baud_combobox = ttk.Combobox(com_frame, values=["9600", "19200", "38400", "57600", "115200"], 
                                         state="readonly", width=10)
        self.baud_combobox.set("9600")
        self.baud_combobox.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(com_frame, text="Подключиться", command=self.connect_to_port)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = tk.Button(com_frame, text="Закрыть порт", 
                                      command=self.disconnect_port, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        self.refresh_btn = tk.Button(com_frame, text="Обновить список", command=self.update_ports_list)
        self.refresh_btn.pack(side=tk.LEFT)
        
        # Control buttons frame
        btn_frame = tk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        
        self.get_sensors_btn = tk.Button(btn_frame, text="Запросить датчики", 
                                       command=self.request_initial_config, state=tk.DISABLED)
        self.get_sensors_btn.pack(side=tk.LEFT, padx=5)
        
        self.start_poll_btn = tk.Button(btn_frame, text="Старт опроса", 
                                      command=self.start_sensor_polling, state=tk.DISABLED)
        self.start_poll_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_poll_btn = tk.Button(btn_frame, text="Стоп опроса", 
                                     command=self.stop_sensor_polling, state=tk.DISABLED)
        self.stop_poll_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        status_frame = tk.Frame(control_frame)
        status_frame.pack(fill=tk.X, pady=2)
        
        self.status_label = tk.Label(status_frame, text="Статус: Не подключено", anchor='w')
        self.status_label.pack(side=tk.LEFT)
        
        self.alive_status = tk.Label(status_frame, text="[ALIVE: ---]", fg="gray")
        self.alive_status.pack(side=tk.RIGHT)
        
        # Notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # System tab
        self.system_tab = tk.Frame(self.notebook)
        self.notebook.add(self.system_tab, text="Система")
        
        self.system_text = tk.Text(self.system_tab, wrap=tk.WORD)
        scrollbar = tk.Scrollbar(self.system_tab, command=self.system_text.yview)
        self.system_text.config(yscrollcommand=scrollbar.set)
        
        self.system_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.system_text.insert(tk.END, "Ожидание подключения к устройству...\n")
        self.system_text.config(state=tk.DISABLED)
        
        # Context menu
        self.context_menu = tk.Menu(self.root, tearoff=0)
        self.context_menu.add_command(label="Копировать", command=self.copy_from_log)
        self.context_menu.add_command(label="Очистить лог", command=self.clear_log)
        self.system_text.bind("<Button-3>", self.show_context_menu)

    def create_sensor_tabs(self):
        """Создание вкладок для датчиков"""
        if not hasattr(self, 'notebook'):
            return
            
        # Удаляем старые вкладки датчиков
        for tab in self.notebook.tabs()[1:]:
            self.notebook.forget(tab)
        
        self.sensor_widgets = {}
        
        for i in range(self.sensors_count):
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=f"Датчик {i}")
            
            # Main sensor frame
            main_frame = tk.Frame(tab)
            main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # Sensor data frame
            data_frame = ttk.LabelFrame(main_frame, text=f"Данные датчика {i}", padding=10)
            data_frame.pack(fill=tk.X, pady=5)
            
            # Fault control frame
            fault_frame = ttk.LabelFrame(main_frame, text="Управление уставками", padding=10)
            fault_frame.pack(fill=tk.X, pady=5)
            
            self.create_sensor_display(data_frame, fault_frame, i)

    def create_sensor_display(self, data_frame, fault_frame, sensor_index):
        """Создание отображения для датчика"""
        sensor_widgets = {}
        
        # Data display
        labels_info = [
            ("Тип:", 'type'),
            ("Расположение:", 'location'), 
            ("Значение:", 'value'),
            ("Усиление:", 'gain'),
            ("Смещение:", 'offset'),
            ("Результат:", 'processed'),
            ("Статус:", 'status'),
            ("Детекция ошибок:", 'fault_detection'),
            ("Уровень ошибки:", 'fault_level')
        ]
        
        for row, (label_text, key) in enumerate(labels_info):
            tk.Label(data_frame, text=label_text, anchor='w').grid(row=row, column=0, sticky=tk.W, pady=2)
            sensor_widgets[key] = tk.Label(data_frame, text="---", anchor='w')
            sensor_widgets[key].grid(row=row, column=1, sticky=tk.W, padx=10)
        
        # Fault control
        tk.Label(fault_frame, text="Новая уставка:").grid(row=0, column=0, sticky=tk.W, pady=2)
        sensor_widgets['fault_entry'] = tk.Entry(fault_frame, width=10)
        sensor_widgets['fault_entry'].grid(row=0, column=1, padx=5, pady=2)
        
        sensor_widgets['fault_detection_var'] = tk.BooleanVar()
        sensor_widgets['fault_checkbox'] = tk.Checkbutton(fault_frame, text="Включить детектирование",
                                                        variable=sensor_widgets['fault_detection_var'])
        sensor_widgets['fault_checkbox'].grid(row=0, column=2, padx=10, pady=2)
        
        sensor_widgets['fault_button'] = tk.Button(fault_frame, text="Установить уставку",
                                                 command=lambda idx=sensor_index: self.send_fault_setting(idx))
        sensor_widgets['fault_button'].grid(row=0, column=3, padx=5, pady=2)
        
        self.sensor_widgets[sensor_index] = sensor_widgets

    def send_fault_setting(self, sensor_index):
        """Отправка новой уставки для датчика"""
        if sensor_index not in self.sensor_widgets:
            return
            
        widgets = self.sensor_widgets[sensor_index]
        
        try:
            fault_value = int(widgets['fault_entry'].get())
            is_fault_on = widgets['fault_detection_var'].get()
            
            self.send_fault_value(sensor_index, fault_value, is_fault_on)
            
        except ValueError:
            messagebox.showerror("Ошибка", "Введите корректное числовое значение уставки")

    def update_sensor_displays(self):
        """Обновление отображения всех датчиков"""
        for sensor_index in range(self.sensors_count):
            self.update_sensor_display(sensor_index)

    def update_sensor_display(self, sensor_index):
        """Обновление отображения конкретного датчика"""
        if sensor_index in self.sensor_widgets and sensor_index in self.sensor_data:
            widgets = self.sensor_widgets[sensor_index]
            data = self.sensor_data[sensor_index]
            
            # Basic info
            sensor_type = data.get('type', 0)
            location = data.get('location', 0)
            value = data.get('value', 0)
            gain = data.get('gain', 1)
            offset = data.get('offset', 0)
            is_valid = data.get('is_valid', False)
            is_fault_detection = data.get('is_fault_detection', False)
            fault_level = data.get('fault_level', 0)
            
            # Calculate processed value
            if gain != 0 and is_valid:
                processed_value = (value / gain) - offset
            else:
                processed_value = 0
            
            # Update widgets
            widgets['type'].config(text=self.sensor_types.get(sensor_type, "UNKNOWN"))
            widgets['location'].config(text=self.sensor_locations.get(location, "UNKNOWN"))
            widgets['value'].config(text=f"{value} (raw)")
            widgets['gain'].config(text=f"{gain}")
            widgets['offset'].config(text=f"{offset}")
            widgets['processed'].config(text=f"{processed_value:.2f}")
            widgets['status'].config(
                text="VALID" if is_valid else "INVALID",
                fg="green" if is_valid else "red"
            )
            widgets['fault_detection'].config(
                text="ON" if is_fault_detection else "OFF",
                fg="green" if is_fault_detection else "red"
            )
            
            # Update fault level with processed value if possible
            if gain != 0:
                processed_fault = (fault_level / gain) - offset
                widgets['fault_level'].config(text=f"{fault_level} (raw) / {processed_fault:.2f}")
            else:
                widgets['fault_level'].config(text=f"{fault_level} (raw)")
            
            # Update fault control
            widgets['fault_detection_var'].set(is_fault_detection)

    def initialize_sensor_system(self):
        """Инициализация системы датчиков"""
        self.log_message(f"🚀 Инициализация системы с {self.sensors_count} датчиками")
        self.create_sensor_tabs()
        
        # Запрашиваем дополнительную информацию
        self.request_sensors_info()
        self.request_faults_info()

    def start_sensor_polling(self):
        """Запуск опроса датчиков"""
        self.polling_active = True
        self.log_message("🔁 Запуск опроса датчиков")
        self.poll_sensors()

    def stop_sensor_polling(self):
        """Остановка опроса датчиков"""
        self.polling_active = False
        self.log_message("⏹️ Остановка опроса датчиков")

    def poll_sensors(self):
        """Опрос датчиков"""
        if not self.polling_active or not self.connected:
            return
            
        self.request_sensors_value()
        self.root.after(self.polling_interval, self.poll_sensors)

    def request_initial_config(self):
        """Запрос начальной конфигурации"""
        self.log_message("📋 Запрос начальной конфигурации устройства...")
        self.request_sensor_count()

    # Остальные методы (connect_to_port, disconnect_port, log_message, etc.) 
    # остаются практически без изменений, нужно только обновить read_serial_data

    def read_serial_data(self):
        """Чтение данных из порта"""
        while self.running:
            try:
                if not self.serial_port or not hasattr(self.serial_port, 'is_open') or not self.serial_port.is_open:
                    time.sleep(1)
                    continue
                    
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    
                    # Логируем сырые данные
                    self.log_message(f"📨 Получено {len(data)} байт: {data.hex(' ')}")
                    
                    # Обрабатываем каждый байт
                    for byte in data:
                        payload = self.process_received_byte(byte)
                        if payload is not None:
                            self.process_received_packet(payload)
                            
            except (serial.SerialException, OSError) as e:
                if self.running:
                    self.log_message(f"Ошибка чтения порта: {str(e)}")
                time.sleep(1)
            except Exception as e:
                if self.running:
                    self.log_message(f"Ошибка в потоке чтения: {str(e)}")
                time.sleep(1)
                
            time.sleep(0.01)

    def connect_to_port(self):
        """Подключение к порту"""
        port_name = self.port_combobox.get()
        baud_rate = self.baud_combobox.get()
        
        if not port_name:
            messagebox.showerror("Ошибка", "Не выбран COM-порт")
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
            self.parser_state = "WAIT_HEADER"
            self.rx_buffer = bytearray()
            self.header_index = 0
            
            # Запускаем новый поток чтения если он не активен
            if not self.read_thread or not self.read_thread.is_alive():
                self.read_thread = Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
            
            # Обновляем UI
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.get_sensors_btn.config(state=tk.NORMAL)
            self.port_combobox.config(state='disabled')
            self.baud_combobox.config(state='disabled')
            
            self.update_status_indicator()
            
            # Автоматически запрашиваем конфигурацию при подключении
            self.root.after(1000, self.request_initial_config)
            
        except Exception as e:
            self.log_message(f"❌ Ошибка подключения к {port_name}: {str(e)}")
            # Сбрасываем состояние при ошибке подключения
            self.connected = False
            self.connection_ok = False
            self.update_status_indicator()
            messagebox.showerror("Ошибка", f"Не удалось подключиться к {port_name}:\n{str(e)}")

    def update_status_indicator(self):
        """Обновление индикатора статуса"""
        if not self.connected:
            color = "gray"
            text = "Не подключено"
        elif not self.connection_ok:
            color = "red"
            text = "Потеря связи"
        elif self.polling_active:
            color = "blue"
            text = "Опрос активен"
        else:
            color = "green"
            text = "Связь OK"
        
        self.status_label.config(text=f"Статус: {text}")

    def safe_disconnect(self):
        """Безопасное отключение с полной очисткой состояния"""
        self.log_message("🔌 Безопасное отключение...")
        self.connected = False
        self.connection_ok = False
        self.polling_active = False
        
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
        self.parser_state = "WAIT_HEADER"
        self.rx_buffer = bytearray()
        self.header_index = 0
        
        # Восстанавливаем UI в главном потоке
        self.root.after(0, self.reset_connection_ui)

    def reset_connection_ui(self):
        """Сброс UI для повторного подключения"""
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.get_sensors_btn.config(state=tk.DISABLED)
        self.start_poll_btn.config(state=tk.DISABLED)
        self.stop_poll_btn.config(state=tk.DISABLED)
        self.port_combobox.config(state='readonly')
        self.baud_combobox.config(state='readonly')
        
        self.status_label.config(text="Статус: Не подключено")
        self.alive_status.config(text="[ALIVE: ---]", fg="gray")
        
        self.sensor_data = {}
        self.sensor_info = {}
        self.sensors_count = 0
        
        # Удаляем вкладки датчиков
        for tab in self.notebook.tabs()[1:]:
            self.notebook.forget(tab)

    def disconnect_port(self):
        """Отключение от порта с полной очисткой состояния"""
        self.safe_disconnect()

    def log_message(self, message):
        """Добавление сообщения в лог"""
        self.system_text.config(state=tk.NORMAL)
        
        line_count = int(self.system_text.index('end-1c').split('.')[0])
        if line_count >= self.max_log_lines:
            delete_lines = min(self.log_buffer_size, line_count)
            self.system_text.delete(1.0, f"{delete_lines}.0")
            self.system_text.insert(tk.END, f"... удалено {delete_lines} старых строк ...\n")
        
        self.system_text.insert(tk.END, message + "\n")
        self.system_text.see(tk.END)
        self.system_text.config(state=tk.DISABLED)

    def check_alive_status(self):
        """Проверка статуса связи"""
        if self.connected and self.serial_port and self.serial_port.is_open:
            current_time = time.time() * 1000
            time_diff = current_time - self.last_alive_time
            
            if time_diff > self.alive_timeout:
                self.alive_status.config(text="[ALIVE: NO RESPONSE]", fg="red")
                self.log_message("⚠️ Устройство не отвечает (Alive timeout)")
            else:
                self.alive_status.config(text="[ALIVE: OK]", fg="green")
        
        self.root.after(self.alive_check_interval, self.check_alive_status)

    def show_context_menu(self, event):
        """Показать контекстное меню"""
        try:
            self.context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self.context_menu.grab_release()

    def copy_from_log(self):
        """Копирование из лога"""
        try:
            if self.system_text.tag_ranges(tk.SEL):
                selected_text = self.system_text.get(tk.SEL_FIRST, tk.SEL_LAST)
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
        except Exception as e:
            self.log_message(f"Ошибка копирования: {str(e)}")

    def clear_log(self):
        """Очистка лога"""
        self.system_text.config(state=tk.NORMAL)
        self.system_text.delete(1.0, tk.END)
        self.system_text.config(state=tk.DISABLED)

    def update_ports_list(self):
        """Обновление списка портов"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.set(ports[0])
        else:
            self.port_combobox.set('')
            self.status_label.config(text="Статус: COM-порты не найдены")

    def on_closing(self):
        """Обработчик закрытия приложения"""
        self.polling_active = False
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()