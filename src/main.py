import time
import uasyncio as asyncio
from machine import Pin, ADC
from sav import SurfaceVehicle
from imu import IMU
from pid_controller import PIDController
import struct
import neopixel
import network
import socket
import json


# Инициализация SurfaceVehicle с пинами двигателей
motor_pins = [27, 12, 14, 13]
surface_vehicle = SurfaceVehicle(motor_pins)

# Инициализация IMU
imu = IMU()

# Инициализация PID-регулятора (начальные коэффициенты устанавливаются в ноль)
surface_vehicle.pid_controller = PIDController(0.0, 0.0, 0.0)
PID_SETTINGS_FILE = 'pid_settings.json'

# Имя файла для хранения настроек Wi-Fi
WIFI_SETTINGS_FILE = 'wifi_settings.json'

# Инициализация АЦП
adc = ADC(Pin(34), atten=ADC.ATTN_11DB)  # ADC1

# Инициализация управления GPIO пином
gpio_pin = Pin(25, Pin.OUT)

# Инициализация LED-ленты WS2812 (NeoPixel)
LED_PIN = 2        # Пин, подключенный к LED-ленте
NUM_LEDS = 64      # Количество светодиодов в ленте
led_strip = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# Определение режимов анимации LED-ленты
LED_MODE_OFF = 0
LED_MODE_STATIC = 1
LED_MODE_BLINK = 2
LED_MODE_RAINBOW = 3
current_led_mode = LED_MODE_RAINBOW
current_led_color = (255, 255, 255)  # По умолчанию белый цвет

# Определение заголовков команд
CMD_MOVE = 0x01       # Команда для установки скоростей движения
CMD_PID = 0x02        # Команда для установки параметров PID-регулятора
CMD_TELEMETRY = 0x03  # Заголовок для телеметрии (используется при отправке данных)
CMD_LED = 0x04        # Команда для управления LED-лентой
CMD_GPIO = 0x05       # Команда для управления GPIO пином
CMD_MODE = 0x06       # Команда для переключения режима работы
CMD_PROBE = 0x07      # Команда для управления пробой
CMD_PROBE_CONTROL = 0x07  # Команда для управления пробоотборником

# Форматы пакетов
# Команда движения: заголовок (1 байт) + forward (float) + lateral (float) + yaw (float)
MOVE_CMD_STRUCT = 'Bfff'  # B: unsigned char, f: float

# Команда PID: заголовок (1 байт) + P (float) + I (float) + D (float)
PID_CMD_STRUCT = 'Bfff'

# Данные телеметрии: заголовок (1 байт) + roll (float) + pitch (float) + yaw (float) + adc_value (float) + pwm двигателей (float * 4)
TELEMETRY_STRUCT = 'Bfffffffffff'  # B: unsigned char, f: float

# Команда LED: заголовок (1 байт) + режим (1 байт) + R (1 байт) + G (1 байт) + B (1 байт)
LED_CMD_STRUCT = 'BBBBB'  # B: unsigned char

# Команда GPIO: заголовок (1 байт) + состояние (1 байт)
GPIO_CMD_STRUCT = 'BB'    # B: unsigned char

# Команда режима: заголовок (1 байт) + режим (1 байт)
MODE_CMD_STRUCT = 'BB'    # B: unsigned char

# Команда пробы: заголовок (1 байт) + режим (1 байт)
PROBE_CMD_STRUCT = 'BB'   # B: unsigned char

# Команда управления пробоотборником: заголовок (1 байт) + действие (1 байт) + таймаут (2 байта, unsigned short)
PROBE_CONTROL_STRUCT = 'BBH'  # B: unsigned char, H: unsigned short (для таймаута в секундах)

# Порт для приема UDP пакетов
UDP_PORT = 5005

# Сохраняем адрес и порт клиента для отправки телеметрии
client_address = None

# Функция для подключения к Wi-Fi сети
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Загрузка настроек Wi-Fi из файла
    try:
        with open(WIFI_SETTINGS_FILE, 'r') as f:
            wifi_data = json.load(f)
            WIFI_SSID = wifi_data.get('ssid')
            WIFI_PASSWORD = wifi_data.get('password')
            print(f"Настройки Wi-Fi загружены: SSID='{WIFI_SSID}'")
    except Exception as e:
        print("Ошибка при загрузке настроек Wi-Fi:")
        print(e)
        WIFI_SSID = None
        WIFI_PASSWORD = None

    if WIFI_SSID and WIFI_PASSWORD:
        if not wlan.isconnected():
            print('Подключение к сети...')
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            timeout = 10  # Таймаут в секундах
            start = time.time()
            while not wlan.isconnected():
                if time.time() - start > timeout:
                    print("Не удалось подключиться к Wi-Fi. Проверьте настройки.")
                    return
                pass
        print('Подключено к Wi-Fi. IP адрес:', wlan.ifconfig()[0])
    else:
        print("Настройки Wi-Fi не заданы. Установка режима точки доступа (AP)...")
        # Настройка точки доступа
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid='ESP32_AP')
        print('Точка доступа запущена. SSID: ESP32_AP')


# Функция для загрузки настроек PID-регулятора из файла
def load_pid_settings():
    global pid_controller
    try:
        with open(PID_SETTINGS_FILE, 'r') as f:
            pid_data = json.load(f)
            p_gain = pid_data.get('p_gain', 0.0)
            i_gain = pid_data.get('i_gain', 0.0)
            d_gain = pid_data.get('d_gain', 0.0)
            pid_controller.Kp = p_gain
            pid_controller.Ki = i_gain
            pid_controller.Kd = d_gain
            print(f"Настройки PID загружены: P={p_gain}, I={i_gain}, D={d_gain}")
    except Exception as e:
        print("Не удалось загрузить настройки PID, используются значения по умолчанию.")
        print("Ошибка:", e)


def save_pid_settings(p_gain, i_gain, d_gain):
    pid_data = {
        'p_gain': p_gain,
        'i_gain': i_gain,
        'd_gain': d_gain
    }
    try:
        with open(PID_SETTINGS_FILE, 'w') as f:
            json.dump(pid_data, f)
        print(f"Настройки PID сохранены: P={p_gain}, I={i_gain}, D={d_gain}")
    except Exception as e:
        print("Ошибка при сохранении настроек PID:")
        print(e)


# Асинхронная задача для чтения данных с IMU
async def imu_task():
    while True:
        imu.read_data()
        await asyncio.sleep_ms(10)  # Пауза 10 мс

# Асинхронная задача для приема команд по UDP
async def udp_receive_task():
    global client_address
    # Создаем UDP сокет
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('', UDP_PORT))
    udp_socket.setblocking(False)
    print(f"UDP сервер запущен на порту {UDP_PORT}")

    buffer = bytearray()
    while True:
        try:
            data, addr = udp_socket.recvfrom(1024)  # Получаем данные и адрес отправителя
            if data:
                client_address = addr  # Сохраняем адрес клиента для отправки телеметрии
                buffer.extend(data)
                while True:
                    if len(buffer) < 1:
                        break  # Необходимо хотя бы 1 байт для заголовка
                    header = buffer[0]
                    if header == CMD_MOVE and len(buffer) >= struct.calcsize(MOVE_CMD_STRUCT):
                        packet_size = struct.calcsize(MOVE_CMD_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_movement_command(packet)
                    elif header == CMD_PID and len(buffer) >= struct.calcsize(PID_CMD_STRUCT):
                        packet_size = struct.calcsize(PID_CMD_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_pid_command(packet)
                    elif header == CMD_LED and len(buffer) >= struct.calcsize(LED_CMD_STRUCT):
                        packet_size = struct.calcsize(LED_CMD_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_led_command(packet)
                    elif header == CMD_GPIO and len(buffer) >= struct.calcsize(GPIO_CMD_STRUCT):
                        packet_size = struct.calcsize(GPIO_CMD_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_gpio_command(packet)
                    elif header == CMD_MODE and len(buffer) >= struct.calcsize(MODE_CMD_STRUCT):
                        packet_size = struct.calcsize(MODE_CMD_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_mode_command(packet)
                    elif header == CMD_PROBE_CONTROL and len(buffer) >= struct.calcsize(PROBE_CONTROL_STRUCT):
                        packet_size = struct.calcsize(PROBE_CONTROL_STRUCT)
                        packet = buffer[:packet_size]
                        buffer = buffer[packet_size:]
                        await process_probe_command(packet)
                    else:
                        # Недостаточно данных или неизвестный заголовок
                        break
        except OSError:
            pass  # Нет данных для чтения
        await asyncio.sleep_ms(10)  # Пауза 10 мс

# Функции для обработки команд (остаются без изменений, как в предыдущем коде)
async def process_movement_command(packet):
    try:
        unpacked = struct.unpack(MOVE_CMD_STRUCT, packet)
        _, forward_speed, lateral_speed, yaw_speed = unpacked
        # Установка скоростей движения
        surface_vehicle.set_motors(forward_speed, lateral_speed, yaw_speed)
    except Exception as e:
        print("Ошибка при обработке команды движения:", e)

async def process_pid_command(packet):
    global pid_controller
    try:
        unpacked = struct.unpack(PID_CMD_STRUCT, packet)
        _, p_gain, i_gain, d_gain = unpacked
        # Обновление параметров PID-регулятора
        surface_vehicle.update_pid_settings(p_gain, i_gain, d_gain)
        # Сохранение настроек PID в файл
        save_pid_settings(p_gain, i_gain, d_gain)
    except Exception as e:
        print("Ошибка при обработке команды PID:", e)

async def process_led_command(packet):
    global current_led_mode, current_led_color
    try:
        unpacked = struct.unpack(LED_CMD_STRUCT, packet)
        _, mode, r, g, b = unpacked
        current_led_mode = mode
        current_led_color = (r, g, b)
        print(f"Команда LED получена: Режим={mode}, Цвет=({r}, {g}, {b})")
    except Exception as e:
        print("Ошибка при обработке команды LED:", e)

async def process_gpio_command(packet):
    try:
        unpacked = struct.unpack(GPIO_CMD_STRUCT, packet)
        _, state = unpacked
        gpio_pin.value(state)
        print(f"Команда GPIO получена: Состояние={state}")
    except Exception as e:
        print("Ошибка при обработке команды GPIO:", e)

async def process_mode_command(packet):
    try:
        unpacked = struct.unpack(MODE_CMD_STRUCT, packet)
        _, mode = unpacked
        if mode == 0:
            surface_vehicle.set_mode('manual')
        elif mode == 1:
            surface_vehicle.set_mode('stabilization')
        else:
            print("Неизвестный режим работы получен")
    except Exception as e:
        print("Ошибка при обработке команды режима работы:", e)

async def process_probe_command(packet):
    try:
        unpacked = struct.unpack(PROBE_CONTROL_STRUCT, packet)
        _, action, timeout = unpacked
        if action == 1:  # Вверх
            # Запустить пробоотборник вверх на заданное время
            surface_vehicle.probe('up')
        elif action == 2:  # Вниз
            # Запустить пробоотборник вниз на заданное время
            surface_vehicle.probe('down')
        elif action == 3:  # Стоп
            # Остановить пробоотборник
            surface_vehicle.probe('stop')
        else:
            print("Неизвестное действие для пробоотборника")
        if timeout > 0:
            await asyncio.sleep(timeout)
            print(f"Пробоотборник будет остановлен через {timeout} сек")
            surface_vehicle.probe('stop')

    except Exception as e:
        print("Ошибка при обработке команды пробоотборника:", e)

# Асинхронная задача для отправки телеметрии по UDP
async def telemetry_task():
    global client_address
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        if client_address:
            # Чтение значения АЦП
            adc_value = adc.read()

            # Подготовка данных телеметрии
            data = [CMD_TELEMETRY,
                imu.roll,
                imu.pitch,
                imu.yaw,
                float(adc_value)] + surface_vehicle.get_pwm_values() + [
                surface_vehicle.pid_controller.Kp,
                surface_vehicle.pid_controller.Ki,
                surface_vehicle.pid_controller.Kd,
            ]
            packet = struct.pack(TELEMETRY_STRUCT,*data)
            # Отправка данных телеметрии по UDP
            try:
                udp_socket.sendto(packet, (client_address[0], 5006))
            except Exception as e:
                print("Ошибка при отправке телеметрии:", e)
        await asyncio.sleep_ms(100)  # Отправка телеметрии каждые 100 мс

# Остальные задачи остаются без изменений
async def control_task():
    while True:
        # Обновление управления на основе текущего угла yaw
        current_yaw = imu.yaw
        surface_vehicle.update_control(current_yaw)
        await asyncio.sleep_ms(10)  # Цикл управления каждые 10 мс

async def led_task():
    while True:
        if current_led_mode == LED_MODE_OFF:
            led_strip.fill((0, 0, 0))
            led_strip.write()
        elif current_led_mode == LED_MODE_STATIC:
            led_strip.fill(current_led_color)
            led_strip.write()
        elif current_led_mode == LED_MODE_BLINK:
            led_strip.fill(current_led_color)
            led_strip.write()
            await asyncio.sleep_ms(500)
            led_strip.fill((0, 0, 0))
            led_strip.write()
            await asyncio.sleep_ms(500)
        elif current_led_mode == LED_MODE_RAINBOW:
            for i in range(NUM_LEDS):
                index = (i * 256 // NUM_LEDS) & 255
                led_strip[i] = wheel((index + time.ticks_ms() // 10) & 255)
            led_strip.write()
            await asyncio.sleep_ms(50)
        else:
            # Неизвестный режим, отключаем LED-ленту
            led_strip.fill((0, 0, 0))
            led_strip.write()
        await asyncio.sleep_ms(10)

def wheel(pos):
    """Генерация цветов радуги по позициям от 0 до 255."""
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    pos -= 170
    return (pos * 3, 0, 255 - pos * 3)

# Главная асинхронная функция для запуска всех задач
async def main():
    connect_wifi()
    tasks = [
        imu_task(),
        udp_receive_task(),
        telemetry_task(),
        control_task(),
        led_task(),
    ]
    await asyncio.gather(*tasks)

# Запуск событийного цикла asyncio
asyncio.run(main())
