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

# Настройки Wi-Fi
WIFI_SSID = 'Photon-Home-2.4G'
WIFI_PASSWORD = 'CgzxRdq4LQ5TwpN9'

# Инициализация SurfaceVehicle с пинами двигателей
motor_pins = [14, 15, 16, 17]  # Пример пинов для управления двигателями
surface_vehicle = SurfaceVehicle(motor_pins)

# Инициализация IMU
imu = IMU()

# Инициализация PID-регулятора (начальные коэффициенты устанавливаются в ноль)
surface_vehicle.pid_controller = PIDController(0.0, 0.0, 0.0)

# Инициализация АЦП
adc = ADC(Pin(34))  # ADC1

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
current_led_mode = LED_MODE_OFF
current_led_color = (255, 255, 255)  # По умолчанию белый цвет

# Определение заголовков команд
CMD_MOVE = 0x01       # Команда для установки скоростей движения
CMD_PID = 0x02        # Команда для установки параметров PID-регулятора
CMD_TELEMETRY = 0x03  # Заголовок для телеметрии (используется при отправке данных)
CMD_LED = 0x04        # Команда для управления LED-лентой
CMD_GPIO = 0x05       # Команда для управления GPIO пином
CMD_MODE = 0x06       # Команда для переключения режима работы
CMD_PROBE = 0x07      # Команда для управления пробой

# Форматы пакетов
# Команда движения: заголовок (1 байт) + forward (float) + lateral (float) + yaw (float)
MOVE_CMD_STRUCT = 'Bfff'  # B: unsigned char, f: float

# Команда PID: заголовок (1 байт) + P (float) + I (float) + D (float)
PID_CMD_STRUCT = 'Bfff'

# Данные телеметрии: заголовок (1 байт) + roll (float) + pitch (float) + yaw (float) + adc_value (float) + pwm двигателей (float * 4)
TELEMETRY_STRUCT = 'Bffffffff'  # B: unsigned char, f: float

# Команда LED: заголовок (1 байт) + режим (1 байт) + R (1 байт) + G (1 байт) + B (1 байт)
LED_CMD_STRUCT = 'BBBBB'  # B: unsigned char

# Команда GPIO: заголовок (1 байт) + состояние (1 байт)
GPIO_CMD_STRUCT = 'BB'    # B: unsigned char

# Команда режима: заголовок (1 байт) + режим (1 байт)
MODE_CMD_STRUCT = 'BB'    # B: unsigned char

# Команда пробы: заголовок (1 байт) + режим (1 байт)
PROBE_CMD_STRUCT = 'BB'   # B: unsigned char

# Порт для приема UDP пакетов
UDP_PORT = 5005

# Сохраняем адрес и порт клиента для отправки телеметрии
client_address = None

# Функция для подключения к Wi-Fi сети
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Подключение к сети...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            pass
    print('Подключено к Wi-Fi. IP адрес:', wlan.ifconfig()[0])

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
    try:
        unpacked = struct.unpack(PID_CMD_STRUCT, packet)
        _, p_gain, i_gain, d_gain = unpacked
        # Обновление параметров PID-регулятора
        surface_vehicle.update_pid_settings(p_gain, i_gain, d_gain)
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
                float(adc_value)] + surface_vehicle.get_pwm_values()
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
