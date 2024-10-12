from lib.microdot import Microdot, send_file
from lib.microdot.websocket import with_websocket
import json
import uasyncio as asyncio
from src.sav import SurfaceVehicle
from src.imu import IMU


app = Microdot()
pins = [27, 12, 14, 13]  # Пины для подключения ESC двигателей (пример)
vehicle = SurfaceVehicle(pins)
imu = IMU()

# Загрузка сохраненных настроек PID-регулятора при запуске
def load_pid_settings():
    try:
        with open('pid_settings.json', 'r') as f:
            data = json.load(f)
            p_gain = float(data.get('pGain', 0))
            i_gain = float(data.get('iGain', 0))
            d_gain = float(data.get('dGain', 0))
            vehicle.update_pid_settings(p_gain, i_gain, d_gain)
            print('Настройки PID-регулятора загружены.')
    except Exception as e:
        print('Не удалось загрузить настройки PID-регулятора:', e)

load_pid_settings()

@app.route('/')
async def index(request):
    return send_file('src/static/index.html')

@app.route('/static/<path:path>')
async def static_files(request, path):
    if '..' in path:
        # Доступ к директориям выше запрещен
        return 'Not found', 404

    content_type = 'application/json'
    content_types = {
        '.svg': 'image/svg+xml',
        '.js': 'text/javascript',
        '.css': 'text/css',
        '.html': 'text/html'
    }
    for file_type in content_types:
        if path.endswith(file_type):
            content_type = content_types[file_type]
            break
    return send_file('src/static/' + path, content_type=content_type)

# Маршрут для получения текущих настроек PID-регулятора
@app.route('/pid_settings', methods=['GET'])
async def get_pid_settings(request):
    try:
        with open('pid_settings.json', 'r') as f:
            data = json.load(f)
            return data, 200
    except Exception as e:
        print('Ошибка при получении настроек PID-регулятора, создаю по умолчанию:', e)
        def_pid_settings = {'pGain': 5, 'iGain': 0, 'dGain': 0}
        with open('pid_settings.json', 'w') as f:
            json.dump(def_pid_settings, f)
        return def_pid_settings, 200

# Маршрут для сохранения настроек PID-регулятора
@app.route('/pid_settings', methods=['POST'])
async def pid_settings(request):
    try:
        data = request.json
        p_gain = float(data.get('pGain', 0))
        i_gain = float(data.get('iGain', 0))
        d_gain = float(data.get('dGain', 0))
        # Обновление настроек PID-регулятора в системе
        vehicle.update_pid_settings(p_gain, i_gain, d_gain)
        # Сохранение настроек в файл
        with open('pid_settings.json', 'w') as f:
            json.dump({'pGain': p_gain, 'iGain': i_gain, 'dGain': d_gain}, f)
        return {'status': 'success'}, 200
    except Exception as e:
        print('Ошибка при обработке настроек PID-регулятора:', e)
        return {'status': 'error', 'message': 'Invalid data'}, 400

# Маршрут для обработки WebSocket соединений
@app.route('/ws')
@with_websocket
async def websocket_handler(request, ws):
    while True:
        try:
            message = await ws.receive()
            if message:
                data = json.loads(message)
                speed = int(data.get('speedMultiplier', 100))
                forward = float(data.get('forward', 0))
                lateral = float(data.get('lateral', 0))
                yaw = float(data.get('yaw', 0))
                mode = data.get('mode', None)
                # Обновление режима, если он передан
                if mode:
                    vehicle.set_mode(mode)
                # Преобразуем отклонение джойстика в скорости двигателей
                vehicle.set_motors(forward * speed, lateral * speed, yaw * speed)
        except Exception as e:
            print('WebSocket error:', e)
            break

@app.route('/telemetry')
@with_websocket
async def telemetry_handler(request, ws):
    while True:
        imu.read_data()
        pwm_values = vehicle.get_pwm_values()
        telemetry = {
            'magnetometer': {
                'roll': imu.roll,
                'pitch': imu.pitch,
                'yaw': imu.yaw
            },
            'pwm': pwm_values  # Добавляем поле pwm с данными ШИМ
        }
        await ws.send(json.dumps(telemetry))
        await asyncio.sleep(0.1)


async def control_loop():
    while True:
        imu.read_data()
        current_yaw = imu.yaw
        vehicle.update_control(current_yaw)
        await asyncio.sleep(0.1)  # Задержка в 100 мс


# В основном коде добавьте запуск этой задачи:
asyncio.create_task(control_loop())
