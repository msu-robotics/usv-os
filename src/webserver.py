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

@app.route('/')
async def index(request):
    return send_file('src/static/index.html')

@app.route('/static/<path:path>')
async def index(request, path):
    if '..' in path:
        # directory traversal is not allowed
        return 'Not found', 404

    content_type = 'application/json'
    content_types = {
        '.svg': 'image/svg+xml',
        '.js': 'text/javascript'
    }
    for file_type in content_types:
        if path.endswith(file_type):
            content_type = content_types[file_type]
            break
    return send_file('src/static/' + path, content_type=content_type)


# Маршрут для обработки WebSocket соединений
@app.route('/ws')
@with_websocket
async def websocket(request, ws):
    while True:
        try:
            message = await ws.receive()
            if message:
                data = json.loads(message)
                speed = int(data.get('speedMultiplier', 100))
                forward = float(data.get('forward', 0))
                lateral = float(data.get('lateral', 0))
                yaw = float(data.get('yaw', 0))
                # Преобразуем отклонение джойстика в скорости двигателей
                vehicle.set_motors(forward * speed, lateral * speed, yaw * speed)
        except Exception as e:
            print('WebSocket error:', e)
            break


@app.route('/telemetry')
@with_websocket
async def websocket(request, ws):
    print('connected to telemetry')
    while True:
        imu.read_data()
        telemetry = {'magnetometer': {
            'roll': imu.roll,
            'pitch': imu.pitch,
            'yaw': imu.yaw
        }}
        await ws.send(json.dumps(telemetry))
        await asyncio.sleep(0.1)

