from lib.microdot import Microdot, send_file
from lib.microdot.websocket import with_websocket
import json

from src.sav import SurfaceVehicle


app = Microdot()
pins = [27, 12, 14, 13]  # Пины для подключения ESC двигателей (пример)
vehicle = SurfaceVehicle(pins)


@app.route('/')
async def index(request):
    return send_file('src/static/index.html')

@app.route('/static/<path:path>')
async def index(request, path):
    if '..' in path:
        # directory traversal is not allowed
        return 'Not found', 404
    return send_file('src/static/' + path, content_type='image/svg+xml')


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
                print(forward, lateral)
                # Преобразуем отклонение джойстика в скорости двигателей
                vehicle.set_motors(forward * speed, 0, lateral * speed)
        except Exception as e:
            print('WebSocket error:', e)
            break
