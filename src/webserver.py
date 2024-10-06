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


# Маршрут для обработки WebSocket соединений
@app.route('/ws')
@with_websocket
async def websocket(request, ws):
    while True:
        try:
            message = await ws.receive()
            if message:
                data = json.loads(message)
                forward = data.get('forward', 0)
                lateral = data.get('lateral', 0)

                # Преобразуем отклонение джойстика в скорости двигателей
                vehicle.set_motors(forward * 100, 0, lateral * 100)
        except Exception as e:
            print('WebSocket error:', e)
            break
