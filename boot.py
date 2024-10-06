import network
import time

ap_if = network.WLAN(network.AP_IF)
ap_if.active(True)
ap_if.config(essid='[msur] drop box', password='12345678')

print('Инициализация точки доступа')
while not ap_if.active():
    print('.', end='')
    time.sleep(1)
print('\nТочка доступа инициализирвоана: {}'.format(ap_if.ifconfig()[0]))
