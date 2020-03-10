import smbus2
import bme280
import Adafruit_DHT
from time import sleep
from sensors import mhz19b

port = 1
address = 0x76
bus = smbus2.SMBus(port)

#mhz = mhz19b()
#dht = Adafruit_DHT.DHT22
#pin = 4

calibration_params = bme280.load_calibration_params(bus, address)

while True:
#    co2, T_co2 = mhz.requestAndReadValues()
    data = bme280.sample(bus, address, calibration_params)
#    hum22, t22 = Adafruit_DHT.read_retry(dht, pin)
    print(" Tbmp =", round(data.temperature, 2), "°C")
#    print(" Tdht =", round(t22, 2), "°C")
    print("RHbmp =", round(data.humidity, 2), "%")
#    print("RHdht =", round(hum22, 2), "%")
#    print("CO2 =", co2, "ppm")
    print("--------------------")
    sleep(5)
