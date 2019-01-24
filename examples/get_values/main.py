################################################################################
# Temperature, Humidity and Pressure Example
#
# Created: 2019-01-18 08:47:18.498321
#
################################################################################

import streams
from bosch.bme280 import bme280

streams.serial()

try:
    # Setup sensor 
    print("start...")
    bme = bme280.BME280(I2C0)
    print("Ready!")
    print("--------------------------------------------------------")
except Exception as e:
    print("Error: ",e)
    
try:
    while True:
        temp, hum, pres = bme.get_values()
        print("Temperature:", temp, "C")
        print("Humidity:", hum, "%")
        print("Pressure:", pres, "Pa")
        print("--------------------------------------------------------")
        sleep(5000)
except Exception as e:
    print("Error2: ",e)