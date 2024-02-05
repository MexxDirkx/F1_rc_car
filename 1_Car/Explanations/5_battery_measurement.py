# More explanation / Formulas
# https://microchip.my.site.com/s/article/Measuring-battery-voltage-using-a-microcontroller

from gpiozero import MCP3008
from time import sleep
import os

mcp3008 = MCP3008(channel=0, select_pin=7)
VDD_VOLTAGE = 3.3

R1 = 10000
R2 = 4700

while True:
    os.system('clear')

    input_voltage = mcp3008.value * VDD_VOLTAGE * ((R1 + R2) / R2)
    print(f"Measured voltage: {input_voltage} V\n{mcp3008.value}")

    sleep(1)