from gpiozero import MCP3008
import pymysql
import time

# Battery configuration
VDD_VOLTAGE = 3.3
R1 = 10000
R2 = 4700

mcp3008 = MCP3008(channel=0, select_pin=7)

# Connect pymysql
conn = pymysql.connect(host="localhost", unix_socket="/var/run/mysqld/mysqld.sock", user="Your_Username", passwd="Your_Password", db="Your_Database")
cur = conn.cursor()

# Input values
def get_battery_voltage():
    battery_voltage = mcp3008.value * VDD_VOLTAGE * ((R1 + R2) / R2)
    return battery_voltage


# Database functions
def get_time_string():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())

def clear_database():
    cur.execute("TRUNCATE TABLE Car")

def update_database():
    time = get_time_string()
    battery_voltage = get_battery_voltage()

    cur.execute("INSERT INTO Car(time, battery_voltage) VALUES(%s,%s)", (time, battery_voltage))
    conn.commit()

try:
    clear_database()

    # Update every 30 seconds
    while True:
        update_database()
        time.sleep(30)

except KeyboardInterrupt:
    cur.close()
    conn.close()