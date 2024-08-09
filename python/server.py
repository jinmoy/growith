import serial
import mysql.connector
from datetime import datetime

# 시리얼 포트 설정
ser = serial.Serial('COM3', 9600)  # 아두이노가 연결된 시리얼 포트를 사용하세요

# MySQL 연결 정보
host = "ls-71260c981793be950570247e3cf33339418bb08f.chams4yk6fob.ap-northeast-2.rds.amazonaws.com"
user = "dbmasteruser"
password = "9=,+D9dm[HUYD}?bD~T5$t*bK0YV6y%N"
database = "dbmonitorsensor"

def connect_to_database():
    return mysql.connector.connect(
        host=host,
        user=user,
        password=password,
        database=database
    )

def insert_data(table, value):
    connection = None
    cursor = None
    try:
        connection = connect_to_database()
        cursor = connection.cursor()
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        insert_query = f"INSERT INTO {table} (date, value) VALUES (%s, %s)"
        cursor.execute(insert_query, (current_time, value))
        connection.commit()
        print(f"Data Inserted into {table} Table Successfully: {current_time}, {value}")
    except mysql.connector.Error as err:
        print(f"Failed to insert data into {table} table: {err}")
    finally:
        if cursor:
            cursor.close()
        if connection and connection.is_connected():
            connection.close()

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        if "Temperature:" in data:
            temperature = float(data.split(":")[1].strip())
            insert_data("water_temp", temperature)
        elif "DO:" in data:
            do_value = float(data.split(":")[1].strip())
            insert_data("do", do_value)
        elif "pH:" in data:
            ph_value = float(data.split(":")[1].strip())
            insert_data("ph", ph_value)
