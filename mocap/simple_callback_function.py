# Simulate a sensor that sends data
def simulate_sensor(send_callback):
    for i in range(20):
        data = f"Sensor: {i}"
        send_callback(data)
        time.sleep(1)

# Lambda function for the sensor callback
sensor_callback = lambda data: print("Received sensor data:", data) if data.startswith("Sensor:") else None

# Simulate receiving sensor data and invoke the lambda callback
if __name__ == "__main__":
    import time

    # Simulate the sensor and pass the lambda callback
    simulate_sensor(sensor_callback)
