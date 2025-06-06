import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
import json
import re
from threading import Lock
import random

# Mock configurations
MOCK_DATA_1 = '{"moisture": 45.6, "temp": 22.3}'
MOCK_DATA_2 = '{"moisture": 47.1, "temp": 21.9}'
MOCK_DB = []

class SoilSensorServiceMock(Node):
    def __init__(self):
        super().__init__('soil_sensor_service_mock')
        self.srv = self.create_service(SetBool, '/read_soil_sensors', self.handle_read_sensors)
        self.lock = Lock()

    def handle_read_sensors(self, request, response):
        with self.lock:
            try:
                self.get_logger().info("Simulating sensor read...")

                def simulate_data():
                    data_dict = {}
                    for _ in range(5):  # Simulate 5 lines of input
                        mock_line = random.choice([MOCK_DATA_1, MOCK_DATA_2])
                        match = re.search(r"\{.*\}", mock_line)
                        if match:
                            try:
                                data = json.loads(match.group(0))
                                for k, v in data.items():
                                    if isinstance(v, (int, float)):
                                        data_dict.setdefault(k, []).append(v)
                            except Exception:
                                pass
                        time.sleep(0.1)
                    return data_dict

                data1_dict = simulate_data()
                data2_dict = simulate_data()

                if not data1_dict or not data2_dict:
                    response.success = False
                    response.message = "Mock sensors returned no data."
                    return response

                def average_dict_of_lists(d):
                    return {k: sum(v)/len(v) if v else None for k, v in d.items()}

                avg1 = average_dict_of_lists(data1_dict)
                avg2 = average_dict_of_lists(data2_dict)
                merged = {
                    **avg1,
                    **avg2,
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
                }

                # Simulate MongoDB insertion
                if len(MOCK_DB) >= 100:
                    MOCK_DB.pop(0)
                MOCK_DB.append(merged)

                self.get_logger().info(f"Mock data stored: {merged}")
                response.success = True
                response.message = "Mock measurement successful."
                return response
            except Exception as e:
                response.success = False
                response.message = str(e)
                return response

def main(args=None):
    rclpy.init(args=args)
    node = SoilSensorServiceMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
