import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
# import pymongo
import time
import json
import re
from threading import Lock

SERIAL_PORT_sen0604 = "/dev/ttyACM0"
SERIAL_PORT_sen0605 = "/dev/ttyACM1"
BAUD_RATE = 9600

MONGO_URI = "mongodb://10.183.232.115:27017/"
DB_NAME = "sensor_data"
COLLECTION_NAME = "soil_metrics"

class SoilSensorService(Node):
    def __init__(self):
        super().__init__('soil_sensor_service')
        self.srv = self.create_service(SetBool, '/read_soil_sensors', self.handle_read_sensors)
        self.lock = Lock()
        # self.client = pymongo.MongoClient(MONGO_URI)
        # self.db = self.client[DB_NAME]
        # self.collection = self.db[COLLECTION_NAME]

    def handle_read_sensors(self, request, response):
        with self.lock:
            try:
                ser1 = serial.Serial(SERIAL_PORT_sen0604, BAUD_RATE, timeout=2)
                ser2 = serial.Serial(SERIAL_PORT_sen0605, BAUD_RATE, timeout=2)
                time.sleep(2)
                self.get_logger().info("Reading sensors for 1 second each...")

                def collect_data(ser):
                    data_dict = {}
                    start = time.time()
                    while time.time() - start < 1.0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        match = re.search(r"\{.*\}", line)
                        if match:
                            try:
                                data = json.loads(match.group(0))
                                for k, v in data.items():
                                    if isinstance(v, (int, float)):
                                        data_dict.setdefault(k, []).append(v)
                            except Exception:
                                pass
                    return data_dict

                data1_dict = collect_data(ser1)
                data2_dict = collect_data(ser2)
                ser1.close()
                ser2.close()

                if not data1_dict or not data2_dict:
                    response.success = False
                    response.message = "No data read from one or both sensors."
                    return response

                def average_dict_of_lists(d):
                    return {k: sum(v)/len(v) if v else None for k, v in d.items()}

                avg1 = average_dict_of_lists(data1_dict)
                avg2 = average_dict_of_lists(data2_dict)
                merged = {**avg1, **avg2, "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")}

                try:
                    # if self.collection.count_documents({}) >= 100:
                    #     oldest = self.collection.find_one(sort=[("timestamp", 1)])
                    #     if oldest:
                    #         self.collection.delete_one({"_id": oldest["_id"]})
                    # self.collection.insert_one(merged)
                    response.success = True
                    response.message = "Measurement successful."
                except Exception as e:
                    response.success = False
                    response.message = f"MongoDB error: {e}"
                return response
            except Exception as e:
                response.success = False
                response.message = str(e)
                return response

    def average_dicts(self, dicts):
        if not dicts:
            return {}
        avg = {}
        keys = dicts[0].keys()
        for k in keys:
            vals = [d[k] for d in dicts if k in d and isinstance(d[k], (int, float))]
            if vals:
                avg[k] = sum(vals) / len(vals)
            else:
                avg[k] = dicts[0][k]
        return avg

def main(args=None):
    rclpy.init(args=args)
    node = SoilSensorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()