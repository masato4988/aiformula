import sys
import rclpy
import rosbag2_py
import csv
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class CsvCreater:
    def __init__(self, bag_file_path):
        self.latitude, self.longitude = self.read_rosbag_data(bag_file_path)
        self.write_to_csv()

    def read_rosbag_data(self, bag_file_path):
        storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        if '/vectornav/gnss' not in type_map:
            print("/vectornav/gnssが見つかりません")
            return [], []
        
        message_type = get_message(type_map['/vectornav/gnss'])
        latitude_list = []
        longitude_list = []

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == '/vectornav/gnss':
                msg = deserialize_message(data, message_type)
                latitude_list.append(msg.latitude)
                longitude_list.append(msg.longitude)
        
        return latitude_list, longitude_list        
    
    def write_to_csv(self):
        output_csv = "course_data.csv"
        with open(output_csv, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["latitude", "longitude"])
            for lat, lon in zip(self.latitude, self.longitude):
                csv_writer.writerow([lat, lon])
        print("CSVファイルを作成しました")

def main():
    if len(sys.argv) != 2:
        print("rosbagのファイルパスを引数として渡してください")
        sys.exit(1)

    bag_file_path = sys.argv[1]

    CsvCreater(bag_file_path)    

if __name__ == "__main__":
    main()
