import socket
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Gprmc
from sensor_msgs.msg import NavSatFix, NavSatStatus
#from velodyne_gps import gprmc_parser
import threading


sensor_ip = "192.168.1.201"
bind_ip = "192.168.1.101"
port = 8308
buffer_size = 4096
publish_period = 2



class GprmcParser():
    DATA_START_TOKEN = '$GPRMC'.encode()
    DATA_END_TOKEN = '\r\n'.encode()

    def parse(self, data):
        start = data.find(GprmcParser.DATA_START_TOKEN)
        end = data.find(GprmcParser.DATA_END_TOKEN)

        if end == -1 or start == -1:
            return None

        data = data[start:end]
        data_str_list = data.decode('ascii').split(',')
        try:
            if len(data_str_list) != 13:
                return None
        except:
            print("ERROR: BAD DATA OR PARSING")
            print(data_str_list)
            return None

        msg = Gprmc()
        msg.utc_seconds = float(data_str_list[1])
        msg.position_status = data_str_list[2]

        if msg.position_status != "A":
            return None

        msg.lat = float(data_str_list[3])# / 100.0
        msg.lat_dir = data_str_list[4]
        msg.lon = float(data_str_list[5])# / 100.0
        msg.lon_dir = data_str_list[6]
        msg.speed = float(data_str_list[7])
        msg.track = float(data_str_list[8])
        msg.date = data_str_list[9]
        msg.mag_var = float(data_str_list[10])
        msg.mag_var_direction = data_str_list[11]
        msg.mode_indicator = data_str_list[12][0]
        return msg

    def gprmc_to_nav_sat_fix(self, gprmc_msg):
        msg = NavSatFix()

        msg.latitude = gprmc_msg.lat
        msg.longitude = gprmc_msg.lon
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.altitude = 0.0
        #msg.position_covariance = np.eye(3, 3).flatten()
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg


class VelodyneGpsListener(Node):
    def __init__(self):
        super().__init__('velodyne_gps_node')
        self.gprmc_publisher_ = self.create_publisher(Gprmc, 'velodyne_gps_gprmc', 10)
        self.navsatfix_publisher_ = self.create_publisher(NavSatFix, 'velodyne_gps_navsatfix', 10)
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind((bind_ip, port))
        except:
            self.get_logger().fatal(f'failed to bind socket! {bind_ip}:{port}')
            # exit()
            return
        self.get_logger().info(f'Node binded to socket: {bind_ip}:{port} successfully.')
        self.parser_ = GprmcParser()
        self.gprmc_msg = None
        self.navsatfix_msg = None
        self.msg_update_thread = threading.Thread(target=self.update_msgs)
        self.msg_update_thread.start()
        self.timer_ = self.create_timer(publish_period, self.publish_udp_data)

    def __del__(self):
        self.msg_update_thread.join(timeout=1)
        self.destroy_node()


    def publish_udp_data(self):
        # Publish the messages
        if (self.gprmc_msg is None):
            return
        self.get_logger().info("Publishing to topic")
        self.gprmc_publisher_.publish(self.gprmc_msg)
        self.navsatfix_publisher_.publish(self.navsatfix_msg)

    def update_msgs(self):
        while (True):
            data, addr = self.udp_socket.recvfrom(buffer_size)
            if addr[0] != sensor_ip:
                self.get_logger().error(f'Got Packet for unexpected source: {addr}, expecting:{sensor_ip}.')
                continue
            self.get_logger().debug(f'Got Packet from sensor: {sensor_ip})')

            # Parse the received data
            gprmc_msg = self.parser_.parse(data)
            if gprmc_msg is None:
                self.get_logger().error(f'Packet Data was invalid.')
                continue
            navsatfix_msg = self.parser_.gprmc_to_nav_sat_fix(gprmc_msg)

            #time = self.get_clock().now()
            #self.get_logger().info(f'[{time}]: {gprmc_msg.lat} {gprmc_msg.lat_dir} {gprmc_msg.lon} {gprmc_msg.lon_dir}')
            self.get_logger().info(f'{gprmc_msg.lat} {gprmc_msg.lat_dir} {gprmc_msg.lon} {gprmc_msg.lon_dir}')

            # update msgs
            self.gprmc_msg = gprmc_msg
            self.navsatfix_msg = navsatfix_msg


def main(args=None):
    rclpy.init(args=args)
    velodyne_gps_node = VelodyneGpsListener()
    rclpy.spin(velodyne_gps_node)
    velodyne_gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
