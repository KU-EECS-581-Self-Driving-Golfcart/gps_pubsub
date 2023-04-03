from sys import platform
from io import BufferedReader
from threading import Thread, Lock
from time import sleep
from serial import Serial
import argparse
import datetime
import os
from pynmeagps import (
    NMEAMessage,
    NMEAReader,
    POLL,
    NMEA_MSGIDS,
)

import rclpy
from rclpy.node import Node

from cadd_e_interface.msg import GPS

reading = False

def parse_args():
	parser = argparse.ArgumentParser()
	parser.add_argument('-v',
						help='Verbose. Set this flag to print readings as the script executes',
						dest='verbose',
						action='store_true')
	return parser.parse_args()


def read_messages(stream, nmeareader, verbose, publisher):
    """
    Reads, parses and prints out incoming UBX messages
    """
    # pylint: disable=unused-variable, broad-except

    global READING

    # Find timestamp (for log)
    dt = datetime.datetime.now()
    dt = str(dt).replace(' ', '_')

    log = 'log/GPS/GPS_log_{}.txt'.format(dt)
    raw_log = 'log/GPS/nmea_{}.txt'.format(dt)
    dump_output = 'out/gps.txt'

    if not os.path.exists('log'):
        os.mkdir('log')
    if not os.path.exists('out'):
        os.mkdir('out')

    if not os.path.exists('log/GPS'):
        os.mkdir('log/GPS')

    # Init readings
    lat = 0.0
    lon = 0.0
    NS = ''
    EW = ''
    speed_mps = 0.0

    while True:
        if stream.in_waiting:
            try:
                (raw_data, parsed_data) = nmeareader.read()
                if parsed_data:
                    if parsed_data._msgID in ['RMC','GLL','GGA']:
                        NS = parsed_data.NS
                        EW = parsed_data.EW
                        lat = parsed_data.lat
                        lon = parsed_data.lon
                    #elif parsed_data._msgID in ['VTG']: # VGA
                    #    speed_mps = float(parsed_data.sogk)/3.6

                    publisher.publish_gps(lat, lon)
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                break


class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(GPS, 'position', 10)

        # TODO: Remove
        timer_period = 0.0005
        self.timer = self.create_timer(timer_period, self.publish_gps)

    def publish_gps(self, lat, lon):
        msg = GPS()
        msg.lat = lat
        msg.lon = lon
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[{}, {}]"'.format(msg.lat, msg.lon))


def main(args=None):
    global READING
    args = parse_args()
    rclpy.init(args=None)

    publisher = GPSPublisher()

    # TODO: Remove
    ##rclpy.spin(minimal_publisher)
    ##exit()


    port = "/dev/ttyACM0"
    baudrate = 38400
    timeout = 0.0005
    i = 0
    with Serial(port, baudrate, timeout=timeout) as serial:

        # create NMEAReader instance
        nmr = NMEAReader(BufferedReader(serial))

        print("\nStarting read thread...\n")
        READING = True
        read_messages(serial, nmr, args.verbose, publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
