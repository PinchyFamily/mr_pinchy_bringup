#!/usr/bin/env python3
"""NTRIP client ROS node using pygnssutils (NTRIP 2.0 + GGA for Swepos VRS)."""
import sys
import threading
from queue import Queue, Empty
from threading import Event, Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix, NavSatStatus
from pygnssutils import GNSSNTRIPClient


_NAV_SAT_STATUS = {
    NavSatStatus.STATUS_NO_FIX: 'NO FIX',
    NavSatStatus.STATUS_FIX: '3D',
    NavSatStatus.STATUS_SBAS_FIX: 'RTK FLOAT',
    NavSatStatus.STATUS_GBAS_FIX: 'RTK FIXED',
}


class NtripClientNode(Node):
    def __init__(self):
        super().__init__('ntrip_client')

        self.declare_parameter('use_https', False)
        self.declare_parameter('host', 'nrtk-swepos.lm.se')
        self.declare_parameter('port', 8500)
        self.declare_parameter('mountpoint', 'MSM_GEC')
        self.declare_parameter('username', 'noname')
        self.declare_parameter('password', 'password')
        self.declare_parameter('reflat', 59.204371)
        self.declare_parameter('reflon', 18.175105)
        self.declare_parameter('refalt', 20.6)
        self.declare_parameter('ggainterval', 10)
        self.declare_parameter('use_live_gga', True)
        self.declare_parameter('fix_topic', '/fix')

        self._lock = threading.Lock()
        self._using_live_gga = False
        self._ref_lat = float(self.get_parameter('reflat').value)
        self._ref_lon = float(self.get_parameter('reflon').value)
        self._ref_alt = float(self.get_parameter('refalt').value)
        self._lat = self._ref_lat
        self._lon = self._ref_lon
        self._alt = self._ref_alt
        self._fix = '3D'

        use_live_gga = self.get_parameter('use_live_gga').value
        fix_topic = self.get_parameter('fix_topic').value
        if use_live_gga:
            self.create_subscription(
                NavSatFix,
                fix_topic,
                self._fix_callback,
                qos_profile_sensor_data,
            )

        self._pub = self.create_publisher(Message, '/ntrip_client/rtcm', 10)
        self._queue = Queue()
        self._stop = Event()
        self._client = GNSSNTRIPClient(app=self)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        mountpoint = self.get_parameter('mountpoint').value
        gga_mode = 0 if use_live_gga else 1
        gga_source = 'live (/fix)' if use_live_gga else 'fixed (secrets.yaml)'
        self.get_logger().info(
            f'Connecting to {host}:{port}/{mountpoint} '
            f'as {self.get_parameter("username").value}, GGA: {gga_source}'
        )

        streaming = self._client.run(
            server=host,
            port=int(port),
            mountpoint=mountpoint,
            ntripuser=self.get_parameter('username').value,
            ntrippassword=self.get_parameter('password').value,
            https=self.get_parameter('use_https').value,
            ggamode=gga_mode,
            ggainterval=int(self.get_parameter('ggainterval').value),
            reflat=self._lat,
            reflon=self._lon,
            refalt=self._alt,
            refsep=0.0,
            output=self._queue,
            stopevent=self._stop,
        )

        if not streaming:
            self.get_logger().error('Failed to start NTRIP stream')
            sys.exit(1)

        self._thread = Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def _fix_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return

        with self._lock:
            self._lat = msg.latitude
            self._lon = msg.longitude
            self._alt = msg.altitude
            self._fix = _NAV_SAT_STATUS.get(msg.status.status, '3D')
            if not self._using_live_gga:
                self.get_logger().info(
                    f'Using live GGA position from /fix: '
                    f'({self._lat:.6f}, {self._lon:.6f}, {self._alt:.1f}m)'
                )
            self._using_live_gga = True

    def get_coordinates(self):
        """Called by pygnssutils to build NTRIP GGA sentences."""
        with self._lock:
            return {
                'lat': self._lat,
                'lon': self._lon,
                'alt': self._alt,
                'sep': 0.0,
                'fix': self._fix,
                'sip': 12,
                'hdop': 1.0,
                'diffage': 0,
                'diffstation': 0,
            }

    def _publish_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                raw, _parsed = self._queue.get(timeout=1.0)
            except Empty:
                continue

            if not rclpy.ok() or self._stop.is_set():
                break

            msg = Message()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('mountpoint').value
            msg.message = list(raw)
            self._pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = NtripClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
