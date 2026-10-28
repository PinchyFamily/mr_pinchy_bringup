#!/usr/bin/env python3
"""Publish the exploreHD camera's own H.264 stream, one access unit per message.

The camera encodes H.264 in hardware (UVC node index 2), so this costs the Pi almost no CPU and
nothing is decoded or re-encoded: ffmpeg only copies packets. Each message is a
sensor_msgs/CompressedImage with format 'h264' whose data is one Annex B access unit (SPS, PPS and
one slice). The camera repeats SPS/PPS before every frame and sends an IDR keyframe every ~30
frames, so any message sequence can be decoded from its first keyframe, live or from a bag.

The stamp is the time the access unit was received here, not the sensor exposure time.
"""
import glob
import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

SLICE_NALS = (1, 5)
# NAL types that start a new access unit when they follow a slice.
AU_START_NALS = (6, 7, 8, 9)
FALLBACK_GLOB = '/dev/v4l/by-id/*exploreHD*-video-index2'


class AccessUnitSplitter:
    """Split an Annex B byte stream into access units (single-slice-per-frame streams).

    A unit ends when an SPS/PPS/SEI/AUD NAL, or a slice with first_mb_in_slice == 0, follows a slice.
    """

    def __init__(self):
        self.buf = bytearray()
        self.scan = 0
        self.has_slice = False

    def feed(self, data):
        """Add bytes; return the list of complete access units."""
        self.buf += data
        units = []
        cut_from = 0
        i = self.buf.find(b'\x00\x00\x01', self.scan)
        # A NAL is judged only once its first payload byte (type, and for slices the first
        # header bit) has arrived; otherwise rescan from it on the next feed.
        while i != -1 and i + 4 < len(self.buf):
            ntype = self.buf[i + 3] & 0x1F
            first_slice_of_frame = ntype in SLICE_NALS and bool(self.buf[i + 4] & 0x80)
            if self.has_slice and (ntype in AU_START_NALS or first_slice_of_frame):
                # A 4-byte start code has a leading zero that belongs to the new unit.
                begin = i - 1 if i > cut_from and self.buf[i - 1] == 0 else i
                units.append(bytes(self.buf[cut_from:begin]))
                cut_from = begin
                self.has_slice = False
            if ntype in SLICE_NALS:
                self.has_slice = True
            i = self.buf.find(b'\x00\x00\x01', i + 3)
        self.scan = (i if i != -1 else max(0, len(self.buf) - 2)) - cut_from
        if cut_from:
            del self.buf[:cut_from]
        return units


class HdCameraH264(Node):
    def __init__(self):
        super().__init__('hd_camera_h264')
        self.declare_parameter('device', '/dev/explorehd_h264')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('frame_id', 'hd_camera_link')
        self.declare_parameter('topic', 'h264')

        # Reliable, depth of one keyframe interval: a bag recorder or a late subscriber gets a whole
        # GOP, and subscribers that want best effort can still request it.
        qos = QoSProfile(depth=30, reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(CompressedImage, self.get_parameter('topic').value, qos)
        self.frame_id = self.get_parameter('frame_id').value
        self.proc = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run_capture, daemon=True)
        self.thread.start()

    def resolve_device(self):
        device = self.get_parameter('device').value
        if os.path.exists(device):
            return device
        found = sorted(glob.glob(FALLBACK_GLOB))
        if found:
            self.get_logger().info(f'{device} not found, using {found[0]}')
            return found[0]
        return None

    def run_capture(self):
        while not self.stop.is_set():
            device = self.resolve_device()
            if device is None:
                self.get_logger().error(
                    'no H.264 device (set the device parameter or add a udev symlink)',
                    throttle_duration_sec=10.0)
                self.stop.wait(2.0)
                continue
            cmd = [
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'v4l2',
                '-input_format', 'h264',
                '-video_size', f"{self.get_parameter('width').value}x{self.get_parameter('height').value}",
                '-framerate', str(self.get_parameter('framerate').value),
                '-i', device, '-c:v', 'copy', '-f', 'h264', 'pipe:1',
            ]
            self.get_logger().info(f'capturing {device}')
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            splitter = AccessUnitSplitter()
            while not self.stop.is_set():
                chunk = self.proc.stdout.read1(65536)
                if not chunk:
                    break
                for unit in splitter.feed(chunk):
                    self.publish(unit)
            self.proc.kill()
            self.proc.wait()
            if not self.stop.is_set():
                self.get_logger().warn('ffmpeg capture ended, restarting in 2 s')
                self.stop.wait(2.0)

    def publish(self, unit):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'h264'
        msg.data = unit
        self.pub.publish(msg)

    def shutdown(self):
        self.stop.set()
        if self.proc is not None:
            self.proc.kill()


def main():
    rclpy.init()
    node = HdCameraH264()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
