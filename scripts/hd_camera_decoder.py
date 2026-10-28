#!/usr/bin/env python3
"""Decode the HD camera's H.264 stream into bgr8 images, on demand.

Subscribes to <ns>/h264 (see hd_camera_h264.py) and publishes decoded, optionally scaled and
rate-limited images. ffmpeg runs, and the input is subscribed, only while the output topic has a
subscriber, so an idle decoder costs no CPU or network. Run it on whichever machine should do the decoding, e.g. topside.

H.264 frames depend on earlier ones, so every frame is decoded even when `rate` is low; `rate` and
`width`/`height` reduce the published image size, not the decode cost. Decoding starts at the
first keyframe and ffmpeg needs a few seconds of stream, so a new subscriber waits ~3 s for a picture.

The output is best effort. With web_video_server add qos_profile=sensor_data to the URL.
The stamp is the time the decoded frame arrived here, not the exposure time.
"""
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

IDR_NAL = 5


def has_idr(data):
    """True if an Annex B access unit contains an IDR slice."""
    i = data.find(b'\x00\x00\x01')
    while i != -1 and i + 3 < len(data):
        if data[i + 3] & 0x1F == IDR_NAL:
            return True
        i = data.find(b'\x00\x00\x01', i + 3)
    return False


class HdCameraDecoder(Node):
    def __init__(self):
        super().__init__('hd_camera_decoder')
        self.declare_parameter('input_topic', 'h264')
        self.declare_parameter('output_topic', 'image_decoded')
        self.declare_parameter('frame_id', 'hd_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('rate', 5.0)  # output Hz; 0 = every frame
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(
            Image, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.input_topic = self.get_parameter('input_topic').value
        self.sub = None  # only subscribed while the output has a subscriber
        self.proc = None
        self.packets = queue.Queue(maxsize=90)
        self.synced = False  # True once an IDR has been queued for the running ffmpeg
        self.create_timer(1.0, self.check_demand)

    def check_demand(self):
        wanted = self.pub.get_subscription_count() > 0
        if wanted and self.proc is None:
            self.start_decoder()
        elif not wanted and self.proc is not None:
            self.stop_decoder()

    def start_decoder(self):
        rate = self.get_parameter('rate').value
        filters = ([f'fps={rate}'] if rate > 0 else []) + [f'scale={self.width}:{self.height}']
        cmd = [
            # Not -fflags nobuffer or a small -probesize: either makes ffmpeg emit nothing for a raw
            # H.264 pipe (a keyframe is larger than 64 KB). Startup takes ~3 s at the default probe.
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-analyzeduration', '0',
            '-f', 'h264', '-i', 'pipe:0',
            '-vf', ','.join(filters), '-f', 'rawvideo', '-pix_fmt', 'bgr24', 'pipe:1',
        ]
        self.get_logger().info('subscriber present, starting decoder')
        self.sub = self.create_subscription(
            CompressedImage, self.input_topic, self.on_packet, qos_profile_sensor_data)
        self.packets = queue.Queue(maxsize=90)
        self.synced = False
        self.proc = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        threading.Thread(target=self.feed, args=(self.proc, self.packets), daemon=True).start()
        threading.Thread(target=self.read, args=(self.proc,), daemon=True).start()

    def stop_decoder(self):
        self.get_logger().info('no subscribers, stopping decoder')
        self.destroy_subscription(self.sub)
        self.sub = None
        proc, self.proc = self.proc, None
        self.packets.put(None)
        proc.kill()
        proc.wait()

    def on_packet(self, msg):
        if self.proc is None or msg.format != 'h264':
            return
        data = bytes(msg.data)
        if not self.synced:
            if not has_idr(data):
                return  # wait for a keyframe before feeding the decoder
            self.synced = True
        try:
            self.packets.put_nowait(data)
        except queue.Full:
            # Decoder cannot keep up; dropping P-frames corrupts the picture, so resync.
            self.get_logger().warn('decoder behind, dropping to next keyframe',
                                   throttle_duration_sec=5.0)
            while not self.packets.empty():
                try:
                    self.packets.get_nowait()
                except queue.Empty:
                    break
            self.synced = False

    @staticmethod
    def feed(proc, packets):
        try:
            while True:
                data = packets.get()
                if data is None:
                    break
                proc.stdin.write(data)
                proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def read(self, proc):
        size = self.width * self.height * 3
        while True:
            frame = proc.stdout.read(size)
            if len(frame) < size:
                return
            out = Image()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.height = self.height
            out.width = self.width
            out.encoding = 'bgr8'
            out.step = self.width * 3
            out.data = frame
            self.pub.publish(out)

    def shutdown(self):
        if self.proc is not None:
            self.stop_decoder()


def main():
    rclpy.init()
    node = HdCameraDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
