#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Most Jetson -> ROS2: odbiera odometrie T265 po UDP (send_odom.py) i publikuje nav_msgs/Odometry.

Na RPi (ROS2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 t265_odom_bridge.py
    python3 t265_odom_bridge.py --ros-args -p publish_tf:=true     # takze TF odom -> t265_link

Parametry ROS (-p nazwa:=wartosc):
    port          5006         port UDP (send_odom.py --udp ...:5006)
    topic         /t265/odom   topic nav_msgs/Odometry
    odom_frame    odom         uklad odometrii (miejsce startu kamery)
    child_frame   t265_link    uklad kamery
    publish_tf    false        TF odom -> child_frame. Wlaczaj tylko, gdy nic innego
                               nie publikuje TF z "odom" (inaczej TF sie pogryzie).

Czas w wiadomosci = zegar RPi w chwili odbioru.
"""
import socket
import struct
import threading
import time

ODOM = struct.Struct("<4sIdB3f4f3f3f")
MAGIC = b"T2O1"
# pewnosc sledzenia T265 (0..3) -> wariancja (pozycja [m^2] / predkosc)
CONF_VAR = {0: 1.0, 1: 0.1, 2: 0.01, 3: 0.001}


def decode_odom(data):
    if len(data) != ODOM.size:
        return None
    f = ODOM.unpack(data)
    if f[0] != MAGIC:
        return None
    return dict(seq=f[1], stamp_ms=f[2], confidence=f[3], pos=f[4:7], quat=f[7:11],
                vel=f[11:14], ang=f[14:17])


def world_to_body(q, v):
    """Obraca wektor z ukladu odom do ukladu kamery (q = orientacja kamery, x y z w)."""
    x, y, z, w = q
    x, y, z = -x, -y, -z   # sprzezenie = obrot odwrotny
    vx, vy, vz = v
    # v' = q v q*
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    class Bridge(Node):
        def __init__(self):
            super().__init__("t265_odom_bridge")
            port = self.declare_parameter("port", 5006).value
            topic = self.declare_parameter("topic", "/t265/odom").value
            self.odom_frame = self.declare_parameter("odom_frame", "odom").value
            self.child_frame = self.declare_parameter("child_frame", "t265_link").value
            self.publish_tf = self.declare_parameter("publish_tf", False).value
            self.pub = self.create_publisher(Odometry, topic, 10)
            self.tfb = TransformBroadcaster(self) if self.publish_tf else None
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", int(port)))
            self.sock.settimeout(0.5)
            self.get_logger().info("Czekam na odometrie z Jetsona na UDP :%d -> %s%s"
                                   % (port, topic, " + TF" if self.publish_tf else ""))
            self.count = self.lost = 0
            self.last_seq = None
            self.last = None
            self.last_rx = None
            self.running = True
            threading.Thread(target=self.loop, daemon=True).start()
            self.create_timer(5.0, self.report)

        def loop(self):
            while self.running:
                try:
                    data = self.sock.recv(1024)
                except socket.timeout:
                    continue
                except OSError:
                    break
                m = decode_odom(data)
                if m is None:
                    continue
                if self.last_seq is not None and m["seq"] > self.last_seq + 1:
                    self.lost += m["seq"] - self.last_seq - 1
                self.last_seq = m["seq"]
                self.last_rx = time.monotonic()
                self.last = m
                self.publish(m)

        def publish(self, m):
            stamp = self.get_clock().now().to_msg()
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = self.odom_frame
            o.child_frame_id = self.child_frame
            p = o.pose.pose
            p.position.x, p.position.y, p.position.z = (float(a) for a in m["pos"])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
                (float(a) for a in m["quat"])
            v = world_to_body(m["quat"], m["vel"])
            w = world_to_body(m["quat"], m["ang"])
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = v
            o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = w
            var = CONF_VAR.get(m["confidence"], 1.0)
            cov = [0.0] * 36
            for i in range(6):
                cov[i * 7] = var
            o.pose.covariance = cov
            o.twist.covariance = cov
            self.pub.publish(o)
            if self.tfb:
                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.child_frame
                t.transform.translation.x = p.position.x
                t.transform.translation.y = p.position.y
                t.transform.translation.z = p.position.z
                t.transform.rotation = p.orientation
                self.tfb.sendTransform(t)
            self.count += 1

        def report(self):
            if self.last_rx is None or time.monotonic() - self.last_rx > 2.0:
                self.get_logger().warn("Brak odometrii z Jetsona (czy send_odom.py --udp dziala?)")
                return
            m = self.last
            self.get_logger().info("opublikowano %d | zgubione %d | x %.2f y %.2f m | pewnosc %d/3"
                                   % (self.count, self.lost, m["pos"][0], m["pos"][1], m["confidence"]))

        def close(self):
            self.running = False
            self.sock.close()

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
