#!/usr/bin/env python

import asyncio
import threading

from numpy.lib.arraysetops import isin
import websockets
import socket
import json
import functools
from threading import Thread
import rospy
import numpy as np

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from tf.transformations import quaternion_from_euler


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(("10.255.255.255", 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = "127.0.0.1"
    finally:
        s.close()
    return IP


class ImuState:
    def __init__(self, frame_id, accel_cov, gyro_cov, orient_cov):
        self.pub = rospy.Publisher("imu", Imu, queue_size=10)
        self.state = Imu()
        self.state_initial_update = [False] * 3

        self.state.linear_acceleration_covariance = accel_cov
        self.state.angular_velocity_covariance = gyro_cov
        self.state.orientation_covariance = orient_cov
        self.state.header.frame_id = frame_id

    def update_orient(self, q):
        self.state.orientation.x = q[0]
        self.state.orientation.y = q[1]
        self.state.orientation.z = q[2]
        self.state.orientation.w = q[3]

        self.state_initial_update[0] = True

    def update_accel(self, x, y, z):
        self.state.linear_acceleration.x = x
        self.state.linear_acceleration.y = y
        self.state.linear_acceleration.z = z

        self.state_initial_update[1] = True

    def update_gyro(self, x, y, z):
        self.state.angular_velocity.x = x
        self.state.angular_velocity.y = y
        self.state.angular_velocity.z = z

        self.state_initial_update[2] = True

    def publish(self):
        if not all(self.state_initial_update):
            return

        self.state.header.stamp = rospy.Time.now()
        self.pub.publish(self.state)


class GpsState:
    def __init__(self, frame_id, cov):
        # latch - to send data after connect
        self.pub = rospy.Publisher("gps", NavSatFix, queue_size=10, latch=True)

        self.state = NavSatFix()
        self.state.header.frame_id = frame_id
        self.state.position_covariance = cov
        self.state.status.service = NavSatStatus.SERVICE_GPS
        self.state.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.updated = False
        self.sender = None

    def start_sender(self):
        if self.sender is None:
            self.sender = threading.Timer(interval=1, function=self._send_cb)
            self.sender.start()

    def _send_cb(self):
        self.sender = None
        self._publish()

    def update_state(self, latitude, longitude, altitude):
        if any(d is None for d in [latitude, longitude]):
            self.state.status.status = NavSatStatus.STATUS_NO_FIX
            self.state.latitude = 0
            self.state.longitude = 0
            self.state.altitude = 0
            return

        self.state.status.status = NavSatStatus.STATUS_FIX
        self.state.latitude = latitude
        self.state.longitude = longitude
        self.state.altitude = altitude

    def publish(self):
        self.start_sender()

    def _publish(self):
        self.state.header.stamp = rospy.Time.now()
        self.pub.publish(self.state)


async def ws_handler(websocket, path, extra_argument):
    imu_state = extra_argument["imu_state"]
    gps_state = extra_argument["gps_state"]

    async for message in websocket:
        if path == "//sensors":
            message = json.loads(message)

            if not isinstance(message, dict):
                rospy.logwarn_throttle(
                    1, "Message has invalid format - check app settings"
                )
                continue

            try:
                # Sample: {"timestamp":1624386177562,"sensors":[
                # {"name":"Accelerometer","value0":0.09303284,"value1":-0.3250122,"value2":9.86026},
                # {"name":"Gyroscope","value0":3.6621094E-4,"value1":2.746582E-4,"value2":-0.0011138916},
                # {"name":"Orientation","value0":209.60489,"value1":1.9141718,"value2":0.57536465},
                # {'name': 'Rotation vector', 'value0': 0.003773575, 'value1': -0.0034588643, 'value2': 0.9765298, 'value3': 0.21532162, 'value4': 0.0},
                # {"name":"Location","value0":null,"value1":null,"value2":null}]}
                sensors_data = message.get("sensors")
                if sensors_data is None:
                    continue

                # X on phone goes right from screen
                # Y goes up
                def orientation_handler(data):
                    rospy.logwarn_throttle(
                        1,
                        "Orientation is deprecated! Only rotation vector should be used!",
                    )
                    # Deprecated
                    azimuth = data["value0"]
                    pitch = data["value1"]
                    roll = data["value2"]

                    q = quaternion_from_euler(
                        np.deg2rad(roll),
                        np.deg2rad(pitch),
                        np.deg2rad(azimuth),
                        axes="sxyz",
                    )
                    imu_state.update_orient(q)

                def rotation_vector_handler(data):
                    x = data["value0"]
                    y = data["value1"]
                    z = data["value2"]
                    w = data["value3"]
                    q = [x, y, z, w]
                    
                    if not all(q):
                        rospy.logwarn(f"Invalid quaternion data: {q}")
                        return
                        
                    imu_state.update_orient(q)

                    # a = euler_from_quaternion(q, axes='sxyz')
                    # print(f'>>> {np.rad2deg(a)} / {q}')

                def gyroscope_handler(data):
                    x = data["value0"]
                    y = data["value1"]
                    z = data["value2"]
                    imu_state.update_gyro(x, y, z)

                def accelerometer_handler(data):
                    x = data["value0"]
                    y = data["value1"]
                    z = data["value2"]
                    imu_state.update_accel(x, y, z)

                def location_handler(data):
                    latitude = data["value0"]
                    longitude = data["value1"]
                    altitude = data["value2"]

                    gps_state.update_state(latitude, longitude, altitude)

                name_2_obj = {
                    "Accelerometer": accelerometer_handler,
                    "Orientation": orientation_handler,
                    "Rotation vector": rotation_vector_handler,
                    "Gyroscope": gyroscope_handler,
                    "Location": location_handler,
                }

                for msg in sensors_data:
                    ds_name = msg["name"]
                    handler = name_2_obj.get(ds_name)
                    if handler is None:
                        rospy.logwarn_throttle(
                            5, f"Unknown data source found: {ds_name}"
                        )
                        continue
                    handler(msg)

                imu_state.publish()
                gps_state.publish()

            except Exception as e:
                rospy.logerr(f"Exception: {e}")


def start_server(loop, server):
    loop.run_until_complete(server)
    loop.run_forever()


def main():
    hostname = socket.gethostname()
    IPAddr = get_ip()
    print("Your Computer Name is: " + hostname)
    print("Your Computer IP Address is: " + IPAddr)

    rospy.init_node("phone_data_sender")

    # ROS params
    param_port = rospy.get_param("~port", 5000)
    param_frame_id_prefix = rospy.get_param("~frame_id_prefix", "phone_sensors")

    ## IMU
    param_accel_cov = rospy.get_param(
        "~accel_cov", [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    )
    param_gyro_cov = rospy.get_param(
        "~gyro_cov", [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
    )
    param_orient_cov = rospy.get_param(
        "~orient_cov", [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    )

    imu_frame_id = f"{param_frame_id_prefix}_imu"

    imu_state = ImuState(
        frame_id=imu_frame_id,
        accel_cov=param_accel_cov,
        gyro_cov=param_gyro_cov,
        orient_cov=param_orient_cov,
    )

    ## GPS
    param_gps_cov = rospy.get_param("~gps_cov", [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01])

    gps_frame_id = f"{param_frame_id_prefix}_gps"

    gps_state = GpsState(frame_id=gps_frame_id, cov=param_gps_cov)

    ## Server init
    bound_handler = functools.partial(
        ws_handler, extra_argument={"imu_state": imu_state, "gps_state": gps_state}
    )

    loop = asyncio.new_event_loop()
    _server = websockets.serve(bound_handler, "0.0.0.0", param_port, loop=loop)

    t = Thread(target=start_server, args=(loop, _server))
    t.start()

    try:
        rospy.spin()
    finally:
        loop.stop()


if __name__ == "__main__":
    main()
