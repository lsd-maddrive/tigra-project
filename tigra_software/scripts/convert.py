import yaml
import numpy as np
import tf


with open("route.yaml") as f:
    data = yaml.safe_load(f)

for p in data["points"]:
    quat = p["orientation"]
    quat = tuple(quat.values())

    euler = tf.transformations.euler_from_quaternion(quat)
    print(np.rad2deg(euler[2]))