#!/bin/bash
# Subscribes to /zed_f9p/fix and calls service /datum with GPS lat/lon

latitude=$(rostopic echo -n1 /zed_f9p/fix/latitude | head -n1)
longitude=$(rostopic echo -n1 /zed_f9p/fix/longitude | head -n1)
altitude=$(rostopic echo -n1 /zed_f9p/fix/altitude | head -n1)

$(rosservice call /datum "geo_pose:
  position:
    latitude: $latitude
    longitude: $longitude
    altitude: $altitude
  orientation:
    x: 0.014415429457016724
    y: -0.010045033975473065
    z: -0.34507838347250286
    w: 0.9103334021823095")
