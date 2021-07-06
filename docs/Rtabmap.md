# Гайд по сборке Rtabmap для ROS


```bash
git clone https://github.com/introlab/rtabmap.git -b 0.20.10-noetic

cd rtabmap; mkdir -p build; cd $_

cmake -DCMAKE_INSTALL_PREFIX=$HOME/catkin_ws/devel ..
cmake --build . --target install -- -j`nproc --all`

```

## References

- https://github.com/introlab/rtabmap_ros#build-from-source

