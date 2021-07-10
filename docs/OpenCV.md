# Гайд по сборке OpenCV

Установите требуемые библиотеки

> Если QT у вас конкретной версии стоит или собран самостоятельно, то уберите из установки

```bash
sudo apt install qt5-default libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3-dev libatlas-base-dev gfortran
```

Создаем папку для сборки и клонируем нужные репки

```bash
mkdir -p opencv_build; cd $_

git clone https://github.com/opencv/opencv.git -b 4.4.0
git clone https://github.com/opencv/opencv_contrib.git -b 4.4.0
```

Переходим в `opencv` и делаем папку для сборки

```bash
cd opencv; mkdir -p build; cd $_
```

Конфигурируем CMake

Для CPU
```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D OPENCV_EXTRA_MODULES_PATH=$(pwd)/../../opencv_contrib/modules \
        -D WITH_IPP=ON \
        -D WITH_QT=ON \
        -D WITH_TBB=ON \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_FFMPEG=ON \
        -D BUILD_DOCS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_JAVA=OFF \
        -D WITH_OPENMP=ON \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_opencv_apps=OFF \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D PYTHON_DEFAULT_EXECUTABLE=$(which python3.8) \
        ..
```
<!-- -D WITH_ITT=OFF \ -->

Для GPU (не забываем по [CUDA](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network) и [сuDNN](https://developer.nvidia.com/rdp/cudnn-download))

<!-- > Для CUDA версии выше 11 и сuDNN версии 8 и выше надо будет сделать фикс в OpenCV: https://github.com/opencv/opencv/pull/17499/files (`cmake/FindCUDNN.cmake` и `cmake/OpenCVDetectCUDA.cmake`) -->

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D OPENCV_EXTRA_MODULES_PATH=$(pwd)/../../opencv_contrib/modules \
        -D WITH_IPP=ON \
        -D WITH_QT=ON \
        -D WITH_TBB=ON \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D WITH_GSTREAMER=ON \
        -D WITH_FFMPEG=ON \
        -D BUILD_DOCS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_JAVA=OFF \
        -D WITH_OPENMP=ON \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_opencv_apps=OFF \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D PYTHON_DEFAULT_EXECUTABLE=$(which python3.8) \
        -D WITH_CUDA=ON \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D BUILD_opencv_cudacodec=OFF \
        -D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=ON \
        ..
```
<!-- -D CUDA_GENERATION=Pascal \ -->

Если OpenCV не определил архитектуру, то можно указать `-D CUDA_GENERATION=Pascal` или вашу архитектуру под сборку.

Проверяем, что все зависимости правильно нашлись и подтянулись и поехали дальше:

- Python - обязательно должны быть установлены поля:
```
--   Python 3:
--     Interpreter:                 /usr/bin/python3 (ver 3.8.5)
--     Libraries:                   /usr/lib/x86_64-linux-gnu/libpython3.8.so (ver 3.8.5)
--     numpy:                       /usr/lib/python3/dist-packages/numpy/core/include (ver 1.17.4)
--     install path:                lib/python3.8/dist-packages/cv2/python-3.8
-- 
--   Python (for build):            /usr/bin/python3.8
```
- GStreamer - проверяем установку полей
```
--   Video I/O:
--     DC1394:                      YES (2.2.5)
--     FFMPEG:                      YES
--       avcodec:                   YES (58.54.100)
--       avformat:                  YES (58.29.100)
--       avutil:                    YES (56.31.100)
--       swscale:                   YES (5.5.100)
--       avresample:                YES (4.0.0)
--     GStreamer:                   YES (1.16.2)
--     v4l/v4l2:                    YES (linux/videodev2.h)
```
- QT - вдруг надо будет GUI рендерить, проверяем установку
```
--   GUI: 
--     QT:                          YES (ver 5.12.8)
--       QT OpenGL support:         NO
--     GTK+:                        NO
--     VTK support:                 YES (ver 7.1.1)
```
- Установка - убеждаемся, что установлено будет в правильную директорию
```
--   Install to:                    /usr/local
```
- GPU - сборка с видюшечкой
```
--   NVIDIA CUDA:                   YES (ver 11.1, CUFFT CUBLAS FAST_MATH)
--     NVIDIA GPU arch:             60 61
--     NVIDIA PTX archs:
-- 
--   cuDNN:                         YES (ver 8.2.1)
```

Далее вызываем сборку и ждееееем

```bash
cmake --build . -- -j`nproc --all`
```

```bash
sudo cmake --build . --target install -- -j`nproc --all`
```

> `sudo` нужен только для установки по системному пути. Если у вас указан `CMAKE_INSTALL_PREFIX` в домашней директории, то `sudo` не требуется


## После сборки

Установите переменные в rc файле

```bash
export PATH=/usr/local/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```


## References

- https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7
- https://docs.opencv.org/4.5.2/db/d05/tutorial_config_reference.html
