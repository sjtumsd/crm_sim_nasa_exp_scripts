FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=US/Central
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ /etc/timezone

RUN DEBIAN_FRONTEND=noninteractive

WORKDIR /root/sbel

#chrono dependency installed here
RUN apt update && apt install -y --no-install-recommends --allow-unauthenticated libeigen3-dev cmake cmake-curses-gui mesa-common-dev wget git ninja-build vim

# Clean up to reduce image size
RUN ldconfig && apt-get autoclean -y && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive https://github.com/projectchrono/chrono.git -b release/8.0
RUN git clone https://github.com/uwsbel/workshop_demo.git
RUN cp -f workshop_demo/chrono_source_files/Ch* chrono/src/chrono_fsi/ && \
  cp -f workshop_demo/chrono_source_files/Viper.cpp chrono/src/chrono_models/robot/viper/
RUN mv workshop_demo/demos . && mv workshop_demo/json . && rm -rf workshop_demo

RUN mkdir chrono/build
RUN cd chrono/build && cmake ../ -G Ninja \
 -DCMAKE_BUILD_TYPE=Release \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DENABLE_MODULE_IRRLICHT=OFF \
 -DENABLE_MODULE_POSTPROCESS=OFF \
 -DENABLE_MODULE_PYTHON=OFF \
 -DENABLE_MODULE_SENSOR=OFF \
 -DENABLE_OPENMP=OFF \
 -DUSE_FSI_DOUBLE=OFF \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_FSI=ON \
 -DEigen3_DIR=/usr/lib/cmake/eigen3 \
 -DCUDA_ARCH_NAME=All \
 && ninja && ninja install

RUN mkdir demos/single_wheel_vv_mode/build
RUN cd demos/single_wheel_vv_mode/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir demos/single_wheel_real_slope_mode/build
RUN cd demos/single_wheel_real_slope_mode/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir demos/viper_real_slope/build
RUN cd demos/viper_real_slope/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir demos/curiosity_uphill/build
RUN cd demos/curiosity_uphill/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir outputs

ENV PATH "$PATH:/bin/3.3/python/bin/"
ENV BLENDER_PATH "/bin/3.3"
ENV BLENDERPIP "/bin/3.3/python/bin/pip3"
ENV BLENDERPY "/bin/3.3/python/bin/python3.10"
ENV HW="GPU"

# Install dependencies
RUN apt-get update && apt-get install -y \
	wget \ 
	libopenexr-dev \ 
	bzip2 \ 
	build-essential \ 
	zlib1g-dev \ 
	libxmu-dev \ 
	libxi-dev \ 
	libxxf86vm-dev \ 
	libfontconfig1 \ 
	libxrender1 \ 
	libgl1-mesa-glx \ 
	xz-utils

# Download and install Blender
RUN wget https://mirror.clarkson.edu/blender/release/Blender3.3/blender-3.3.1-linux-x64.tar.xz \
	&& tar -xvf blender-3.3.1-linux-x64.tar.xz --strip-components=1 -C /bin \
	&& rm -rf blender-3.3.1-linux-x64.tar.xz \
	&& rm -rf blender-3.3.1-linux-x64

# Download the Python source since it is not bundled with Blender
RUN wget https://www.python.org/ftp/python/3.10.5/Python-3.10.5.tgz \
	&& tar -xzf Python-3.10.5.tgz \
	&& cp -r Python-3.10.5/Include/* $BLENDER_PATH/python/include/python3.10/ \
	&& rm -rf Python-3.10.5.tgz \
	&& rm -rf Python-3.10.5

# Blender comes with a super outdated version of numpy (which is needed for matplotlib / opencv) so override it with a modern one
RUN rm -rf ${BLENDER_PATH}/python/lib/python3.10/site-packages/numpy

# Must first ensurepip to install Blender pip3 and then new numpy
RUN ${BLENDERPY} -m ensurepip && ${BLENDERPIP} install --upgrade pip && ${BLENDERPIP} install numpy mathutils

#install python for postprocessing
RUN apt-get update && apt-get install -y python3 python3-pip ffmpeg
RUN pip install numpy matplotlib
COPY demos.sh /root/sbel/
RUN chmod u+x demos.sh

ENTRYPOINT ["/bin/bash"]
