# ─────────────────────────────────────────────────────────────────────────────
# iCub Whole-Body Task Controller
# Base: Ubuntu 18.04
#
# Dependencies built from source:
#   - YARP  v2.3.72  (last YARP 2.x release; protocol match with 2.3.64.13)
#   - iDynTree  v0.11.2  (latest v0.x, compatible with YARP 2.x)
#   - ICUB  v1.10.1  (requires YARP >= 2.3.72)
#   - gazebo-yarp-plugins  v2.3.72  (YARP↔Gazebo bridge, controlboard plugin)
#   - icub-gazebo  (worlds & models)
#   - qpOASES  3.2.1  (built at path expected by CMakeLists.txt)
#
# Build:  docker compose build --build-arg JOBS=8
#   or    docker build -t icub-controller .
# ─────────────────────────────────────────────────────────────────────────────
FROM ubuntu:18.04

ARG JOBS=4

ENV DEBIAN_FRONTEND=noninteractive

# ─── System packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    curl \
    wget \
    git \
    ninja-build \
    pkg-config \
    build-essential \
    # YARP / iCub runtime deps
    libace-dev \
    libgsl-dev \
    libeigen3-dev \
    libxml2-dev \
    libtinyxml-dev \
    libjpeg-dev \
    libboost-all-dev \
    libode-dev \
    # IPOPT (required by iDynTree when IDYNTREE_USES_IPOPT is ON)
    coinor-libipopt-dev \
    # Qt5 (YARP GUIs: yarpmotorgui, yarpscope, etc.)
    qtbase5-dev \
    qt5-default \
    libqt5opengl5-dev \
    libqt5widgets5 \
    qtdeclarative5-dev \
    qtmultimedia5-dev \
    # Gazebo 9
    gazebo9 \
    libgazebo9-dev \
    # Misc
    swig \
    python3-dev \
    python3-pip \
    libprotobuf-dev \
    protobuf-compiler \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ─── GCC 8 ────────────────────────────────────────────────────────────────────
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y --no-install-recommends gcc-8 g++-8 \
    && update-alternatives \
        --install /usr/bin/gcc gcc /usr/bin/gcc-8 80 \
        --slave   /usr/bin/g++ g++ /usr/bin/g++-8 \
    && rm -rf /var/lib/apt/lists/*

# ─── CMake 3.16 ───────────────────────────────────────────────────────────────
RUN wget -q \
    https://github.com/Kitware/CMake/releases/download/v3.16.9/cmake-3.16.9-Linux-x86_64.sh \
    -O /tmp/cmake.sh \
    && sh /tmp/cmake.sh --skip-license --prefix=/usr/local \
    && rm /tmp/cmake.sh

# ─── Build prefix ─────────────────────────────────────────────────────────────
ENV ROBOTOLOGY_INSTALL=/opt/robotology/install
ENV ROBOTOLOGY_SRC=/opt/robotology/src
ENV ROBOTOLOGY_SUPERBUILD_ROOT=/opt/robotology-superbuild

RUN mkdir -p ${ROBOTOLOGY_INSTALL} ${ROBOTOLOGY_SRC} ${ROBOTOLOGY_SUPERBUILD_ROOT}/external

# ─── YARP v2.3.72 ─────────────────────────────────────────────────────────────
RUN git clone --depth 1 --branch v2.3.72 \
    https://github.com/robotology/yarp.git \
    ${ROBOTOLOGY_SRC}/yarp

RUN cd ${ROBOTOLOGY_SRC}/yarp \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCREATE_LIB_MATH:BOOL=ON \
        -DYARP_COMPILE_BINDINGS:BOOL=ON \
        -DCREATE_PYTHON:BOOL=ON \
        -DCREATE_GUIS:BOOL=ON \
    && ninja -j${JOBS} install \
    && sed -i 's|set(YARP_HAS_MATH_LIB FALSE)|set(YARP_HAS_MATH_LIB TRUE)|' \
        ${ROBOTOLOGY_INSTALL}/lib/cmake/YARP/YARPConfig.cmake

# ─── iDynTree v0.11.2 ─────────────────────────────────────────────────────────
RUN git clone --depth 1 --branch v0.11.2 \
    https://github.com/robotology/idyntree.git \
    ${ROBOTOLOGY_SRC}/idyntree

RUN cd ${ROBOTOLOGY_SRC}/idyntree \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_BUILD_TYPE=Release \
        -DIDYNTREE_USES_IPOPT:BOOL=ON \
        -DIDYNTREE_USES_YARP:BOOL=ON \
    && ninja -j${JOBS} install

# ─── ICUB v1.10.1 ─────────────────────────────────────────────────────────────
RUN git clone --depth 1 --branch v1.10.1 \
    https://github.com/robotology/icub-main.git \
    ${ROBOTOLOGY_SRC}/icub-main

RUN cd ${ROBOTOLOGY_SRC}/icub-main \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_BUILD_TYPE=Release \
    && ninja -j${JOBS} install

# ─── gazebo-yarp-plugins v2.3.72 ──────────────────────────────────────────────
RUN git clone --depth 1 --branch v2.3.72 \
    https://github.com/robotology/gazebo-yarp-plugins.git \
    ${ROBOTOLOGY_SRC}/gazebo-yarp-plugins

RUN cd ${ROBOTOLOGY_SRC}/gazebo-yarp-plugins \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_BUILD_TYPE=Release \
    && ninja -j${JOBS} install

# ─── icub-gazebo ──────────────────────────────────────────────────────────────
RUN git clone --depth 1 \
    https://github.com/robotology/icub-gazebo.git \
    ${ROBOTOLOGY_SRC}/icub-gazebo

RUN cd ${ROBOTOLOGY_SRC}/icub-gazebo \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL} \
    && ninja -j${JOBS} install

# ─── qpOASES ──────────────────────────────────────────────────────────────────
RUN git clone --depth 1 \
    https://github.com/robotology-dependencies/qpOASES.git \
    ${ROBOTOLOGY_SUPERBUILD_ROOT}/external/qpOASES

RUN mkdir -p ${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES \
    && cd ${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES \
    && cmake ${ROBOTOLOGY_SUPERBUILD_ROOT}/external/qpOASES \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES/lib \
    && ninja -j${JOBS}

# ─── Runtime environment ──────────────────────────────────────────────────────
ENV ROBOTOLOGY_SUPERBUILD_SOURCE_DIR=${ROBOTOLOGY_SUPERBUILD_ROOT}
ENV ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX=${ROBOTOLOGY_INSTALL}

ENV PATH=${ROBOTOLOGY_INSTALL}/bin:${PATH}

ENV LD_LIBRARY_PATH=\
${ROBOTOLOGY_INSTALL}/lib:\
${ROBOTOLOGY_INSTALL}/lib/yarp:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES/lib

ENV CMAKE_PREFIX_PATH=${ROBOTOLOGY_INSTALL}

ENV YARP_DATA_DIRS=\
${ROBOTOLOGY_INSTALL}/share/yarp:\
${ROBOTOLOGY_INSTALL}/share/iCub:\
${ROBOTOLOGY_INSTALL}/share/codyco

ENV ICUB_DIR=${ROBOTOLOGY_INSTALL}

# ─── Python bindings ──────────────────────────────────────────────────────────
ENV PYTHONPATH=\
${ROBOTOLOGY_INSTALL}/lib/python2.7/dist-packages:\
${ROBOTOLOGY_INSTALL}/lib/python3/dist-packages:\
${ROBOTOLOGY_INSTALL}/lib/python3.6/dist-packages

# ─── Gazebo environment ───────────────────────────────────────────────────────
ENV GAZEBO_MASTER_URI=http://localhost:11345
ENV GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:${ROBOTOLOGY_INSTALL}/share/gazebo
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:${ROBOTOLOGY_INSTALL}/lib
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:${ROBOTOLOGY_INSTALL}/share/gazebo/models

# ─── Build the controller ─────────────────────────────────────────────────────
WORKDIR /workspace/icub_whole_body_task_controller

COPY . .

RUN mkdir -p /opt/hwbtc_build && cd /opt/hwbtc_build \
    && cmake /workspace/icub_whole_body_task_controller \
        -DROBOTOLOGY_SUPERBUILD_ROOT=${ROBOTOLOGY_SUPERBUILD_ROOT} \
        -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_INSTALL} \
        -DCMAKE_BUILD_TYPE=Release \
    && make -j${JOBS}

ENV PATH=/opt/hwbtc_build:${PATH}

CMD ["bash"]
