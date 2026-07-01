# ─────────────────────────────────────────────────────────────────────────────
# iCub Whole-Body Task Controller
# Base: Ubuntu 18.04
#
# Dependency strategy:
#   - robotology-superbuild v2020.02 → YARP + iCub (icub-main) + iDynTree
#   - qpOASES 3.2.1 built manually at the path expected by CMakeLists.txt
#
# Build:  docker compose build
#   or    docker build -t icub-controller .
#
# Build time: 30-60 min (superbuild compiles from source)
# Parallelism: --build-arg JOBS=N  (default 4)
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
    # IPOPT (required by iDynTree when ROBOTOLOGY_ENABLE_DYNAMICS is ON)
    coinor-libipopt-dev \
    # Qt5 (YARP GUIs: yarpmotorgui, yarpscope, etc.)
    qtbase5-dev \
    qt5-default \
    libqt5opengl5-dev \
    libqt5widgets5 \
    qtdeclarative5-dev \
    qtmultimedia5-dev \
    # Gazebo 9 (Ubuntu 18.04 default repos)
    gazebo9 \
    libgazebo9-dev \
    # Misc
    swig \
    python3-dev \
    python3-pip \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ─── GCC 8 ────────────────────────────────────────────────────────────────────
# Ubuntu 18.04 ships GCC 7.
# GCC 7 only has <experimental/filesystem>; icub-models requires <filesystem>
# which needs GCC 8+.  The project's -std=c++17 also works fine with GCC 8.
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y --no-install-recommends gcc-8 g++-8 \
    && update-alternatives \
        --install /usr/bin/gcc gcc /usr/bin/gcc-8 80 \
        --slave   /usr/bin/g++ g++ /usr/bin/g++-8 \
    && rm -rf /var/lib/apt/lists/*

# ─── CMake 3.16 ───────────────────────────────────────────────────────────────
# robotology-superbuild needs CMake >= 3.12; Ubuntu 16.04 ships 3.5.
RUN wget -q \
    https://github.com/Kitware/CMake/releases/download/v3.16.9/cmake-3.16.9-Linux-x86_64.sh \
    -O /tmp/cmake.sh \
    && sh /tmp/cmake.sh --skip-license --prefix=/usr/local \
    && rm /tmp/cmake.sh

# ─── robotology-superbuild v2020.02 ───────────────────────────────────────────
# Builds: YARP v3.3.0 · icub-main v2.1.0 · iDynTree v2.0.0
# v2020.02 is the earliest tagged release of the superbuild.
ENV ROBOTOLOGY_SUPERBUILD_ROOT=/opt/robotology-superbuild

RUN git clone --depth 1 \
    --branch v2020.02 \
    https://github.com/robotology/robotology-superbuild.git \
    ${ROBOTOLOGY_SUPERBUILD_ROOT}

RUN git config --global user.name "Docker Build" \
    && git config --global user.email "docker@build.local"

RUN cd ${ROBOTOLOGY_SUPERBUILD_ROOT} \
    && mkdir build && cd build \
    && cmake .. -G Ninja \
        -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
        -DROBOTOLOGY_ENABLE_DYNAMICS:BOOL=ON \
        -DROBOTOLOGY_USES_GAZEBO:BOOL=ON \
        -DROBOTOLOGY_USES_PYTHON:BOOL=ON \
        -DROBOTOLOGY_USES_MATLAB:BOOL=OFF \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . -- -j${JOBS}

# ─── YARP GUI tools + Python bindings ────────────────────────────────────────
# The superbuild leaves YARP_COMPILE_GUIS OFF; reconfigure YARP directly so
# Qt5 is detected and yarpmotorgui is built and installed.
# Python bindings are also enabled here (SWIG + Python 3) so that the
# superbuild PYTHON=ON flag is honoured even after this reconfigure step.
RUN cd /opt/robotology-superbuild/build/robotology/YARP \
    && cmake . \
        -DYARP_COMPILE_GUIS:BOOL=ON \
        -DQt5_DIR=/usr/lib/x86_64-linux-gnu/cmake/Qt5 \
        -DYARP_COMPILE_BINDINGS:BOOL=ON \
        -DCREATE_PYTHON:BOOL=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
        -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
    && ninja -j${JOBS} install

# ─── qpOASES 3.2.1 ────────────────────────────────────────────────────────────
# CMakeLists.txt looks for libqpOASES.so at:
#   ${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES/lib/
# and for qpOASES headers at:
#   ${ROBOTOLOGY_SUPERBUILD_ROOT}/external/qpOASES/include/
# The superbuild already clones qpOASES source into external/qpOASES; we only
# need to build it as a shared library at the path CMakeLists.txt expects.
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
ENV ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install

ENV PATH=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/bin:${PATH}

ENV LD_LIBRARY_PATH=\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/lib:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/lib/yarp:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/external/qpOASES/lib

ENV CMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install

ENV YARP_DATA_DIRS=\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/share/yarp:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/share/iCub:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/share/codyco

ENV ICUB_DIR=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install

# ─── Python bindings ──────────────────────────────────────────────────────────
# YARP installs its Python module under the install prefix; cover both the
# generic "python3" path (Debian GNUInstallDirs default) and the versioned
# one (python3.6) so import yarp works regardless of cmake's choice.
ENV PYTHONPATH=\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/lib/python3/dist-packages:\
${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/lib/python3.6/dist-packages

# ─── Gazebo environment ───────────────────────────────────────────────────────
# Mirrors what /usr/share/gazebo/setup.sh would source, extended with the
# superbuild install outputs (gazebo-yarp-plugins + icub-gazebo models).
ENV GAZEBO_MASTER_URI=http://localhost:11345
ENV GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/lib
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install/share/gazebo/models

# ─── Build the controller ─────────────────────────────────────────────────────
# Source is COPY'd here at build time; the dev compose service later mounts the
# host source over this path.  The build lives at /opt/hwbtc_build so the
# volume mount never hides the compiled binary.
WORKDIR /workspace/icub_whole_body_task_controller

COPY . .

RUN mkdir -p /opt/hwbtc_build && cd /opt/hwbtc_build \
    && cmake /workspace/icub_whole_body_task_controller \
        -DROBOTOLOGY_SUPERBUILD_ROOT=${ROBOTOLOGY_SUPERBUILD_ROOT} \
        -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_ROOT}/build/install \
        -DCMAKE_BUILD_TYPE=Release \
    && make -j${JOBS}

ENV PATH=/opt/hwbtc_build:${PATH}

CMD ["bash"]
