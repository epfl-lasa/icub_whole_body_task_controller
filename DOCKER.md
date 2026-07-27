# iCub Whole-Body Controller — Docker Setup

YARP 2.3.72 + iCub 1.10.1 + iDynTree 0.11.2 + Gazebo 9

## Build

```bash
docker compose build --build-arg JOBS=8
```

## Launch container

```bash
xhost +local:docker          # allow Gazebo GUI on host
docker compose up -d
docker exec -it icub_whole_body_task_controller-icub-controller-dev-1 bash
```

## Simulation

Inside the container:

```bash
# 1. Start YARP name server
yarpserver

# 2. Launch Gazebo with iCub world
gazebo /opt/robotology/install/share/gazebo/worlds/icub.world

# 3. Motor GUI (connect to Gazebo controlboards)
yarpmotorgui --robot icubSim
```

## Python control script

```bash
python control_icub.py
```

## Run the C++ controller

```bash
cd /opt/hwbtc_build
./hwbtc_v61 --from /workspace/icub_whole_body_task_controller/config/wholeBodyHPIDControl.ini test
```
