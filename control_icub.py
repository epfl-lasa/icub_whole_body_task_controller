#!/usr/bin/env python3
"""
Simple iCub controller via YARP Python bindings.
Works with Gazebo simulation and real hardware.

Usage:
    # Start yarpserver and Gazebo (or real robot) first, then:
    python3 control_icub.py
"""

import sys
import time
from typing import List, Optional
import yarp

ROBOT = "icubSim"

# Home pose (degrees) for each part.
# Joints: [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
#          wrist_prosup, wrist_pitch, wrist_yaw]
ARM_HOME  = [-25.0, 20.0, 0.0, 50.0, 0.0, 0.0, 0.0]
# Joints: [yaw, roll, pitch]
TORSO_HOME = [0.0, 0.0, 0.0]
# Joints: [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
LEG_HOME  = [25.0, 0.0, 0.0, -40.0, -15.0, 0.0]


def connect(part: str) -> Optional[yarp.PolyDriver]:
    opts = yarp.Property()
    opts.put("device", "remote_controlboard")
    opts.put("local",  f"/py_ctrl/{part}")
    opts.put("remote", f"/{ROBOT}/{part}")
    driver = yarp.PolyDriver(opts)
    if not driver.isValid():
        print(f"[WARN] Could not connect to /{ROBOT}/{part} -- skipping")
        return None
    return driver


def read_encoders(driver: yarp.PolyDriver) -> List[float]:
    enc = driver.viewIEncoders()
    n   = enc.getAxes()
    buf = yarp.Vector(n)
    enc.getEncoders(buf.data())
    return [round(buf[i], 2) for i in range(n)]


def move(driver: yarp.PolyDriver, targets: List[float], speed: float = 15.0):
    pos = driver.viewIPositionControl()
    enc = driver.viewIEncoders()
    n   = enc.getAxes()
    spd = yarp.Vector(n, speed)
    tgt = yarp.Vector(n)
    for i, v in enumerate(targets[:n]):
        tgt[i] = v
    pos.setRefSpeeds(spd.data())
    pos.positionMove(tgt.data())


def wait_done(driver: yarp.PolyDriver, timeout: float = 10.0) -> bool:
    pos   = driver.viewIPositionControl()
    start = time.time()
    while time.time() - start < timeout:
        if pos.checkMotionDone():
            return True
        time.sleep(0.1)
    return False


def demo_part(name: str, home: List[float]):
    print(f"\n-- {name} --------------------------")
    driver = connect(name)
    if driver is None:
        return
    print(f"  encoders before : {read_encoders(driver)}")
    print(f"  moving to home  : {home}")
    move(driver, home)
    done = wait_done(driver)
    print(f"  motion done     : {done}")
    print(f"  encoders after  : {read_encoders(driver)}")
    driver.close()


def main():
    yarp.Network.init()

    if not yarp.Network.checkNetwork(3.0):
        print("[ERROR] YARP network unreachable — is yarpserver running?")
        yarp.Network.fini()
        sys.exit(1)

    print("YARP network OK")

    demo_part("torso",     TORSO_HOME)
    demo_part("left_arm",  ARM_HOME)
    demo_part("right_arm", ARM_HOME)
    demo_part("left_leg",  LEG_HOME)
    demo_part("right_leg", LEG_HOME)

    yarp.Network.fini()
    print("\nDone.")


if __name__ == "__main__":
    main()
