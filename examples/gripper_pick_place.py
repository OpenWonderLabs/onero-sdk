"""Arm + Onero CAN gripper pick-and-place example.

The gripper shares the Arm-owned CAN transport. TCP, payload, and gravity
compensation are configured separately.

Hardware example:
    - A1R arm and Onero gripper on can0
    - Gripper command ID 0x08, feedback ID 0x18
"""

import time

from onero_sdk import Arm, OneroGripper


gripper = OneroGripper()
arm = Arm(
    "A1R",
    channel="can0",
    end_effector=gripper,
    with_gripper=True,  # independent empirical gravity profile
)

PRE_GRASP = [0.0, 0.8, 0.0, -1.0, 0.0, 0.5, 0.0]
GRASP_POS = [0.0, 1.0, 0.0, -1.2, 0.0, 0.5, 0.0]
PLACE_POS = [0.5, 0.8, 0.0, -1.0, 0.0, 0.5, 0.0]

try:
    arm.enable()
    arm.gripper.open()
    time.sleep(0.5)
    arm.movej(PRE_GRASP, duration=2.0)
    arm.movej(GRASP_POS, duration=1.5)

    arm.gripper.grasp()
    time.sleep(0.5)
    arm.movej(PRE_GRASP, duration=1.5)
    arm.movej(PLACE_POS, duration=2.0)

    arm.gripper.open()
    time.sleep(0.5)
    arm.home(duration=3.0)
finally:
    # disable() removes the arm's holding torque. Physically support the arm
    # before this point in a real setup.
    arm.disable()
    arm.close()

print("Pick-and-place complete.")
