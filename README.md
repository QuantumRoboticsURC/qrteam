# qrteam
___
This github repo contains at the moment the following packages all in one:

- [x] qrteam/src/qrteam/**qr_drive_teleop.py** <-> before this package was **drive_teleop.py** from simple_drive
- [x] qrteam/src/qrteam/**qr_cmd_vel_mux.py** <-> before this package was **cmd_vel_mux.py** from simple_drive
- [ ] qrteam/src/qrteam/**qr_arm_teleop.py** <-> before this package was **arm_teleop.py** from simple_arm

and all these nodes runs at the same time thanks with the launcher ***roslaunch qrteam qrteam.launch***
```
roslaunch qrteam qrteam.launch
```

**But the qrteam.launch is not ready yet, so we need to wait for the qrteam.launch to be ready**
___
At the moment the package has the launcher ***roslaunch qrteam qrteam_arm.launch*** ready
This launcher execute firts two packages with a different third package
- [x] qrteam/src/qrteam/**sar_arm_test.py** <-> that contains a preliminary version of the node to test the arm
```
roslaunch qrteam qrteam_arm.launch
```
The joysticks for qr_drive_teleop and sar_arm_test could be specified by the parameters:
```
joy_drive:=/dev/input/js1
joy_arm:=/dev/input/js0
```
