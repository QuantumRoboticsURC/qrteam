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
*The package satisfies the next checklist:*
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **Control of traction** with joystick publishing the cmd_vel topic
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **Change to autonomous mode** with start button
  - [x] The **led matrix change color** when autonomous mode is activated
  - [ ] The *autonomous mode is working*
- [x]  ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) Change to move base mode with options button
  - [x] The **led matrix change color** when move base mode is activated
  - [ ] The *move base mode is working*
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) When autonomous or movebase mode is activated, the **teleop mode is locked**
- [ ] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) You can *return to teleop mode at any moment*
- [ ] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) The lost_comms package are *looking for connection*
- [x] ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) You can **control the robotic arm** with Attack 3 logitech joystick
- [ ] ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) You can *abort any action* when the actions has started
![Esta es una imagen](https://zonayummy.com/arteam_github/sar_arm_test_mapping.png)
- [x] ![#1589F0](https://via.placeholder.com/15/1589F0/000000?text=+) You can
