# qrteam for urc
___
# FOR TESTING VELOCITY 2022/02/11
`roslaunch qrteam qrteam_veltest.launch joy_drive:=/dev/input/js0`
___
## for control rover without an active network (ROSMASTER == ROSIP)
___
This github repo contains at the moment the following packages all in one:

- [x] qrteam/**qrteam** <-> un paquete que cuenta unicamente con los launcher para cada ocasión
- [x] qrteam/**qr_base_drive_teleop** <-> before this package was **drive_teleop.py** from simple_drive
- [ ] qrteam/**qr_rover_drive_firmware** <-> before this package was **bus_can_drive.cpp** from bus_can_drive
- [ ] qrteam/**qr_rover_drive_auto** <-> before this package was **nav_auto.py & decision_auto.py** from nav_auto & decision_auto
- [ ] qrteam/**qr_rover_nav_difusa** <-> before this package was **nav_dif.py** from nav_dif
- [x] qrteam/**qr_rover_cmd_vel_mux** <-> before this package was **cmd_vel_mux.py** from simple_drive
- [x] qrteam/**qr_rover_lost_comms** <-> before this package was **lost_comms.py** from lost_comms
- [ ] qrteam/**qr_base_arm_teleop** <-> before this package was **arm_teleop.py** from simple_arm
  - [x] The SAR test version is ready qrteam/**sar_base_arm_test.py**
- [ ] qrteam/**qr_rover_arm_firmware** <-> before this package was **bus_can_arm.cpp** from bus_can_arm
- ... We also hope to have a lite version of the packages *zed_wrapper y zed_depth_sub_tutorial* available soon

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
## Important!
The joysticks for qr_drive_teleop and sar_arm_test could be specified by the parameters (js1 and js0 are default):
```
joy_drive:=/dev/input/js1
joy_arm:=/dev/input/js0
```
You have to select you model of joystick specified by the parameters ("ps5" is the default):
```
jm:=xbox
jm:=ec
```
And you must to input the parameter of the IP addres or IP addresses that you can monitor ("no ip" is the default value):
```
ip:=192.168.100.1
ip:=192.168.100.1,192.168.100.2
```
*The package satisfies the next checklist:*
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **Control of traction** with joystick publishing the cmd_vel topic
  - [ ] The velocity PID control area integrated
  - [ ] The bus_can_drive in c++ is launched
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) **Change to autonomous mode** with start button
  - [x] The **led matrix change color** when autonomous mode is activated
  - [ ] The *autonomous mode is working*
    - [ ] The zed_ros_wrapper and zed_ros_exampels in c++ are launched
    - [ ] The rover follows a gps coordinate
    - [ ] The rover search a AR tag and follows it
- [x]  ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) Change to move base mode with options button
  - [x] The **led matrix change color** when move base mode is activated
  - [ ] The *move base mode is working*
    - [ ] The rover saves the gps coordinates and return to the last if is in autonomous mode
    - [ ] THe rover return to the base
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) When autonomous or movebase mode is activated, the **teleop mode is locked**
- [ ] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) You can *return to teleop mode at any moment*
- [x] ![#f03c15](https://via.placeholder.com/15/f03c15/000000?text=+) The lost_comms package are *looking for connection*
  - [x] Decrease the velocity when the ping is over 500ms
- [x] ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) You can **control the robotic arm** with Attack 3 logitech joystick
  - [ ] The position and velocity PID control are integrated
  - [ ] The bus_can_arm in c++ is launched
  - [ ] The actios ***1a to 1d*** are coded and tested
  - [ ] The actios ***2a to 2f*** are coded and tested
- [x] ![#c5f015](https://via.placeholder.com/15/c5f015/000000?text=+) You can *abort any action* when the actions has started
- [ ] ![#1589F0](https://via.placeholder.com/15/1589F0/000000?text=+) You can *visualize all info in a interface and interact*
  - [ ] You can controll the lab on board with the interface
  - [ ] You can see all the cameras with in the interface

* For test the robotic arm*
You can guide you with the next list and image
![Esta es una imagen](https://github.com/QuantumRoboticsURC/qrteam/blob/main/manual/sar_arm_test_mapping.png)

`1a. Tomar un destornillador` / `Take a screwdriver`

`1b. Tomar un contenedor de suministros` / `Take a supply containers`

`1c. Tomar una roca de 5kg` / `Take a rock`

`1d. Jalar y tomar con una cuerda un objeto` / `Pull object by a rope`

`2a. Tomar un cache container y guardarlo` / `Save the cache container`

`2b. Desbloquear la bisagra de una puerta(duda)` / `Undo a latch on a hinged panel`

`2c. Escribir en un teclado` / `Write on a keybord`

`2d. Manejar un joystick` / `Move a joystick`

`2e Tomar e introducir una memoria USB` / `Take and insert a USB`

`2f Botones, interruptores y girar mandos` / `Buttons, switches and joysticks`
