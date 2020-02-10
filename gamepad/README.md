# PS4 Gamepad
It's a DRAFT
## Requirements
- deepracer obviously
- PS4 gamepad
- bluetooth usb controller
- python module evdev
## Installation and Setup
Following steps have to be done in the deepracer after you login using ssh.  
### 
```
# pip install evdev
# bluetoothctl
# power on
# scan on
# pair <MAC>
# trust <MAC>
# connect <MAC>
```
- Test the bluetooth device:

```
% /usr/bin/evtest
```

- Clone the deepracer-tools repository

```
% git clone https://github.com/cahya-wirawan/deepracer-tools.git
% cd deepracer-tools/gamepad
% python gamepad.py
```
## ToDo List
- Add more gamepads (XBox, ...) if I can get access to it
