# ROS ROBOTIC VA PROJECT

__AUTHOR__ : Tsung-Han Brian Lee <br />
__LICENSE__ : MIT

---

## Get Started

Environment Setting
  * Operating System : `Ubuntu 16` | `Raspian Strench`, `ROS Kinetic` (not necessary)
  * Python Dependencies: `pyusb`, `webrtcvad`, and `Sphinx`

Device Firmware (can be found in github/respeaker)
  * Respeaker 8 mics DFU (Device Firmware Upgrade)
  * Drivers for Raspberry Pi

To access USB device with root permission, you can add a udev `.rules` file to `/etc/udev/rules.d`
```
$ echo 'SUBSYSTEM=="usb", MODE="0666"' | sudo tee -a /etc/udev/rules.d/60-usb.rules
$ sudo udevadm control -R  # then re-plug the usb device
```
---

## Python 3 on ROS

First, you need to install `catkin-tools` and `rospkg`.
```
$ pip3 install catkin-tools rospkg
```
or
```
$ python3 -m pip install catkin-tools rospkg
```
Second, invoke Python 3 interpreter at the beginning of Python scripts.
```
1   #!/usr/bin/env python3
```
Congrats! You can develop in Python 3 now.

---

## Build Customized Keywords

Configure `dictionary.txt` and `keyword.txt` in `script/assets/pocketsphinx-data/` directory
<ul>
	<li>specific explanation
    <a href="https://github.com/respeaker/get_started_with_respeaker/issues/68">CLICK HERE</a>
  </li>
	<li>For more keywords dictionary
		<a href="https://raw.githubusercontent.com/respeaker/pocketsphinx-data/master/dictionary.txt">CLICK HERE</a>
  </li>
</ul>

---

## REFERENCE

__Python3 in ROS__
  1. https://www.cnblogs.com/h46incon/p/6207145.html
  2. https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3

__Respeaker Microphone Array__
  1. https://github.com/respeaker
