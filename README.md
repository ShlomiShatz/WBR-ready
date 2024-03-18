## WBR-ready
This is a continuation of Eyal Brilling's WBR project, regarding adding specifics to setting up the RPi4 and WBR to work remotely.
### Setting Up
The first thing needed, is setting up the RPi4. There is an extensive guide in Eyal's project, make sure to install Ubuntu server, ros2 and the wbr914_package as specified [here](https://github.com/EyalBrilling/914-PC-BOT-integration-with-raspberry-pi-4-and-ROS2/blob/main/docs/raspberry_pi_setup.md).  
You will need to connect the RPi4 to a monitor (using the Micro-HDMI port on the RPi, make sure to use the one labled **0**) and a keyboard. Preferably make the username Pi and computer name WBR$$$ (with the WBR's specific number).  
Next, using Eyal's script or changing the filed directly, make sure that the RPi is connecting to the WBR-net router found in the robotic's lab, you will get the password from me.
If you encounter issues, try this in the terminal:  
`sudo nano /etc/netplan/50-cloud-init.yaml` and change the file directly.  
Now restart the NetworkManager or run `sudo reboot` to initiate the new settings.  

### Remove Required Login
In this part, we will make sure the RPi is logging in automatically when powered on, without the need to enter the password (this is crucial so you do not need log in manually to the RPi when turning on).
First, run this command in the terminal:  
`sudo systemctl edit getty@tty1.service`  
Now, change the Service part to:
```
[Service]
ExecStart=
ExecStart=-/sbin/agetty --noissue --autologin USERNAME %I $TERM
Type=idle
```
Remember to change the *USERNAME* to the desired username.
Now save and close the file, and run `sudo reboot` to make sure it worked.

### Connecting the RPi to the WBR
After making sure the RPi is set up, we will want to connect it to the WBR itself to make it work.  
