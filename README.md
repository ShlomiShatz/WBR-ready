## WBR-ready
This is a continuation of Eyal Brilling's WBR project, regarding adding specifics to setting up the RPi4 and WBR to work remotely.
### Setting Up the RPi
The first thing needed, is setting up the RPi4. There is an extensive guide in Eyal's project, make sure to install Ubuntu server, ros2 and the wbr914_package as specified [here](https://github.com/EyalBrilling/914-PC-BOT-integration-with-raspberry-pi-4-and-ROS2/blob/main/docs/raspberry_pi_setup.md).  
You will need to connect the RPi4 to a monitor (using the Micro-HDMI port on the RPi, make sure to use the one labled **0**) and a keyboard. Preferably make the username `Pi` and computer name `WBR$$$` (with the WBR's specific number).  
Next, using Eyal's script or changing the files directly, make sure that the RPi is connecting to the `WBR_net` router found in the robotic's lab, you will get the password from Shlomi.
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

### Adding Telegram IP Bot
The next thing we need, is to use some kind of method to know what IP the RPi got from the router in order to SSH to it.  
One possible way is to use a Telegram Bot that will automatically send a message with the IP details to a specific group.  
The bot that is made for this task is `WBR-IP-reports`, and it sends the messages directly to a group named `WBR RPI IP reports`. You can ask Shlomi to connect you to it. Now, for the script itself. You can use anything you want, my personal choice for this guide is Bash. Make a file named `send_telegram.sh` and copy this into it:  
```bash
#!/bin/bash
TOKEN=""
CHAT_ID=""
ROBOT_NAME="WBR$$$"
IP=$(hostname -I)
MESSAGE="$ROBOT_NAME IP: $IP"
{
        python3 /home/pi/ledup.py
}&
if ping -c 1 google.com &> /dev/null; then
        curl -s -X POST https://api.telegram.org/bot$TOKEN/sendMessage -d chat_id=$CHAT_ID -d text="$MESSAGE" > /dev/null
else
        echo "Trying to send IP address, but failed!"
fi
```
Make sure to take the Token and the ChatID from Shlomi, and change the robot name to the relevant number.  
After saving and exiting, make sure to test it:  
```
chmod +x send_telegram.sh
./send_telegram.sh
```
Now check if you got the message. *Note: there is a problem with connecting the WBR_net to the internet, it is possible it is not working just yet.*  
Another important step that will help you is to change the router settings to give your RPi a specific IP when connected. Ask Shlomi for help if needed.  

### Connecting the RPi to the WBR
The WBR has two sides. We will name the side with the main power switch **SIDE A**, and the side with the black switch and LED lights **SIDE B**:
![1710766432243](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/a6c4b6ee-fe3f-4d62-95a8-3e4580969ccd)
Now,

