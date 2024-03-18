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

### Setting Up the WBR
#### Replacing the Batteries
The WhiteBoxRobotics PC-Bot 914 is powered by two 12V lead-acid batteries located at the bottom of the robot. For most of the WBRs in the lab, they need to be replaced. Few steps to do it:
1. Disconnect the cables connecting the top part of the robot to the bottom part, in the front and back of the robot (**MAKE SURE EACH CABLE DISCONNECTED IS LABLED**):
   - Main power cable (marked MPC)
   - Infra red cables (marked IO1-IO8 or IR1-IR8)
   - The motor cables (marked MOT-L, MOT-R)
   - 12V cable
   - USB cable
   - Any other that might get in the way
![2](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/9b3e7ef3-3ecb-46a6-8187-8284486f987f =250x250)
![3](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/94fa45ea-7ea5-44ef-8840-12cc0255f1e0 =250x250)
2. Unscrew the 8 small screws that hold the bottom and the top part together, 4 on the front and 4 on the back.
![4](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/933d1b55-3560-4b49-a3fd-4461e1affb0c)
3. Separate the bottom and the top parts of the robot, and place the top part aside carefully.
![5](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/d52ee74d-ac07-4c1c-b7a1-f6e7df4cfb48)
4. Disconnect the two batteries from the cables.
![6](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/06997c57-c566-4373-a648-fc667a3580e9)
5. Remove screws of the part that holds the batteries in place and the part itself.
![7](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/45daa7f3-b661-429f-8ae3-5dc5bbd8317a)
![8](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/39d69316-0c50-46c8-a0d4-0ac1a1540a75)
6. Change both batteries and make sure it is in the right place, facing the right direction as the old ones.
7. Put the part that holds them back to place and use the screws to make sure it fits tightly *Don't forget to orginize the cables in a way that will make connecting them to the batteries possible before the screws*.
![9](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/413c17a3-adad-467e-801d-27d0a88c8d4c)
8. Connect the cables to the batteries, making sure keeping the polarity intact (black wires go to black (negative), yellow wires go to red (positive)).
![10](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/9f944824-6e60-43b6-8dac-a0fab1b5aa8e)
9. Put the top part back on the bottom part, make sure they are facing the same direction.
10. Screw the 8 small screws back and connect the cables to where each of them was.
11. Turn on the red power button and make sure it lights up.
#### Power Suppliers
The WBR has two sides. We will name the side with the main power switch **SIDE A**, and the side with the black switch and LED lights **SIDE B**:
![1](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/d28adfaf-763f-4ae3-8e9e-fd0293a38e47)
Open the A side and you will see two M2-ATXs that supply the power to the intire robot.
