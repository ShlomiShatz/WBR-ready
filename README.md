# WBR-ready
This is a continuation of Eyal Brilling's WBR project, regarding adding specifics to setting up the RPi4 and WBR to work remotely.

## Setting Up the RPi
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
#### Adding a Service
Now, we need to take the `send_telegram` script and make sure it runs automatically whenever the RPi is up and connected to the network. For that, we will use a service.  
To create the service, we will go to this directory:  
`cd /etc/systemd/system`  
Next, we will create a file called `telegram_send.service` and copy the following to it:  
```
[Unit]

```

## Setting Up the WBR
### Batteries
The WhiteBoxRobotics PC-Bot 914 is powered by two 12V lead-acid batteries located at the bottom of the robot. For most of the WBRs in the lab, they need to be replaced. Few steps to do it (make sure the robot is turned off!):
1. Disconnect the cables connecting the top part of the robot to the bottom part, in the front and back of the robot (**MAKE SURE EACH CABLE DISCONNECTED IS LABLED**):
   - Main power cable (marked MPC)
   - Infra red cables (marked IO1-IO8 or IR1-IR8)
   - The motor cables (marked MOT-L, MOT-R)
   - 12V cable
   - USB cable
   - Any other that might get in the way

<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/9b3e7ef3-3ecb-46a6-8187-8284486f987f" width="300" height="300">
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/94fa45ea-7ea5-44ef-8840-12cc0255f1e0" width="300" height="300">

2. Unscrew the 8 small screws that hold the bottom and the top part together, 4 on the front and 4 on the back.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/933d1b55-3560-4b49-a3fd-4461e1affb0c" width="300" height="300">

3. Separate the bottom and the top parts of the robot, and place the top part aside carefully.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/d52ee74d-ac07-4c1c-b7a1-f6e7df4cfb48" width="300" height="300">

4. Disconnect the two batteries from the cables.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/06997c57-c566-4373-a648-fc667a3580e9" width="300" height="300">

5. Remove screws of the part that holds the batteries in place and the part itself.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/45daa7f3-b661-429f-8ae3-5dc5bbd8317a" width="300" height="300">
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/39d69316-0c50-46c8-a0d4-0ac1a1540a75" width="300" height="300">

6. Change both batteries and make sure it is in the right place, facing the same direction as the old ones.
7. Put the part that holds them back to place and use the screws to make sure it fits tightly. *Don't forget to orginize the cables in a way that will make connecting them to the batteries possible before tightning the screws*.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/413c17a3-adad-467e-801d-27d0a88c8d4c" width="300" height="300">

8. Connect the cables to the batteries, making sure keeping the polarity intact (black wires go to black (negative), yellow wires go to red (positive)).
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/9f944824-6e60-43b6-8dac-a0fab1b5aa8e" width="300" height="300">

9. Put the top part back on the bottom part, make sure they are facing the same direction.
10. Screw the 8 small screws back and connect the cables to where each of them was.
11. Turn on the red power button and the green power button to make sure they light up.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/b4df1310-7a28-42a7-ae2b-0da50b886acd" width="300" height="300">

### Power Suppliers
The WBR has two sides. We will name the side with the main power switch **SIDE A**, and the side with the black switch and LED lights **SIDE B**:
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/d28adfaf-763f-4ae3-8e9e-fd0293a38e47" width="300" height="300">

Open A side, and you will find two M2-ATX power suppliers, each with its own purpse. The one on the that is closer to the power buttons is designated for the computer itself (and in our case - the RPi4), and the other is connected to the rest of the robot (fans, IRs, motors, etc.). As seen in this picture, each power supplier is connected to a battery (on the bottom right of the picture, two yellow and one red connectors, marked blue in the picture) and gives out power through the top ports (marked ATX1 and ATX 2, green in the picture). The right one is also suppliying power through the square port.  
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/6545367b-3016-4f79-96e1-84cf40ae4a75" width="300" height="300">

You will need to replace them, more importantly the one that is closer to the power button. To do it (make sure the robot is turned off!):
1. Disconnect *CAREFULLY* all of the cables, and remember where each went to, **INCLUDING THE JUMPER ON THE LEFT POWER SUPPLIER, ATTACHED TO THE *A* SLOT (marked with a red circle in the picture above)**.
2. Unscrew the top and bottom screws.
3. Remove the power supplier and position the new one.
4. Screw it to place and connect all the cables back.
5. Turn on the power and make sure everything works properly - every yellow wire outputs 12V, red wire outputs 5V (use a multimeter if needed).

### Connecting RPi to WBR
After you have a working 5V output, you can connect the RPi to the WBR.  
Open side B (as specified in the *Power Suppliers* section). It should look empty, and with a few cables loose. That is where you wan to position your RPi. Connecting the RPi will include:
1. Power source
2. Motors
3. Off switch & LED indicators

Each section will be detailed below, but a few things beforehand:
- We will use the GPIO pins of the RPi. It carries risks and might damage the RPi. Please make sure multiple times that everything is set up properly.
- Most of what is written here is suggestive, I urge you to read the documentation and expand your knowledge in these subjects to come up with your own ideas.
- For later reference, we will use this sketch of the RPi GPIO pins as specified [here](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html):
![GPIO-Pinout-Diagram-2](https://github.com/ShlomiShatz/WBR-ready/assets/86709272/4c039bb4-9862-4777-b4c6-38c9fabcb2d4)
Every pin is numbered, some of them has specific purpse. For example, pins #2 and #4 are for 5V inputs (we will use one of them later), and pin #6 is GROUND pin, which means it is used to connect the negative wires to it.

#### Power Source
To connect the RPi to its power source, first make sure you got 5V output from the power supplier. Use a multimeter. **THE RASPBERRY TAKES 5V, INPUTTING 12V WILL CAUSE DAMAGE TO THE DEVICE**. After checking, take one 5V output (preferably from the M2-ATX that is closer to the red power switch), connect it to a red wire with a female pin input, and the ground to a black one, and connect the red to pin #4, and the black to pin #6, as shown in the picture below: 
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/72e9084a-178d-4154-a08b-291d7a69e0dc" width="300" height="300">

Turn on the power button and wait a few seconds, the RPi leds should turn on and remain stable.

#### Motors
After powering the RPi, you can connect it to the motors. take the USB cable that connects to the back of the robot (marked USB), and connect it to the **lower usb port in the middle**, called `tty0`, as seen in the pictures below. Connecting it to a different one might cause trouble in the next steps.
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/e336121b-bc8a-4e80-9b72-101e01d9fd41" width="300" height="300">
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/8492b352-69f6-43ab-bafe-82b1cb560b80" width="300" height="300">

Turn on/restart the RPi, by now you are good-to-go on connecting to the raspberry pi through ssh and running some of the node_examples found in Eyal's repository.

#### Off Switch & LED Indicators
Next thing we want to do, is to connect an off switch to the RPi, so we can turn it off safely without needing to SSH every time it turns on, as well as a LED light to indicate if it is on.  
In the terminal, run `sudo nano /boot/firmware/config.txt`  
Scroll all the way down, and write:
```
[all]
dtoverlay=gpio-shutdown,gpio_pin=23
dtoverlay=uart3
```
This will make sure that connecting the off switch (the black switch) to the RPi in pin #16 (*GPIO #23*) will enable shutting the RPi down (you can connect it to any other regular pin, I put it there because it is next to a GROUND pin). Now, take the wires labled `PRST` and connect the red to pin #16 and the black to pin #14. Now, everytime you click the black **PC Reset** switch, the RPi will shutdown safely.  
Next, we need to connect the LED. The second line we entered (uart3) makes it so that a specific pin will turn a LED on when the RPi is on. The default pin is #7 (GPIO #4). So take the wires labled `PLED` and connect the red to pin #7 and the black to pin #9. Now, the green LED (marked PC Power) will turn on whenever the RPi is on.  
Finally, we want to make sure that the orange LED is turned on to indicate that the RPi is ready for SSH. In order to do it, we need to make a simple file that will turn on a specific pin for LED output, and run it whenever the RPi is ready. In this case, we will use pin #18 (GPIO #24) and the GROUND pin next to it. First, we need to install the `gpiozero` python library in our RPi. To do it, run:
`sudo pip install gpiozero`  
Now, make a new file called `ledup.py` and in it write:
```python
from gpiozero import LED
from time import sleep

led = LED(24)
led.on()
sleep(20)
```
We turned GPIO 24# on, and waited 20 seconds before closing (so we can actually see the LED turning on). Now, connect the wires labeled `HLED` to pin #18 (red wire) and to pin #20 (black wire). Now run the python file and the orange light (marked HDD) should turn on. Now, we want it to run whenever the RPi is turned on, so we need to change our `send_telegram.sh` file to the following:  
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
We are running the python file in parallel to sending the message to the Telegram bot.
