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
Description=Sends a telegram after connecting to the internet
Requires=network-online.target 
After=network-online.target

[Service]
ExecStart=/home/USERNAME/send_telegram.sh

[Install]
WantedBy=multi-user.target
```
The `[Unit]` part contains a description of the service, requirements and when to start it (`After=...`). The `[Service]` part says contains the actions made by the service - in this case, relying on the `send_telegram.sh` file to be at the home directory, run the file. **Make sure to change the `USERNAME` in the `[Service]` part to your own username!**
After creating the file, run these commands to reload the services, start your service and enable it on every reboot:  
```
sudo systemctl daemon-reload
sudo systemctl start telegram_send.service
sudo systemctl enable telegram_send.service
```
Next, make sure the service is functioning by typing:  
`sudo systemctl status telegram_send.service`  
You can also reboot your RPi and make sure it works automatically during startup.

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
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/b4df1310-7a28-42a7-ae2b-0da50b886acd" width="300" height="400">

### Power Suppliers
The WBR has two sides. We will name the side with the main power switch **SIDE A**, and the side with the black switch and LED lights **SIDE B**:  
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/d28adfaf-763f-4ae3-8e9e-fd0293a38e47" width="400" height="300">

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
After powering the RPi, you can connect it to the motors. take the USB cable that connects to the back of the robot (marked USB), and connect it to the **lower usb port in the middle**, called `ttyUSB0`, as seen in the pictures below. Connecting it to a different one might cause trouble in the next steps.  
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
Right now, your RPi pinout should look something like this:  
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/e221633a-189c-4106-b5c0-0752eb553983" width="300" height="300">
<img src="https://github.com/ShlomiShatz/WBR-ready/assets/86709272/30ecef6b-32ea-4cf6-add2-9620878459e7" width="300" height="300">

## Extra - Teleop Keyboard Control
Now, we will see how to add a new node to the project, used to control the robot with the keyboard. We will add the node to the *src/node_examples* directory.  
In the *include/node_examples* directory, create a new file called `velocity_publisher_teleop_key.h` and write the following to it:
```cpp
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// The topic that we publish to
const std::string velocity_topic_name = "velocity_cmd";

// Create a class that inherits ros2 node
class CmdVelPublisher_teleop_key: public rclcpp::Node {
public:

    // Constructor 
    CmdVelPublisher_teleop_key() : Node("CmdVelPublisher_teleop_key") {
        
        // Defining publisher
        publisher = this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic_name, 10);
        geometry_msgs::msg::Twist msg;
        // Printing instructions
        printf("%s", instructions);

        // Open new thread to continuously publish Twist message
        std::thread thread([this,&msg]() {
            while(rclcpp::ok()) {
                publisher->publish(msg);
            }
        });

        while (rclcpp::ok()) {

            // get the pressed key
            key = getch();

            //'A' and 'B' represent the Up and Down arrow keys consecutively 
            if(key == 'A'||key == 'B') {
                x = Lvel(key, x);
                printf(updates, linear * x, linear, angular* th, angular, key);
            }

            //'C' and 'D' represent the Right and Left arrow keys consecutively 
            else if(key == 'C' || key == 'D') {
                th = Avel(key, th);
                printf(updates, linear * x, linear, angular* th, angular, key);
            }

            else if (moveBindings.count(key) == 1) {
                // Grab the direction data
                x = moveBindings[key][0];
                th = moveBindings[key][1];
                printf(updates, linear * x, linear, angular* th, angular, key);
            } 
            // Otherwise if it corresponds to a key in speedBindings
            else if (key == 'e') {
                if (angular < 2) {
                    angular += 0.1;
                    printf(updates, linear * x, linear, angular* th, angular, key);
                } else {
                    printf(updates_higher, linear * x, linear, angular* th, angular, key);
                } 
            }

            else if (key == 'c' && angular > 0) {
                if (angular > 0.1) {
                    angular -= 0.1;
                    printf(updates, linear * x, linear, angular* th, angular, key);
                } else {
                    printf(updates_lower, linear * x, linear, angular* th, angular, key);
                }
            }

            else if (key == 'w') {
                if (linear < 0.3) {
                    linear += 0.01;
                    printf(updates, linear * x, linear, angular* th, angular, key);
                } else {
                    printf(updates_higher, linear * x, linear, angular* th, angular, key);
                } 
            }

            else if (key == 'x' && linear > 0) {
                if (linear > 0.02) {
                    linear -= 0.01;
                    printf(updates, linear * x, linear, angular* th, angular, key);
                } else {
                    printf(updates_lower, linear * x, linear, angular* th, angular, key);
                }
            }

            // Otherwise, set the robot to stop
            else { 
                if (key == 's'||key == 'S') {
                    x = 0;
                    th = 0;
                    printf(updates, linear * x, linear, angular* th, angular, key);
                }
                // If ctrl-C (^C) was pressed, terminate the program
                else if (key == '\x03') {
                    printf("\nStopped\n");
                    rclcpp::shutdown();
                    break;
                }
                else {
                    printf(updates_invalid, linear * x, linear, angular* th, angular, key);
                }
            }

            // Update the Twist message
            msg.linear.x = x * linear;
            msg.angular.x = th * angular;
        }
        thread.join();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings {
        {'i', {1, 0}},
        {'o', {1, -1}},
        {'j', {0, 1}},
        {'l', {0, -1}},
        {'u', {1, 1}},
        {',', {-1, 0}},
        {'.', {-1, 1}},
        {'m', {-1, -1}},
        {'k', {0, 0}}
    };

    // Instructions
    const char* instructions = R"(
    Reading from the keyboard and publishing to Twist!
    ---------------------------
    Moving around:
    u    i    o
    j    k    l
    m    ,    .

    Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩
    ---------------------------
    s/S/k : stop
    w/x : Increase/decrease linear velocity by 0.01
    e/c : Increase/decrease angular velocity by 0.1
    
    0.01 <= linear velocity <= 0.3
    0.1 <= angular velocity <= 2.0

    CTRL-C to quit
    )";

    const char* updates = "\rCurrent: Linear %f (max: %f) Angular %f (max: %f) | Last command: %c          ";
    const char* updates_higher = "\rCurrent: Linear %f (max: %f) Angular %f (max: %f) | Highest achieved %c          ";
    const char* updates_lower = "\rCurrent: Linear %f (max: %f) Angular %f (max: %f) | Lowest achieved %c          ";
    const char* updates_invalid = "\rCurrent: Linear %f (max: %f) Angular %f (max: %f) | Invalid command: %c          ";
    // Init variables
    float linear = 0.3; // Linear velocity (m/s)
    float angular = 1.0; // Angular velocity (rad/s)
    float x, th; // Forward/backward/neutral direction vars
    char key = ' ';

    int getch(void) {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    // Function to check linear is in the range or not
    // Used to linearly increase/decrease the linear
    float vel_check(float curr, bool decrease = false) {
        if (decrease) curr = (curr >= -0.95) ? curr - 0.05 : -1;
        else curr = (curr <= 0.95) ? curr + 0.05 : 1;
        return curr;
    }

    // Linear vel for arrow keys
    float Lvel(char key, float x) {
        if(key == 'A') return vel_check(x, false);
        if(key == 'B') return vel_check(x, true);
        return 0;
    }
    // Angular vel for arrow keys
    float Avel(char key, float th) {
        if(key == 'C') return vel_check(th, true);
        if(key == 'D') return vel_check(th, false);
        return 0;
    }

};
```
Next, in the src file, create a new file called `velocity_publisher_teleop_key.cpp` and write to it the following:
```cpp
#include "velocity_publisher_teleop_key.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<CmdVelPublisher_teleop_key>());

  rclcpp::shutdown();

  return 0;
}
```
now, make sure to add the files to the *CMakeLists.txt* file, executable, dependencies, etc. It should look like this:
```
cmake_minimum_required(VERSION 3.8)
project(node_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) ## For interfaces
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
## For the services
find_package(wbr914_package REQUIRED)

include_directories(
include/node_examples
)

# Nodes executables
add_executable(wbr914_velocity_publisher_basic 
src/velocity_publisher_basic.cpp
include/node_examples/velocity_publisher_basic.h) 

add_executable(wbr914_velocity_publisher_continuous_basic 
src/velocity_publisher_continuous_basic.cpp
include/node_examples/velocity_publisher_continuous_basic.h) 

add_executable(wbr914_velocity_publisher_teleop_key 
src/velocity_publisher_teleop_key.cpp
include/node_examples/velocity_publisher_teleop_key.h) 

add_executable(wbr914_wiggle 
src/wiggle_robot.cpp
include/node_examples/wiggle_robot.h) 

# Add needed ROS packages to the executables
ament_target_dependencies(wbr914_velocity_publisher_basic rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_continuous_basic rclcpp geometry_msgs)
ament_target_dependencies(wbr914_wiggle rclcpp geometry_msgs)
ament_target_dependencies(wbr914_velocity_publisher_teleop_key rclcpp geometry_msgs)

install(TARGETS
 wbr914_velocity_publisher_basic
 wbr914_velocity_publisher_continuous_basic
 wbr914_wiggle
 wbr914_velocity_publisher_teleop_key
  DESTINATION lib/${PROJECT_NAME})


ament_package()
```
Now, build the package and run the file, and using the keyboard - controll the robot.

## Extra - IR Activation
Next, we will see how to get a simple read from the IR sensors. *Note - This is written when the 12V M2-ATX issue is not yet resolved, so the readings are limited.*  
The robot has 8 IR sensors, marked 1-8, where 1-5 are on the bottom half of the robot, and 6-8 are on the top of the robot. The schematics can be found [here](https://github.com/EyalBrilling/914-PC-BOT-integration-with-raspberry-pi-4-and-ROS2/blob/main/docs/tech_specifications/PC-Bot_Tech_Spec-Infra-redSensorsv1.2.pdf).  
First, we will take the relevant code from the original player driver and convert it to our use. In the **wbr914_base_driver**, we will add to the *wbr914_minimal.h* file:
```cpp
#define READANALOG        0xEF
#define NUM_IR_SENSORS    8
```
And in the *wbr914_minimal* class itself:
```cpp
void GetIRData( float *d);
int  GetAnalogSensor(int s, short * val );
```
Second, we will add the functions to the *wbr914_minimal.cpp* file:
```cpp
int wbr914_minimal::GetAnalogSensor(int s, short * val )
{
  unsigned char ret[6];

  if ( sendCmd16( s / 8, READANALOG, s % 8, 4, ret )<0 )
  {
    printf( "Error reading Analog values\n" );
  }
  // Analog sensor values are 10 bit values that have been left shifted
  // 6 bits, so right-shift them back

  uint16_t v = ( (uint16_t)BytesToInt16(  &(ret[2]) ) >> 6) & 0x03ff;

  if ( _debug )
    printf( "sensor %d value: 0x%hx\n", s, v );

  *val = (uint16_t)v;

  return 0;
}

void wbr914_minimal::GetIRData(float *d)
{
  // At 80cm Vmin=0.25V Vtyp=0.4V Vmax=0.55V
  // At 10cm delta increase in voltage Vmin=1.75V Vtyp=2.0V Vmax=2.25V
  // Therefore lets choose V=0.25V at 80cm and V=2.8V (2.25+0.55) at 10cm
  // Assume the formula for mm = 270 * (voltage)^-1.1 for 80cm to 10cm
  // Assume ADC input of 5.0V gives max value of 1023

  float adcLo = 0.0;
  float adcHi = 5.0;
  float vPerCount = (adcHi-adcLo)/1023.0;
  //  float v80 = 0.25;
  //  float deltaV = 2.25;
  //  float v10 = v80+deltaV;
  //  float mmPerVolt = (800.0-100.0)/(v80-v10);

  for (uint32_t i=0; i < NUM_IR_SENSORS; i++)
  {
    int16_t val = 0;

    GetAnalogSensor( i+8, &val );

    // Range values are useless further out than 80-90 cm
    // with the Sharp sensors, so truncate them accordingly
    if ( val > 80 )
    {
      val = 80;
    }

    // Convert 10 bit value to a distance in meters
    float meters;

    // Formula for range conversion is different for long range
    // sensors than short range ones. Use appropriate formula.
    if ( i == 5 || i == 7 )
    {
      // Beak side firing sensors are longer range sensors
      // Sharp GP2Y0A02 sensors 20-150cm
      meters = ((16933.0/((float)val - 8.0)) - 13.0)/100.0;
    }
    else
    {
      // Sharp GP2Y0A21 sensors 10-80cm
      meters = ((6787.0/((float)val - 3.0)) - 4.0)/100.0;
    }
    d[ i ] = meters;
  }
```
***Note: In the repository, the IR code is a bit simpler, with the complex conversions commented out.***  
Now we have the basic code to get the IR sensors to work and return data, when the function is called.  
We want it to work with ROS. We'll make a service that when called, return the current output of the IR sensors, in an array of Range messages form. First, we will create the service in the *src/wbr914_package/srv* directory a new file named `IRGet.srv`, and in it we will write:
```
---
sensor_msgs/Range[] response_ranges
```
Second, in the *src/wbr914_package* directory, you can find the *wbr914_node.cpp* file and inside the *src/wbr914_package/include/wbr914_package* you can find the *wbr914_node.h* file. To the **header** file, add:
```cpp
#include "sensor_msgs/msg/range.hpp"
#include <wbr914_package/srv/ir_get.hpp>
```
Now, to the constructor add:
```cpp
// Create the service that will return IR Data (Range message) on request
IRGetService = this-> create_service<wbr914_package::srv::IRGet>("ir_get_robot",
        std::bind(&CmdVelListener::get_ir_service,this,_1,_2));
```
To the private section add:
```cpp
/*
  sensor_msgs/Range[] message array is returned by the get_ir service
    uint8 ULTRASOUND=0
    uint8 INFRARED=1
    std_msgs/msg/Header header
    uint8 radiation_type
    float field_of_view
    float min_range
    float max_range
    float range
  */

void get_ir_service(const std::shared_ptr<wbr914_package::srv::IRGet::Request> request,
        std::shared_ptr<wbr914_package::srv::IRGet::Response> response);
rclcpp::Service<wbr914_package::srv::IRGet>::SharedPtr IRGetService;
```
Now we go to the **cpp** file and we add:
```cpp
void CmdVelListener::get_ir_service(const std::shared_ptr<wbr914_package::srv::IRGet::Request> request,
          std::shared_ptr<wbr914_package::srv::IRGet::Response> response) {  
            float d[NUM_IR_SENSORS] = {0};
            while (true) {
              wbr914.GetIRData(d);
              printf("Val: %f\n", d[0]);
            }
            for (int i = 0; i < NUM_IR_SENSORS; i++) {
              auto msg = std::make_unique<sensor_msgs::msg::Range>();
              msg->header.frame_id = "ir_sensor_" + std::to_string(i + 1);
              msg->header.stamp = this->now();
              msg->radiation_type = 1;
              msg->field_of_view = 0.1;
              if ( i == 5 || i == 7 ) {
                // Sharp GP2Y0A02 sensors 20-150cm
                msg->min_range = 0.2;
                msg->max_range = 1.5;
              }
              else {
                // Sharp GP2Y0A21 sensors 10-80cm
                msg->min_range = 0.1;
                msg->max_range = 0.8;
              }
              msg->range = d[i];
              response->response_ranges.push_back(*msg);
            }
            return;
          }
```
Lastly, we need to add the sensor messages to the dependencies of the project. Make sure to add the new srv file and the sensor_msgs everywhere needed in the CMakeLists.txt and package.xml files (i.e find_package in the CMakeLists file).  
Afterwards, rebuild the wbr914_package node and make sure the ir_get service is available to use. Upon being called, it will return the IR status of the entire array of sensors as needed.

## Extra - Asus Xtion Kinect Activation
In this part, we will see how to use the Asus Xtion kinect (image below) with the RPi and ros.  
First, we will use a repository that specifically made for using this kinect and ros. Clone the following [repository](https://github.com/mgonzs13/ros2_asus_xtion) into your source directory using this command:  
`git clone --recurse-submodules https://github.com/mgonzs13/ros2_asus_xtion`  
Next, make sure to install depth-image-proc package:  
`sudo apt install ros-iron-depth-image-proc`  
Now, we will install a few necessary packages:  
1. Gazebo - using the following command we will install *Ignition Gazebo*, which is the default version for ros iron: `sudo apt-get install ros-iron-ros-gz`  
2. Camera Info Manager package: `sudo apt install ros-iron-camera-info-manager`
3. Image Transport Package: `sudo apt install ros-iron-image-transport`
Next, I made a few adjustments to the `CMakeLists.txt` and `package.xml` files inside the repository, in order to make it compatible with ros iron rather than other distributions. Everywhere the package `gazebo` is required -> replace it with `ros_gz`, and everywhere that the package `gazebo_ros2_control` is required -> replace it with `gz_ros2_control`. *Everywhere else that requires a package called `gazebo_XXX` (e.g. `gazebo_plugins`) you can simply delete it. *Remember to make sure to go through all of the asus_xtion packages, both the CMakeLists.txt and the package.xml files.*
After installing the necessary files and modifying the cmake and xml files, restart the terminal (or source the ros iron environment file) and use `colcon build` to build the package. If more packages are needed, make sure to install them.  
Now, take the kinect and plug it in one of the RPi USB ports. Then, use `lsusb | grep -i "ASUS Xtion"` to make sure it is visible and connected correctly. Next, make sure the *install/setup.bash* file is sourced, and run the following command:
`ros2 launch asus_xtion asus_xtion.launch.py`  
Next, open a second terminal and ssh to the RPi ***USING THE -X FLAG***, or the next command will fail. e.g.: `ssh -X pi@192.168.0.11`
From the second terminal, after making sure the *setup* file is sourced, run the following command:
`ros2 launch asus_xtion_visualization rviz2.launch.py`
You should see the rviz2 running, and the camera output should appear. *Note: using rviz over the ssh with the RPi can be very slow.* It should look something like this:

