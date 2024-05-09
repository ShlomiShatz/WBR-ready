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
