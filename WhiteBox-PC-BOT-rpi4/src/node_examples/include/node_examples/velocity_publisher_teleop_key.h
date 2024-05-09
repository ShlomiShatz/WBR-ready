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