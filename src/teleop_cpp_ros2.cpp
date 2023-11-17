#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
        {
                {'i', {1, 0, 0, 0}},
                {'o', {1, 0, 0, -1}},
                {'j', {0, 0, 0, 1}},
                {'l', {0, 0, 0, -1}},
                {'u', {1, 0, 0, 1}},
                {',', {-1, 0, 0, 0}},
                {'.', {-1, 0, 0, 1}},
                {'m', {-1, 0, 0, -1}},
                {'O', {1, -1, 0, 0}},
                {'I', {1, 0, 0, 0}},
                {'J', {0, 1, 0, 0}},
                {'L', {0, -1, 0, 0}},
                {'U', {1, 1, 0, 0}},
                {'<', {-1, 0, 0, 0}},
                {'>', {-1, -1, 0, 0}},
                {'M', {-1, 1, 0, 0}},
                {'t', {0, 0, 1, 0}},
                {'b', {0, 0, -1, 0}},
                {'k', {0, 0, 0, 0}},
                {'K', {0, 0, 0, 0}}
        };

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
        {
                {'q', {1.1, 1.1}},
                {'z', {0.9, 0.9}},
                {'w', {1.1, 1}},
                {'x', {0.9, 1}},
                {'e', {1, 1.1}},
                {'c', {1, 0.9}}
        };


// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B

---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE : Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
CTRL-C to quit
THIS IS A EXACT REPLICA OF https://github.com/ros-teleop/teleop_twist_keyboard 
WITH SOME ADD-ONS BUT IMPLEMENTED WITH C++ and ROS2-foxy.
)";

// Init variables
float speed(8); // Linear velocity (m/s)
float turn(6); // Angular velocity (rad/s)
float x, y, z, th; // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
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

// Function to check speed is in the range or not
// Used to linearly increase/decrease the speed
float vel_check(float curr, bool decrease = false){
    if (decrease)
        curr = (curr >= -0.95) ? curr-0.05 : -1;
    else
        curr = (curr <= 0.95) ? curr+0.05 : 1;
    return curr;
}

// Linear vel for arrow keys
float Lvel(char key, float x){
    if(key=='A')
        return vel_check(x,false);
    if(key=='B')
        return vel_check(x,true);
    return 0;
}
// Angular vel for arrow keys
float Avel(char key, float th){
    if(key=='C')
        return vel_check(th,true);
    if(key=='D')
        return vel_check(th,false);
    return 0;
}


int main(int argc, char** argv){

    // node init
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("teleop");
    // define publisher
    auto _pub = node->create_publisher<geometry_msgs::msg::Wrench>("/cmd_vel", 10);
    auto _pub_1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

    auto publisherF1_ = node->create_publisher<geometry_msgs::msg::Wrench>("/Final/ForceInput/F1", 10);
    auto publisherF2_ = node->create_publisher<geometry_msgs::msg::Wrench>("/Final/ForceInput/F2", 10);

    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Wrench wrench1, wrench2;

    std_msgs::msg::Float64MultiArray FM64;
    printf("%s", msg);
    printf("\nNow top Speed is %f and turn is %f | Last command: \n", speed, turn);

    while(rclcpp::ok()){
        // get the pressed key
        key = getch();

        //'A' and 'B' represent the Up and Down arrow keys consecutively
        if(key=='A'||key=='B'){
            x = Lvel(key, x);
            y = 0.0;
            z = 0.0;
            th = 0;
            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed*x, turn*th, key);
        }

            //'C' and 'D' represent the Right and Left arrow keys consecutively
        else if(key=='C'||key=='D'){
            th = Avel(key,th);
            y = 0.0;
            z = 0.0;
            x = 0;
            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed*x, turn*th, key);
        }

        else if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }
            // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            printf("\nNow top Speed is %f and turn is %f | Last command: %c \n\t\tCurrent speed might be affected\n", speed, turn, key);
        }

            // Otherwise, set the robot to stop
        else
        { if (key=='s'||key=='S'){
                x = 0;
                y = 0;
                z = 0;
                th = 0;
                printf("\n\t\tRobot Stopped..!! \n");
                printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed*x, turn*th, key);
            }
                // If ctrl-C (^C) was pressed, terminate the program
            else if (key == '\x03')
            {
                printf("\n\n    Terminated..! \n\n");
                break;
            }
            else
                printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed*x, turn*th, key);
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        // twist.linear.y = y * speed;
        // twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // set up dimensions

        // FM64.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        // FM64.layout.dim[0].size = 2;
        // FM64.layout.dim[0].stride = 1;
        // FM64.layout.dim[0].label = "test"; // or whatever name you typically use to index vec1

        // copy in the data
        FM64.data.clear();
        FM64.data = {x * speed - th * turn, x * speed + th * turn};

        wrench1.force.y = x * speed + th * turn;
        wrench2.force.y = x * speed - th * turn;

        publisherF1_->publish(wrench1);
        publisherF2_->publish(wrench2);

        // _pub->publish(twist);
        // _pub_1->publish(FM64);
        rclcpp::spin_some(node);

    }
    return 0;

}