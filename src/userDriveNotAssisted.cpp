#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <termios.h>

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN "\033[1;32m"
#define RED "\033[1;31m"

// Initializing the publisher and the message of type geometry_msgs::Twist
ros::Publisher pub_vel;
geometry_msgs::Twist robot_vel;

// Define global variables for the distance threshold and the initial velocity
float th_min = 0.8;
float vel = 0.5;

// Define initial angular velocity
float twist_vel = 1;

// Define variables for teleop_key speed directions
int angle_dir = 0;
int lin_dir = 0;

// Define the menu to show the possible commands
std::string menuTeleop = R"(You can move the robot with the following commands:

   7    8    9
   4    5    6
   1    2    3
   
Press:
*/ /: increase/decrease all speeds by 10%
+/- : increase/decrease only linear speed by 10%
a/z : increase/decrease only angular speed by 10%
R/r : reset all speeds
e   : reset only linear speed
w   : reset only angular speed
CTRL-C to quit
Notice that any other input will stop the robot.

)";

/*
In order to use non-blocking inputs from the keyboard, let's take these function to select non-blocking mode and then restore to blocking mode.
Functions are taken from the repository of github kbNonblock.c at https://gist.github.com/whyrusleeping/3983293
*/
void RestoreKeyboardBlocking(struct termios *initial_settings)
{
    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, initial_settings);
}

void SetKeyboardNonBlock(struct termios *initial_settings)
{

    struct termios new_settings;

    // Store old settings, and copy to new settings
    tcgetattr(fileno(stdin), initial_settings);

    // Make required changes and apply the settings
    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

/*
Function to have non-blocking keyboard input
*/
int GetChar(void)
{
    struct termios term_settings;
    char c = 0;

    SetKeyboardNonBlock(&term_settings);

    c = getchar();

    // Not restoring the keyboard settings causes the input from the terminal to not work right
    RestoreKeyboardBlocking(&term_settings);

    return c;
}

/*
Function that returns true if an input has arrived and chooses what to do based on it:
it is dedicated to the change of the velocity communicating with the user interface.
*/
void UpdateVelocity(char input)
{
    switch (input)
    {

    // change direction
    case '7':
        lin_dir = 1;
        angle_dir = 1;
        break;
    case '8':
        lin_dir = 1;
        angle_dir = 0;
        break;
    case '9':
        lin_dir = 1;
        angle_dir = -1;
        break;
    case '4':
        lin_dir = 0;
        angle_dir = 1;
        break;
    case '5':
        lin_dir = 0;
        angle_dir = 0;
        break;
    case '6':
        lin_dir = 0;
        angle_dir = -1;
        break;
    case '1':
        lin_dir = -1;
        angle_dir = -1;
        break;
    case '2':
        lin_dir = -1;
        angle_dir = 0;
        break;
    case '3':
        lin_dir = -1;
        angle_dir = 1;
        break;
    case '*':
        vel *= 1.1;
        twist_vel *= 1.1;
        break;
    case '/':
        vel *= 0.9;
        twist_vel *= 0.9;
        break;
    case '+':
        vel *= 1.1;
        break;
    case '-':
        vel *= 0.9;
        break;
    case 'a':
        twist_vel *= 1.1;
        break;
    case 'z':
        twist_vel *= 0.9;
        break;
    case 'R':
    case 'r':
        vel = 0.5;
        twist_vel = 1;
        break;
    case 'e':
        vel = 0.5;
        break;
    case 'w':
        twist_vel = 1;
        break;

    // CTRL+C
    case '\x03':

        // exit the node after have stopped the robot
        robot_vel.angular.z = 0;
        robot_vel.linear.x = 0;
        pub_vel.publish(robot_vel);
        system("clear");
        ros::shutdown();
        exit(0);
        break;
    default:
        // stop robot
        lin_dir = 0;
        angle_dir = 0;
        break;
    }
   
    return;
}

/*
Function that acts as UI of the current node, it is used to show the possible commands
*/
void TeleopCommands()
{
    std::cout << BOLD << ITALIC << "Welcome to the user interface, you can let the robot move based on these different behaviours:\n"
              << NC;
    std::cout << GREEN << menuTeleop << NC;
	
    // Updating and showing the changed value of speed
    printf("\nUpdated speed: @[%.2f,%.2f]\n", vel, twist_vel);
    
    char user_input;
    user_input = GetChar();

    // Call the function to decide what to do depending on which key the user has pressed
    UpdateVelocity(user_input);

    // Calculate new values of speed if the user increment/decrement linear/angular velocity
    robot_vel.linear.x = vel * lin_dir;
    robot_vel.angular.z = twist_vel * angle_dir;

    // Publish new value of speed
    pub_vel.publish(robot_vel);
    
    system("clear");
}

int main(int argc, char **argv)
{
    system("clear");
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    ros::init(argc, argv, "userDriveNotAssisted");
    ros::NodeHandle nh;


    // Advertise, the node publishes updates to the topic /cmd_vel
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    while (true)
        TeleopCommands();

    return 0;
}
