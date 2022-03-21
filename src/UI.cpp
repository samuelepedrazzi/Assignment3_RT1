#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <termios.h>
#include <stdlib.h>

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN  "\033[1;32m"
#define RED "\033[1;31m"

// Defining the Empty object reset which is needed to reset the robot position and the first value of the speed
std_srvs::Empty reset;

// Define menu to select which activity the robot has to do
std::string Menu = R"(
___________________________________________________________________

1 - Insert new coordinates to reach

2 - Drive the robot with the keyboard 

3 - Drive the robot with the keyboard with wall collision avoidance
	
4 - Quit the program

5 - Reset the simulation

)";

/*
Function that display the options and, depending on the user input, select the one chosen.
*/
int BehaviourChoice()
{
    system("clear");
    std::cout << BOLD << ITALIC << "Welcome to the user interface, you can let the robot move based on these different behaviours:" << NC ;
    std::cout << GREEN << Menu << NC;
    int option ;
    option = getchar();
    
    return option;
    
}

int main(int argc, char **argv)
{
    // Initialize the node 
    ros::init(argc, argv, "UI");
    
    while (true)
    {

        switch (BehaviourChoice())
        {
        case '1':
            //launch achieveGoalPosition node
            system("rosrun Assignment3_RT1 achieveGoalPosition");
            break;
        case '2':
            //launch userDriveNotAssisted node
            system("rosrun Assignment3_RT1 userDriveNotAssisted");
            break;
            
        case '3':
            //launch userDriveAssisted node
            system("rosrun Assignment3_RT1 userDriveAssisted");
            break;
            
        case '4':
            //exit from main function
            system("clear");
            return 0;
            break;
            
        case '5':
            system("clear");        	
            //call gazebo/reset_simulation service and reset the simulation
            ros::service::call("/gazebo/reset_simulation", reset);
            break;
        default:
            break;
        }
    }

    return 0;
}
