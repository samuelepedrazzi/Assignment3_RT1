#include "ros/ros.h"
#include "stdio.h"
#include "string.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include "time.h"

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN  "\033[1;32m"
#define RED "\033[1;31m"
#define BLUE "\033[1;34m"

// Initializing the publishers for the goal and for cancelling the goal
ros::Publisher pub_goal;
ros::Publisher pub_canc_goal;

// Initialize subsrcibers
ros::Subscriber sub_feedback;

// Variable for the goal
// The messages needed to communicate with the move_base node are contained in the package move_base_msgs.
// The move_base package implements an action that, given a goal in the world, try to achieve it.
move_base_msgs::MoveBaseActionGoal msg_goal;

// Create a variable to read the goal's fields and cancel it.
actionlib_msgs::GoalID msg_canc;

// Define a boolean if there is a pending goal or not
bool pending_goal = false;

// Define the position coordinates of the actual goal
float x_goal;
float y_goal;

// Define a variable to save the goal id
int goal_id;

void GetFeedback(const actionlib_msgs::GoalStatusArray::ConstPtr& fdb_msg);

/*
Function that simply show the actual goal coordinates.
*/
void DisplayMenu()
{	
	if(pending_goal)
	{
		std::cout << BLUE << "Actual goal is: [" << x_goal << "], [" << y_goal << "].\n"<< NC;
	}
	else
	{
		std::cout << RED << "There is no goal set.\n" << NC;
	}
}

/*
Function that receive the coordinates from the user and set the goal with an id.
Then it publish the goal and subscribe to the goal status.
*/
void SetTargetCoordinates()
{	
	
	system("clear");
	
	std::cout << BOLD << ITALIC << "Welcome to the interface dedicated to allowing you to reach a certain position.\n" << NC;
	
	std::string string_to_convert;
	
        std::cout << BOLD << ITALIC << "It's time to set the coordinates to reach. \n\n" << NC;
        std::cout << BLUE << BOLD << "Insert x coordinate: \n\n" << NC;
        std::cin >> string_to_convert;
        x_goal = atof(string_to_convert.c_str() );
        
        system("clear");
        std::cout << BLUE << BOLD << "Now insert y coordinate: \n\n" << NC;
        std::cin >> string_to_convert;
        y_goal = atof(string_to_convert.c_str() );
	
	msg_goal.goal.target_pose.header.frame_id = "map";
	msg_goal.goal.target_pose.pose.orientation.w = 1;
		    
	msg_goal.goal.target_pose.pose.position.x = x_goal;
	msg_goal.goal.target_pose.pose.position.y = y_goal;
	
	// Update the flag to true because there is a new goal
	pending_goal = true;
	
	// Assign an id to the new goal set
	goal_id = rand();
	msg_goal.goal_id.id = std::to_string(goal_id);
	
	
	system("clear");
	std::cout << GREEN << "A new goal has just been set.\n" << NC;
	
	// Publish new goal 
	pub_goal.publish(msg_goal);
	
	DisplayMenu();
	
	//subscribe to goal status
        ros::NodeHandle nh;
        sub_feedback = nh.subscribe("/move_base/status", 1, GetFeedback);  
}

/*
Function that returns false if there isn't a goal to cancel
otherwise, it cancels the goal by its id and returns true.
*/
bool CancelGoal()
{
	//if there is not any pending goal, do nothing, otherwise delete the goal.
	if(!pending_goal)
		return false;
	msg_canc.id = std::to_string(goal_id); 
	pub_canc_goal.publish(msg_canc);
	pending_goal = false;
	
	sub_feedback.shutdown();
	
	return true;
}

/*
Function that checks the returning status from the ros message GoalStatusArray and
select the correct action and feedback to take back to the user.
*/
void GetFeedback(const actionlib_msgs::GoalStatusArray::ConstPtr& fdb_msg)
{	
	// Define a variable to save the goal's status
	int result = 0;
	
	//look for goals with correct id
        if (std::to_string(goal_id) == fdb_msg->status_list[0].goal_id.id) 
        	result = fdb_msg->status_list[0].status;
        	
        // switch statement to select the correct status code feedback 
        // checking the messages published into /move_base/status 
    	switch(result)
    	{
    	
    	//status SUCCEDED
    	case 3:
        	std::cout << BOLD << GREEN << "Goal achieved!\n" << NC;
        	CancelGoal();
        	break;
        
        //status ABORTED
    	case 4:
        	std::cout << BOLD << RED << "Goal aborted, it cannot be reached!\n" << NC;
        	CancelGoal();
        	break;
        	
        //status REJECTED
        case 5:
        	std::cout << BOLD << RED << "Goal rejected!\n" << NC;
        	CancelGoal();
        	break;
        	
        //status LOST
    	case 9:
        	std::cout << BOLD << RED << "Goal lost!\n" << NC;
        	CancelGoal();
        	break;

        	
        //Any other status
    	default:
        	break;
    	}
	
	// if there is no pending goal active, it is possible to set another goal.
	if(!pending_goal)
	{
		char c;
		std::cout << BOLD << ITALIC << "Would you want to set another goal? y/n\n\n" << NC;
   		std::cin >> c;
		
		if(c == 'y' || c == 'Y') 
		SetTargetCoordinates();
		
		else if(c == 'n' || c == 'N')
		{
		system("clear");
        	CancelGoal();
        	ros::shutdown();
        	exit(0);
		}
	}
	
}


int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "achieveGoalPosition");
    ros::NodeHandle nh;

    // the goal position is published in the "/move_base/goal" topic
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    
    // the message to cancel the goal is published on the "/move_base/cancel" topic
    pub_canc_goal = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
   

    // Get the coordinates to try reach
    SetTargetCoordinates();
    ros::spin();

    return 0;
}

