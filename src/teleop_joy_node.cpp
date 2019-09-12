// code based on the turtle teleop tutorial available at:
// http://wiki.ros.org/joy/Tutorials/WritingTeleopNode
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

class Teleop
{
	public:
		Teleop();
	private:
		void CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy);
		
		ros::NodeHandle nh;
		ros::Publisher velocityPublisher;
		ros::Subscriber joySubscriber;
		int operate_mode;
		float angular_divider;
		std::string joy_type;
};

Teleop::Teleop()
{
	this->velocityPublisher = this->nh.advertise<geometry_msgs::Twist>("cmd_joy", 10);
	this->joySubscriber = this->nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::CallbackJoy, this);
	this->operate_mode = 0;
	this->angular_divider = 1.f;
}

void Teleop::CallbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{

	geometry_msgs::Twist velocities;
	if(joy->buttons[0]){
		operate_mode = 0;
		ROS_DEBUG("Operation mode 0 selected");
	}
	if(joy->buttons[1]){
		operate_mode = 1;
		ROS_DEBUG("Operation mode 1 selected");
	}
	if(joy->buttons[2]){
		angular_divider = 1.f;
		ROS_DEBUG("Angular velocity is full speed");
	}
	if(joy->buttons[3]){
		angular_divider = 2.f;
		ROS_DEBUG("Angular velocity is half speed");
	}

	switch(this->operate_mode)
	{
		case 0:
		{
			// Button 5 (Xbox RB) is turbo: <-2; 2>
			// Button 4 (Xbox LB) is slowmo: <-0.5; 0.5>
			// combination of both modifiers makes slow, but a bit faster: <-0.(6); 0.(6)>
			float divider = (float)(2-joy->buttons[5]+(joy->buttons[4]*2));

			float forward_vel = (float)(-joy->axes[5]+1)/divider;		// move forward <0;1> Xbox RT (axis 5)
			float backward_vel = (float)(-joy->axes[2]+1)/divider;		// move backward <0;1> Xbox LT (axis 2)
			float linear_velocity = forward_vel - backward_vel;			// linear velocity combined <-1;1>

			velocities.linear.x=linear_velocity;
			velocities.angular.z=joy->axes[0]/this->angular_divider;	// turn - Xbox Left/Right Axis stick left <1;-1> (axis 0)
			break;
		}
		case 1:
		{
			velocities.linear.x=joy->axes[4];							// linear - Xbox Up/Down Axis stick right <-1; 1> (axis 4)
			velocities.angular.z=joy->axes[0]/this->angular_divider;	// turn - Xbox Left/Right Axis stick left <-1; 1> (axis 0)
			break;
		}
		default:
		{
			velocities.linear.x=0;
			velocities.angular.z=0;
			ROS_ERROR("telop_joy_node: operate mode not known, falling back to mode 0. If error repeats try restarting node");
			break;
		}
	}
	//ROS_INFO("linear: %f, angular: %f",velocities.linear.x ,velocities.angular.z);
	this->velocityPublisher.publish(velocities);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy_node");
	Teleop teleop;
	ros::spin();
}
