#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "model1_controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */


  ros::Publisher model1_controller_pub_joint0 = n.advertise<std_msgs::Float64>("stationary_base_spin_base_joint_controller/command", 1000);
  ros::Publisher model1_controller_pub_joint1 = n.advertise<std_msgs::Float64>("spin_base_arm1_joint_controller/command", 1000);
  ros::Publisher model1_controller_pub_joint2 = n.advertise<std_msgs::Float64>("arm1_arm2_joint_controller/command", 1000);
  ros::Publisher model1_controller_pub_joint3 = n.advertise<std_msgs::Float64>("arm2_arm3_joint_controller/command", 1000);
  ros::Publisher model1_controller_pub_gripper1 = n.advertise<std_msgs::Float64>("arm3_gripper1_joint_controller/command", 1000);
  ros::Publisher model1_controller_pub_gripper2 = n.advertise<std_msgs::Float64>("arm3_gripper2_joint_controller/command", 1000);
  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 joint[4];
    std_msgs::Float64 grip[2];
    std::string command;
    std::cout << "q to quit, m followed by each axis rotation (4) to control the robot, g and ug to control the gripper\n" << std::endl;
    std::cin >> command;
    if (command=="q"){
      return 0;
    }
    else if (command=="m"){
      std::cin >> joint[0].data >> joint[1].data >> joint[2].data >> joint[3].data;
      ROS_INFO("%f %f %f %f", joint[0].data,joint[1].data,joint[2].data,joint[3].data);
      model1_controller_pub_joint0.publish(joint[0]);
      model1_controller_pub_joint1.publish(joint[1]);
      model1_controller_pub_joint2.publish(joint[2]);
      model1_controller_pub_joint3.publish(joint[3]);
    }
    else if (command=="g"){
      grip[0].data=-0.2;grip[1].data=-0.2;
      model1_controller_pub_gripper1.publish(grip[0]);
      model1_controller_pub_gripper2.publish(grip[1]);
    }
    else if (command=="ug"){
      grip[0].data=-1.1;grip[1].data=-1.1;
      model1_controller_pub_gripper1.publish(grip[0]);
      model1_controller_pub_gripper2.publish(grip[1]);
    }
    else{
      std::cout << "Wrong command!\n" << std::endl;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}