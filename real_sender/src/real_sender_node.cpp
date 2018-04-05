#include    <ros/ros.h>
#include    <ros/console.h>
#include    "serializer/serializer.hpp"
#include    <iostream>
#include    <string>
#include    <unistd.h>
#include    <consai_msgs/robot_commands.h>
#include	<sys/types.h>
#include	<sys/socket.h>
#include	<netinet/in.h>
#include    <arpa/inet.h>

class Sender
{
public:
    Sender(){
    }
    ~Sender(){
    	close(mSock);
	}

    void setID(const int id){
        mID_ = id;
        mSock = socket(AF_INET, SOCK_DGRAM, 0);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(12345);

        if(id == 0)         addr.sin_addr.s_addr = inet_addr("192.168.11.10");
        else if (id == 1)   addr.sin_addr.s_addr = inet_addr("192.168.11.11");
        else if (id == 2)   addr.sin_addr.s_addr = inet_addr("192.168.11.12");
        else if (id == 3)   addr.sin_addr.s_addr = inet_addr("192.168.11.13");
        else if (id == 4)   addr.sin_addr.s_addr = inet_addr("192.168.11.14");
        else if (id == 5)   addr.sin_addr.s_addr = inet_addr("192.168.11.15");
        else                addr.sin_addr.s_addr = inet_addr("192.168.11.16");

        ROS_INFO("Set IPaddr");
    }
    void callback(const consai_msgs::robot_commandsConstPtr& msg){
        ROS_INFO("before Send message");

        ScrambleSerializer serializer;

        float   vel_x  = msg->vel_surge,
            vel_y = msg->vel_sway,
            omega     = msg->omega,
            kick_power= (msg->kick_speed_x > 0.0) ? 15 : 0,
            dribble_power   = (msg->dribble_power > 0.0) ? 15 : 0;
        RobotCommand::KickType  kick_type = (msg->kick_speed_z > 0.0) ? RobotCommand::CHIP : RobotCommand::STRAIGHT;

        RobotCommand cmd(mID_, vel_x, vel_y, omega, dribble_power, kick_power, kick_type);

        char data[10];
        serializer.serialize(cmd, data);

        //data[0] = 255;

        //sendto(mSock, "TEST", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
		sendto(mSock, data, 10, 0, (struct sockaddr *)&addr, sizeof(addr));
        ROS_INFO("Send message");
    }
    /*void test_send(void){
        ROS_INFO("test send");
        sendto(mSock, "TEST", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
    }*/
    
private:
	int mSock;
	struct sockaddr_in addr;

    int mID_;
};

int main(int argc, char **argv) {
    int robot_num = 12;
    ros::init(argc, argv, "real_sender");
    ros::NodeHandle nh;
    ros::Rate r(60);

    Sender senders[robot_num];
    ros::Subscriber subscribers[robot_num];

    for(int i=0;i<12;i++){
        std::stringstream topicStream;
        topicStream << "robot_" << i << "/robot_commands";
        std::string topicName = topicStream.str();
        senders[i].setID(i);
        subscribers[i] = nh.subscribe(topicName.c_str(),100,&Sender::callback, &senders[i]);
    }

    while (ros::ok()) {
        //ROS_INFO("Send loop");
        //senders[0].test_send();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
