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
#include    <boost/format.hpp>
#include    <boost/algorithm/clamp.hpp>

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

        int ip_suffix = id;
        if (id < 0 || id > 12) {
            ip_suffix = 20;
        }
        ip_suffix += 10;
        std::string ipaddr = (boost::format("192.168.15.%2d")%(ip_suffix)).str();
        addr.sin_addr.s_addr = inet_addr(ipaddr.c_str());

        ROS_INFO("Set Robot(%d) IPaddr to %s", mID_, ipaddr.c_str());
    }
    void callback(const consai_msgs::robot_commandsConstPtr& msg){
        ROS_INFO("before Send message");

        ScrambleSerializer serializer;

        float   vel_x     = msg->vel_surge,
            vel_y         = msg->vel_sway,
            omega         = msg->omega,
            kick_power    = boost::algorithm::clamp(msg->kick_speed_x * 2, 0.0, 15.0), //XXX: this value is halfen inside the kick circuit
            dribble_power = boost::algorithm::clamp(msg->dribble_power / 50, 0.0, 15.0); //XXX: rpm -> [0 - 16]
        RobotCommand::KickType  kick_type = (msg->kick_speed_z > 0.0) ? RobotCommand::CHIP : RobotCommand::STRAIGHT;

        RobotCommand cmd(mID_, vel_x, vel_y, omega, dribble_power, kick_power, kick_type);

        char data[18];
        serializer.serialize(cmd, data);
        if(mID_ == 0) {
	        ROS_INFO("%d",data[2]);
        }

        //for debug
        //if(mID_ == 0)
	    //    ROS_ERROR("%d", data[2]);
        /*if(mID_== 0) sendto(mSock, "ID0", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
        else if(mID_== 1) sendto(mSock, "ID1", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
        else if(mID_== 2) sendto(mSock, "ID2", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
        else if(mID_== 3) sendto(mSock, "ID3", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
        else if(mID_== 4) sendto(mSock, "ID4", 4, 0, (struct sockaddr *)&addr, sizeof(addr));
        else if(mID_== 5) sendto(mSock, "ID5", 4, 0, (struct sockaddr *)&addr, sizeof(addr));*/
		sendto(mSock, data, 18, 0, (struct sockaddr *)&addr, sizeof(addr));
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
