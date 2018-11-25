import socket
from proto import grSim_RobotStatus_pb2
import rospy
from consai_msgs.msg import robot_status

class StatusReceiver(object):
    def __init__(self, ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP
        #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        friend_color = rospy.get_param('friend_color', 'blue')
        port = 30011
        if friend_color == 'yellow':
            port = 30012
        self.sock.bind(("127.0.0.1", port))
        self.status = grSim_RobotStatus_pb2.grSim_Robot_Status()
        self.msg = robot_status()
        self.pub = rospy.Publisher('~robot_status', robot_status, queue_size=10)

    def run(self, ):
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(2048)
            self.status.ParseFromString(data)
            self.msg.header.stamp = rospy.Time.from_sec(self.status.timestamp/1000.0)
            self.msg.id = self.status.id
            self.msg.isteamyellow = self.status.isteamyellow
            self.msg.robottype = self.status.robottype
            self.msg.touchingball = self.status.tou
            self.msg.powerison = self.status.powerison
            rospy.logdebug(status)
            self.pub.publish(self.msg)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('status_receiver')
    self = StatusReceiver()
    self.run()

