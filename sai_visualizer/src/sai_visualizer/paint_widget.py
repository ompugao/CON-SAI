#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import tf
import math
import types
import functools

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QPointF, QRectF
from python_qt_binding import QT_BINDING_VERSION
from python_qt_binding.QtGui import QPainter, QPen, QColor
from python_qt_binding.QtGui import QMouseEvent
g_PYQT_MAJOR_VERSION = int(QT_BINDING_VERSION.split('.')[0])
if g_PYQT_MAJOR_VERSION == 4:
    from python_qt_binding.QtGui import QWidget
elif g_PYQT_MAJOR_VERSION == 5:
    from python_qt_binding.QtWidgets import QWidget

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16MultiArray as UIntArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Point
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc
from consai_msgs.msg import ReplaceBall, ReplaceRobot

from geometry import Geometry


# monkey patch
def mouseevent_wrapper(func):
    if g_PYQT_MAJOR_VERSION == 5:
        @functools.wraps(func)
        def wrapper(self, event):
            def _posF(self):
                return self.localPos()
            event.posF = types.MethodType(_posF, event)
            return func(self, event)
        return wrapper
    else:
        return func


def wheelevent_wrapper(func):
    if g_PYQT_MAJOR_VERSION == 5:
        @functools.wraps(func)
        def wrapper(self, event):
            def _delta(self):
                # in Qt4, eveet.delta() returns int value
                # in Qt5, event.angleDelta() returns QPoint value
                return self.angleDelta().y()

            event.delta = types.MethodType(_delta, event)
            return func(self, event)
        return wrapper
    else:
        return func


class PaintWidget(QWidget):
    def __init__(self,parent=None):
        super(PaintWidget,self).__init__(parent)

        # geometry parameters
        self.geometry = Geometry()
        self.scaleOnField   = 1.0
        self.world_height   = 0.0
        self.world_width    = 0.0
        self.trans      = QPointF(0.0,0.0) # 慢性的なトランス
        self.mouseTrans = QPointF(0.0, 0.0) # マウス操作で発生する一時的なトランス
        self.scale      = QPointF(1.0,1.0)
        self.clickPoint = QPointF(0.0,0.0)
        self._current_mouse_pos = QPointF(0.0,0.0)

        # Colors
        self.friendDrawColor = Qt.cyan
        self.enemyDrawColor = Qt.yellow
        self.friend_color = rospy.get_param("friend_color", "blue")
        if self.friend_color != "blue":
            self.friendDrawColor = Qt.yellow
            self.enemyDrawColor = Qt.cyan
        self.targetPosDrawColor = QColor(102, 0, 255, 100)

        # Replace
        self._CLICK_POS_THRESHOLD = 0.1
        self._CLICK_VEL_ANGLE_THRESHOLD = self._CLICK_POS_THRESHOLD + 0.2
        self._replace_ball = ReplaceBall()
        self._replace_pos = QPointF()
        self._replace_vel = QPointF()
        self._replace_angle = 0.0
        self._replace_func = None

        # Status
        self._should_rotate_world = False
        self._can_replace = False
        self._is_ballpos_replacement = False
        self._is_ballvel_replacement = False
        self._is_robotpos_replacement = False
        self._is_robotangle_replacement = False


        # Configs
        # This function enables mouse tracking without pressing mouse button
        self.setMouseTracking(True)

        # Subscribers
        self.field_geometry = GeometryFieldSize()
        self.sub_geometry = rospy.Subscriber("geometry_field_size",
                GeometryFieldSize, self.callbackGeometry)

        self.ballOdom = Odometry()
        self.sub_ballPosition = rospy.Subscriber("ball_observer/estimation", 
                Odometry,self.callbackBallOdom)

        self.friendsIDArray = UIntArray()
        self.sub_friendsID = rospy.Subscriber("existing_friends_id", 
                UIntArray, self.callbackFriendsID)

        self.friendOdoms = [Odometry()] * 12
        self.sub_friendOdoms = []

        self.enemyIDArray = UIntArray()
        self.sub_enemiesID = rospy.Subscriber("existing_enemies_id",
                UIntArray, self.callbackEnemiesID)

        self.enemyOdoms = [Odometry()] * 12
        self.sub_enemyOdoms = []

        self.targetPositions = [PoseStamped()] * 12
        self.sub_targetPositions =[]
        self.targetVelocities = [TwistStamped()] * 12
        self.sub_targetVelocities = []
        self.targetIsPosition = [False] * 12

        self.avoidingPoints = [Point()] * 12
        self.sub_avoidingPoints = []

        for i in xrange(12):
            strID = str(i)
            topicFriend = "robot_" + strID + "/odom"
            topicEnemy = "enemy_" + strID + "/odom"
            topicPosition = "robot_" + strID + "/move_base_simple/goal"
            topicVelocity = "robot_" + strID + "/move_base_simple/target_velocity"
            topicAvoidingPoint = "robot_" + strID + "/avoiding_point"

            self.sub_friendOdoms.append(
                    rospy.Subscriber(topicFriend, Odometry, 
                        self.callbackFriendOdom, callback_args=i))

            self.sub_enemyOdoms.append(
                    rospy.Subscriber(topicEnemy, Odometry,
                        self.callbackEnemiesOdom, callback_args=i))

            self.sub_targetPositions.append(
                    rospy.Subscriber(topicPosition, PoseStamped,
                        self.callbackTargetPosition, callback_args=i))

            self.sub_targetVelocities.append(
                    rospy.Subscriber(topicVelocity, TwistStamped,
                        self.callbackTargetVelocity, callback_args=i))

            self.sub_avoidingPoints.append(
                    rospy.Subscriber(topicAvoidingPoint, Point,
                        self.callbackAvoidingPoint, callback_args=i))

        # Publishers
        self._pub_replace_ball = rospy.Publisher(
                'replacement_ball', ReplaceBall, queue_size=10)


    def callbackGeometry(self, msg):
        self.field_geometry = msg
        self.geometry.set_field(
                self.field_geometry.field_length, 
                self.field_geometry.field_width)


    def callbackBallOdom(self, msg):
        self.ballOdom = msg
        # self.update()


    def callbackFriendsID(self, msg):
        self.friendsIDArray = msg
        # self.update()


    def callbackFriendOdom(self, msg, robot_id):
        self.friendOdoms[robot_id] = msg


    def callbackEnemiesID(self, msg):
        self.enemyIDArray = msg


    def callbackEnemiesOdom(self, msg, robot_id):
        self.enemyOdoms[robot_id] = msg


    def callbackTargetPosition(self, msg, robot_id):
        self.targetPositions[robot_id] = msg
        self.targetIsPosition[robot_id] = True


    def callbackTargetVelocity(self, msg, robot_id):
        self.targetVelocities[robot_id] = msg
        self.targetIsPosition[robot_id] = False


    def callbackAvoidingPoint(self, msg, robot_id):
        self.avoidingPoints[robot_id] = msg


    @mouseevent_wrapper
    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            self.clickPoint = event.posF()

            self._can_replace = self._isReplacementClick(self.clickPoint)

        elif event.buttons() == Qt.RightButton:
            self.resetPainterState()

        self.update()


    @mouseevent_wrapper
    def mouseMoveEvent(self, event):
        self._current_mouse_pos = event.posF()

        if self._can_replace :
            pass
        elif event.buttons() == Qt.LeftButton:
                pos = event.posF()
                self.mouseTrans = (pos - self.clickPoint) / self.scale.x()

        self.update()


    @mouseevent_wrapper
    def mouseReleaseEvent(self, event):
        if self._can_replace:
            self._can_replace = False
            self._replace_func(event.posF())

        else:
            self.trans += self.mouseTrans
            self.mouseTrans = QPointF(0.0, 0.0)

        self.update()


    @wheelevent_wrapper
    def wheelEvent(self, event):
        # マウスのホイール操作でスケールを変える
        if event.delta() > 0:
            s = self.scale.x()
            self.scale.setX(s + 0.1)
            self.scale.setY(s + 0.1)
        else:
            s = self.scale.x()
            if s > 0.2 :
                self.scale.setX(s - 0.1)
                self.scale.setY(s - 0.1)

        self.update()


    def resizeEvent(self, event):
        # widgetのサイズ変更によるイベント
        self.resizeDrawWorld()


    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx,cy)

        # これ以降にトランスとスケール操作を持ってくる
        painter.scale(self.scale.x(), self.scale.y())
        painter.translate(self.trans + self.mouseTrans)

        if self._should_rotate_world == True:
            painter.rotate(-90)


        # これ以降に描きたいものを重ねていく
        self.drawField(painter)
        
        self.drawAvoidingPoints(painter)
        self.drawTargets(painter)

        self.drawFriends(painter)
        self.drawEnemis(painter)
        self.drawBallVelocity(painter)
        self.drawBall(painter)
        self.drawCoordinateText(painter)

        if self._is_ballpos_replacement or self._is_robotpos_replacement:
            self.drawPosReplacement(painter)


    def resetPainterState(self):
        self.trans = QPointF(0.0,0.0)
        self.mouseTrans = QPointF(0.0, 0.0)
        self.scale = QPointF(1.0, 1.0)


    def resizeDrawWorld(self):
        # Widgetのサイズに合わせて、描くフィールドのサイズを変える
        # 描画の回転判断もしてくれるすぐれもの

        widgetHeight = float(self.height())
        widgetWidth = float(self.width())
        w_per_h = widgetWidth/widgetHeight

        if w_per_h >= self.geometry.WORLD_W_PER_H:
            # Widgetが横長のとき
            self.world_height = widgetHeight
            self.world_width = widgetHeight * self.geometry.WORLD_W_PER_H
            self._should_rotate_world = False
        elif w_per_h <= self.geometry.WORLD_H_PER_W:
            # Widgetが縦長のとき
            self.world_height = widgetWidth
            self.world_width = widgetWidth * self.geometry.WORLD_W_PER_H
            self._should_rotate_world = True
        else:
            # 描画回転にヒステリシス性をもたせる
            if self._should_rotate_world == True:
                self.world_height = widgetHeight * self.geometry.WORLD_H_PER_W
                self.world_width = widgetHeight
            else:
                self.world_height = widgetWidth * self.geometry.WORLD_H_PER_W
                self.world_width = widgetWidth

        self.scaleOnField = self.world_width / self.geometry.WORLD_WIDTH


    def convertToDrawWorld(self, x, y):
        drawX = x * self.scaleOnField
        drawY = -y * self.scaleOnField
        point = QPointF(drawX, drawY)

        return point


    def convertToRealWorld(self, x, y):
        x /= self.scale.x()
        y /= self.scale.y()

        x -= (self.trans.x() + self.mouseTrans.x())
        y -= (self.trans.y() + self.mouseTrans.y()) 

        x -= self.width() * 0.5 / self.scale.x()
        y -= self.height() * 0.5 / self.scale.y()

        if self._should_rotate_world:
            x, y = -y, x

        real_x = x / self.scaleOnField
        real_y = -y / self.scaleOnField
        point = QPointF(real_x, real_y)

        return point

    
    def _isReplacementClick(self, mouse_pos):
        real_pos = self.convertToRealWorld(mouse_pos.x(), mouse_pos.y())

        is_clicked = False

        if self._isBallPosClicked(real_pos):
            is_clicked = True
            self._is_ballpos_replacement = True
            self._replace_func = self._replaceBallPos

        return is_clicked

        
    def _isBallPosClicked(self, real_pos):
        posX = self.ballOdom.pose.pose.position.x
        posY = self.ballOdom.pose.pose.position.y
        ball_pos = QPointF(posX, posY)

        return self._isClicked(real_pos, ball_pos)


    def _isClicked(self, real_pos1, real_pos2):
        diff = real_pos1 - real_pos2
        if math.hypot(diff.x(), diff.y()) < self._CLICK_POS_THRESHOLD:
            return True

        return False


    def _replaceBallPos(self, mouse_pos):
        real_pos = self.convertToRealWorld(mouse_pos.x(), mouse_pos.y())

        replace = ReplaceBall()
        replace.pos_x = real_pos.x()
        replace.pos_y = real_pos.y()
        self._pub_replace_ball.publish(replace)

        self._is_ballpos_replacement = False


    def drawField(self, painter):
        # draw green surface rectangle
        painter.setPen(Qt.black)
        painter.setBrush(Qt.green)
        
        rx = -self.world_width * 0.5
        ry = -self.world_height * 0.5
        
        rect = QRectF(rx, ry, self.world_width, self.world_height)
        painter.drawRect(rect)

        # draw white lines
        painter.setPen(QPen(Qt.white,2))

        for line in self.field_geometry.field_lines:
            p1 = self.convertToDrawWorld(line.p1_x, line.p1_y)
            p2 = self.convertToDrawWorld(line.p2_x, line.p2_y)
            painter.drawLine(p1, p2)

        for arc in self.field_geometry.field_arcs:
            top_left = self.convertToDrawWorld(
                    arc.center_x - arc.radius, arc.center_y + arc.radius)
            size = arc.radius * 2.0 * self.scaleOnField

            start_angle = math.degrees(arc.a1)
            end_angle = math.degrees(arc.a2)
            span_angle = end_angle - start_angle

            # angle must be 1/16 degrees order
            start_angle *= 16 
            span_angle *= 16
            painter.drawArc(top_left.x(), top_left.y(), size, size, start_angle, span_angle)


    def drawBall(self, painter):
        posX = self.ballOdom.pose.pose.position.x
        posY = self.ballOdom.pose.pose.position.y

        point = self.convertToDrawWorld(posX,posY)
        size = self.geometry.BALL_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(point, size, size)


    def drawBallVelocity(self, painter):
        ballPos = self.ballOdom.pose.pose.position
        ballVel = self.ballOdom.twist.twist.linear

        if math.hypot(ballVel.x, ballVel.y) < 1.0:
            return 

        angleOfSpeed = math.atan2(ballVel.y, ballVel.x)

        paintDist = 10.0

        velPosX = paintDist * math.cos(angleOfSpeed) + ballPos.x
        velPosY = paintDist * math.sin(angleOfSpeed) + ballPos.y

        ballPosPoint = self.convertToDrawWorld(ballPos.x, ballPos.y)
        velPosPoint = self.convertToDrawWorld(velPosX, velPosY)

        painter.setPen(QPen(QColor(102,0,255),2))
        painter.drawLine(ballPosPoint, velPosPoint)


    def drawFriends(self, painter):
        for robot_id in self.friendsIDArray.data:
            self.drawRobot(painter, robot_id, 
                    self.friendOdoms[robot_id], self.friendDrawColor)


    def drawEnemis(self, painter):
        for robot_id in self.enemyIDArray.data:
            self.drawRobot(painter, robot_id,
                    self.enemyOdoms[robot_id], self.enemyDrawColor)


    def drawRobot(self, painter,robot_id, odom, color):
        # draw robot body on its position
        posX = odom.pose.pose.position.x
        posY = odom.pose.pose.position.y

        point = self.convertToDrawWorld(posX, posY)
        size = self.geometry.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(color)
        painter.drawEllipse(point, size, size)

        # draw robot angle on its body
        orientation = odom.pose.pose.orientation
        # euler_from_quaternion does not support geometry_msgs:Quaternion
        euler = tf.transformations.euler_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w))
        # euler <- (roll, pitch, yaw)
        linePosX = self.geometry.ROBOT_RADIUS * math.cos(euler[2])
        linePosY = self.geometry.ROBOT_RADIUS * math.sin(euler[2])
        linePoint = point + self.convertToDrawWorld(linePosX, linePosY)
        painter.drawLine(point, linePoint)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))


    def drawTargets(self, painter):
        for robot_id in self.friendsIDArray.data:
            if self.targetIsPosition[robot_id] == True:
                self.drawTargetPosition(painter, 
                        robot_id, self.targetPositions[robot_id])
            else:
                self.drawTargetVelocity(painter,
                        robot_id, self.targetVelocities[robot_id])


    def drawTargetPosition(self, painter, robot_id, positionStamped):
        posX = positionStamped.pose.position.x
        posY = positionStamped.pose.position.y

        point = self.convertToDrawWorld(posX, posY)
        size = self.geometry.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(self.targetPosDrawColor)
        painter.drawEllipse(point, size, size)

        orientation = positionStamped.pose.orientation
        # euler_from_quaternion does not support geometry_msgs:Quaternion
        euler = tf.transformations.euler_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w))
        # euler <- (roll, pitch, yaw)
        linePosX = self.geometry.ROBOT_RADIUS * math.cos(euler[2])
        linePosY = self.geometry.ROBOT_RADIUS * math.sin(euler[2])
        linePoint = point + self.convertToDrawWorld(linePosX, linePosY)
        painter.drawLine(point, linePoint)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))


    def drawTargetVelocity(self, painter, robot_id, twistStamped):
        odom = self.friendOdoms[robot_id]
        posX = odom.pose.pose.position.x
        posY = odom.pose.pose.position.y

        point = self.convertToDrawWorld(posX, posY)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = -0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)

        velX = twistStamped.twist.linear.x
        velY = twistStamped.twist.linear.y
        velZ = twistStamped.twist.angular.z

        text = "(" + str(velX) + ", " + str(velY) + ", " + str(velZ) + ")"
        painter.setPen(Qt.red)
        painter.drawText(textPoint, text)


    def drawAvoidingPoints(self, painter):
        for robot_id in self.friendsIDArray.data:
            self.drawAvoidingPoint(painter, robot_id, self.avoidingPoints[robot_id])


    def drawAvoidingPoint(self, painter, robot_id, point):
        drawPoint = self.convertToDrawWorld(point.x, point.y)
        size = self.geometry.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(drawPoint, size, size)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = drawPoint + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))

    
    def drawCoordinateText(self, painter):
        mouse = self._current_mouse_pos
        mouse = self.convertToRealWorld(mouse.x(), mouse.y())

        text = "(" + str(round(mouse.x(),2)) + ", " + str(round(mouse.y(),2)) + ")"
        draw_pos = self.convertToDrawWorld(mouse.x(), mouse.y())

        painter.setPen(Qt.black)
        painter.drawText(draw_pos, text)


    def drawPosReplacement(self, painter):
        startPos = self.convertToRealWorld(
                self.clickPoint.x(), self.clickPoint.y())
        currentPos = self.convertToRealWorld(
                self._current_mouse_pos.x(), self._current_mouse_pos.y())

        startPoint = self.convertToDrawWorld(startPos.x(), startPos.y())
        currentPoint = self.convertToDrawWorld(currentPos.x(), currentPos.y())

        painter.setPen(QPen(Qt.red,2))
        painter.drawLine(startPoint, currentPoint)

