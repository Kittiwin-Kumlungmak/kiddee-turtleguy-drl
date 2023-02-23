import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from collections import deque
from common import utilities as util
from geometry_msgs.msg import Pose
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('hut_node')
    
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(depth=1)
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', QoSProfile(depth=10))
        self.MIN_REQUIRED = 0.5
        self.MAX_LIDAR_RANGE = 2.0
        self.MIN_ODOM_STACK_LEN = 30
        self.odom_stack = deque(maxlen=self.MIN_ODOM_STACK_LEN)
        self.odom_topic = '/odom'
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.timer_period = 0.1 # seconds
        self.i = 0
        self.start = True
        self.goal = [3.65, 3.65] #[-3.56,-3.47]
        self.num_msg = None

    def scan_callback(self, msg):
        #print(msg.ranges,'\n','----------------------')
        #num_msg = msg.ranges
        self.num_msg = [min(range, self.MAX_LIDAR_RANGE) for range in msg.ranges]

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)

        # Moodang
        #print('odom_st: ', len(self.odom_stack))
        self.stack_odom(self.robot_x,self.robot_y)
        #time.sleep(self.timer_period)

        
        if not self.start:
            if len(self.odom_stack) >= self.MIN_ODOM_STACK_LEN:
                if not self.near_goal():
                    if self.check_loop(self.odom_stack, self.MIN_REQUIRED):
                        best_goal = self.find_best_subgoal()
                        goal_pose = Pose()
                        goal_pose.position.x = best_goal[0]
                        goal_pose.position.y = best_goal[1]
                        self.goal_pose_pub.publish(goal_pose)
                        time.sleep(0.1)
                        print('I am in a loop')
                    else:
                        goal_pose = Pose()
                        goal_pose.position.x = self.goal[0]
                        goal_pose.position.y = self.goal[1]
                        self.goal_pose_pub.publish(goal_pose)
                        time.sleep(0.1)
                        print('I am going to goal')
                else:
                    goal_pose = Pose()
                    goal_pose.position.x = self.goal[0]
                    goal_pose.position.y = self.goal[1]
                    self.goal_pose_pub.publish(goal_pose)
                    time.sleep(0.1)
                    print('I am near goal')
            else:
                print('my odom stack is not full yet')
                goal_pose = Pose()
                goal_pose.position.x = self.goal[0]
                goal_pose.position.y = self.goal[1]
                self.goal_pose_pub.publish(goal_pose)
                time.sleep(0.1)
        else:
            print('I am at the starting point')
            goal_pose = Pose()
            goal_pose.position.x = self.goal[0]
            goal_pose.position.y = self.goal[1]
            self.goal_pose_pub.publish(goal_pose)
            time.sleep(0.1)
            self.start = False


#--------------------------------------------------------------------------------------------------------------------------------------

    def subgoals(self, r, xbot, ybot):
        d={}
        subgoals = []
        #xbot = 0
        #ybot = 0

        sector = 0
        theta = 0
        while (theta <= 359-1): #359 เส้น คือเส้นที่ 0-358
            #if (r[theta+1] - r[theta])/max(r[theta+1], r[theta]) >= 0: #กี่เท่าถึงจะพอ ต้องลองดู
            #if r[theta+1] - r[theta] >= 0.03: #ระยะมากพอ
            if self.distance(r[theta], theta, r[theta+1], theta+1) >= 0.5:   #vector1 - vector2
                #print("firstangle :", theta)
                #print("delta :", distance(r[theta], theta, r[theta+1], theta+1))
                sector = 1
                while (sector <= 358 - theta):                          # เส้นตรงกลางระหว่างช่องจะยาวกว่า เราจะทำจนกว่ามันจะใกล้กันเกิน 0.3
                    #print("firstangle :", sector)
                    if (r[theta+sector] - r[theta] >= 0.3):
                    #if (distance(r[theta+sector],theta+sector,r[theta],theta) >= 0) :
                        sector += 1
                        #print("sector :", sector)
                    else:
                        d[theta] = sector
                        theta += sector
                        sector = 0
                        break

                #print("theta :", theta)
                #print("r[theta] :", r[theta])
                #print("test2")
            theta += 1
        #print("d=",d)
        #return d
        
        for key in d:
            if self.distance(r[key],key,r[key+d[key]],key+d[key]) >= 0.3: #?????
                subgoals.append(self.find_subgoal(r[key],key,r[key+d[key]],key+d[key],xbot,ybot))
                #print(key)
                #print(xbot)
                #print("subgoal = ",find_subgoal(r[key],key,r[key+d[key]],key+d[key],xbot,ybot))
        return subgoals



    def find_subgoal(self, r1, theta1, r2, theta2, xbot, ybot):
        v1 = [r1*math.cos(theta1), r2*math.sin(theta1)]
        v2 = [r2*math.cos(theta2), r2*math.sin(theta2)]
        Rx = 0.5*(v2[0] + v1[0]) + xbot
        Ry = 0.5*(v2[1] + v1[1]) + ybot
        #print(xbot)
        return [Rx, Ry]

    def distance(self, r1, theta1, r2, theta2):
        x1 = r1*math.cos(math.radians(theta1))
        x2 = r2*math.cos(math.radians(theta2))
        y1 = r1*math.sin(math.radians(theta1))
        y2 = r2*math.sin(math.radians(theta2))
        return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )


    def distance2(self, l1,l2) :
        return math.sqrt((l1[0]-l2[0])**2 + (l1[1]-l2[1])**2)

    def select_subgoal(self, goal_pos, l_of_subgoal) :         
        min_distance = 200
        result =[]
        if l_of_subgoal == []:
            return goal_pos
        for i in l_of_subgoal :
            if(self.distance2(goal_pos,i) < min_distance) :
                min_distance = self.distance2(goal_pos,i)
                result = i
        return result
    
#-------------------------------------------------------------------------------------------

    def stack_odom(self, xbot, ybot):
        self.odom_stack.append((xbot, ybot))

    def find_best_subgoal(self) :
        range = self.num_msg
        xbot = self.robot_x
        ybot = self.robot_y
        goal_pos = self.goal
        all_sub = self.subgoals(range, xbot, ybot)
        return self.select_subgoal(goal_pos,all_sub)
    

    def find_distance(self, pose1, pose2) : #x,y is tuple
        # a,b = x
        # c,d = y
        # result = math.sqrt((pose1[0]-c)**2 + (b-d)**2)
        result = math.dist(pose1, pose2)
        return result


    def find_max_distance(self, l) :
        # l --> list of pairs of odom
        result = 0
        for i in l :
            for j in l :
                if(self.find_distance(i,j) > result) :
                    result = self.find_distance(i,j)
        return result


    def check_loop(self, l, min_require) :
        if self.find_max_distance(l) > min_require:
            return 0   #not loop
        return 1       #loop
        
#-----------------------------------------------------------------------------------------------------

    def near_goal(self):
        if math.dist((self.robot_x,self.robot_y), (self.goal[0],self.goal[1])) <= 0.5:
            return 1
        else :
            return 0
    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)



    
    minimal_publisher.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()


