import math,random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from autoware_planning_msgs.srv import SetRoute
from geometry_msgs.msg import PoseStamped

from autoware_auto_planning_msgs.msg import PathWithLaneId,PathPoint,Trajectory,TrajectoryPoint
from geometry_msgs.msg import Point

class TrajectoryShiftAvoidance(Node):
    def __init__(self):
        super().__init__('simple_avoidance')
        self.subscription = self.create_subscription(
            PathWithLaneId,
            'input',
            self.onTrigger,
            1)
        self.objects_subscription = self.create_subscription(
            Float64MultiArray,
            '/aichallenge/objects',
            self.onObjects,
            1)
        self.objects=[]
        self.left_bound = []
        self.right_bound = []
        self.first_round_flag = 0

        self.publisher_ = self.create_publisher(Trajectory, 'output', 1)


        self.declare_parameter('minimum_objects_point_range', 8.0)
        self.minimum_objects_point_range = self.get_parameter('minimum_objects_point_range').get_parameter_value().double_value
        
        self.declare_parameter('start_avoid_lim', -7)
        self.start_avoid_lim = self.get_parameter('start_avoid_lim').get_parameter_value().integer_value
        
        self.declare_parameter('end_avoid_lim', 6)
        self.end_avoid_lim = self.get_parameter('end_avoid_lim').get_parameter_value().integer_value

        ## vector weights  = 1/{(alpha * index + beta)}

        self.declare_parameter('shift_alpha', 0.5)
        self.shift_alpha = self.get_parameter('shift_alpha').get_parameter_value().double_value

        self.declare_parameter('shift_beta', 1.8)
        self.shift_beta = self.get_parameter('shift_beta').get_parameter_value().double_value

    # Recive Objects Topic
    def onObjects(self, msg:Float64MultiArray):
        self.objects = []        
        objects_num = int(len(msg.data) / 4.0)
        for n in range(objects_num):
            object_x = msg.data[n*4 + 0]
            object_y = msg.data[n*4 + 1]
            object_z = msg.data[n*4 + 2]
            object_radius = msg.data[n*4 + 2]
            self.objects.append([object_x,
                                 object_y,
                                 object_z,
                                 object_radius])

    def find_nearest_point_in_boundary(self,trajectory_point: PathPoint,boundary, gain):
        distance_min = 1e+3
        point_min = 1
        for i,data in enumerate(boundary):
            dist = (trajectory_point.pose.position.x - data.x)**2 + (trajectory_point.pose.position.y - data.y)**2
            if(dist<distance_min):
                distance_min =dist
                point_min = i
        trajectory_point.pose.position.x -= (trajectory_point.pose.position.x - boundary[point_min].x)/(self.shift_beta + gain)
        trajectory_point.pose.position.y -= (trajectory_point.pose.position.y - boundary[point_min].y)/(self.shift_beta + gain)

    def onTrigger(self, msg:PathWithLaneId):
        trajectory = Trajectory()
        trajectory.header = msg.header

        # get boundary at start
        if(len(self.left_bound) == 0):
            self.left_bound = msg.left_bound 
            self.get_logger().info("left_bount_len: %s" % len(msg.left_bound))
           
        if(len(self.right_bound) == 0):
            self.right_bound = msg.right_bound
            self.get_logger().info("right_bount_len: %s" % len(msg.right_bound))
        
        # append boundary of goal -> start point, which is not showed at firts time , may be need fix
        if( (len(msg.left_bound) == 2) & (self.first_round_flag == 0 ) ):
            self.first_round_flag = 1
        if( (len(msg.left_bound) != 2) & (self.first_round_flag == 1 ) ):
            self.first_round_flag = 2
            self.left_bound += msg.left_bound[2:15]
            self.right_bound += msg.right_bound[2:15]
            self.get_logger().info("route merged")
        
        # transfer PathWithLaneId() to TrajectoryPoint()
        for data in msg.points:
            trajectory_point = TrajectoryPoint()
            trajectory_point.pose = data.point.pose
            trajectory_point.longitudinal_velocity_mps = data.point.longitudinal_velocity_mps
            trajectory.points.append(trajectory_point)

        # load objects and update TrajectoryPoint() to avoid
        for i in range(len(self.objects)):
            object_x = self.objects[i][0]
            object_y = self.objects[i][1]

            # find nearest point in TrajectoryPoint() of object
            distance_min = 1e+3
            point_min = 0
            for i, point in enumerate(trajectory.points):                
                dist_temp = (point.pose.position.x - object_x)**2 + (point.pose.position.y - object_y)**2
                if(dist_temp < distance_min):
                    distance_min = dist_temp
                    point_min = i

            if(distance_min > self.minimum_objects_point_range):
                continue

            if(point_min > (len(trajectory.points)-2)):
                point_min = len(trajectory.points) - 2

            # check object is at right or left of TrajectoryPoint()
            if(point_min < 2):
                point_min = 2 # fix out of range problem

            vx1 = trajectory.points[point_min].pose.position.x - trajectory.points[point_min-2].pose.position.x
            vy1 = trajectory.points[point_min].pose.position.y - trajectory.points[point_min-2].pose.position.y
            vx2 = object_x - trajectory.points[point_min-2].pose.position.x
            vy2 = object_y - trajectory.points[point_min-2].pose.position.y  
            left_right_flag = vx1 * vy2 - vy1 * vx2  # if <0, objects is at right.

            # <0 is right, then aviod object by driving at left side
            if( (self.start_avoid_lim + point_min) < 0):
                start_avoid_ = 0 - point_min # fix out of range problem
            else:
                start_avoid_ = self.start_avoid_lim

            if( (self.end_avoid_lim + point_min+1) > len(trajectory.points)):
                end_avoid_ = len(trajectory.points) - point_min - 1 # fix out of range problem
            else:
                end_avoid_ = self.end_avoid_lim

            # Shift trajectory points
            if (left_right_flag < 0 ):
                for i in range(start_avoid_, end_avoid_):
                    self.find_nearest_point_in_boundary(trajectory.points[point_min+i],self.left_bound, abs(i) * self.shift_alpha)
            else:
                for i in range(start_avoid_, end_avoid_):
                    self.find_nearest_point_in_boundary(trajectory.points[point_min+i],self.right_bound, abs(i) * self.shift_alpha)

        self.publisher_.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node_ = TrajectoryShiftAvoidance()
    rclpy.spin(node_)

if __name__ == '__main__':
    main()