#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer
    We make this a class rather than a script because it bundles up 
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming
    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene
    """
    def __init__(self, group_name):
        """
        Constructor.
        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """

        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # This publishes updates to the planning scene
        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(5)

        # Set the bounds of the workspace
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def plan_to_pose(self, target, orientation_constraints):
        """
        Generates a plan given an end effector pose subject to orientation constraints
        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages
        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        self._group.set_pose_target(target)
        self._group.set_start_state_to_current_state()

        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self._group.set_path_constraints(constraints)

        plan = self._group.plan()

        return plan

    def execute_plan(self, plan):
        """
        Uses the robot's built-in controllers to execute a plan
        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """

        return self._group.execute(plan, True)


    def add_box_obstacle(self, size, name, pose):
        """
        Adds a rectangular prism obstacle to the planning scene
        Inputs:
        size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
        name: unique name of the obstacle (used for adding and removing)
        pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
        """    

        # Create a CollisionObject, which will be added to the planning scene
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        # Create a box primitive, which will be inside the CollisionObject
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size

        # Fill the collision object with primitive(s)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]

        # Publish the object
        self._planning_scene_publisher.publish(co)

    def remove_obstacle(self, name):
        """
        Removes an obstacle from the planning scene
        Inputs:
        name: unique name of the obstacle
        """

        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name

        self._planning_scene_publisher.publish(co)


def controller(turtlebot_frame, goal_frame):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  K1 = 0.3
  K2 = 1
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(base, ar_marker_13, rospy.Time())

      # Process trans to get your state error
      # Generate a control command to send to the robot

      control_command = Twist()
      control_command.linear.x = trans.transform.translation.x * K1
      control_command.angular.z = trans.transform.translation.y * K2

      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    controller(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
    pass