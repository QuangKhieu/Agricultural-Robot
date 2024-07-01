#!/usr/bin/env python3
import rospy
from PIL import Image, ImageOps
import numpy as np
# import matplotlib.pyplot as plt
import json
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

# load img of map

img = Image.open('/home/lab701/catkin_ws/src/my_robot/navigation/src/testmap.png')
img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = - np_img # invert loaded img, because 0 = free, 1 = obj
np_img[np_img > 0] = 1
print(np_img.shape)
# plt.set_cmap('binary')
# plt.imshow(np_img)

# save image
np.save('cspace.npy', np_img)

grid = np.load('cspace.npy')


class treeNode():
  def __init__(self, locationX, locationY):
    self.locationX = locationX
    self.locationY = locationY
    self.children = []
    self.parent = None

class RRTAlgorithm():
  def __init__(self, start, goal, num_iterations, grid, step_size):
    self.randomTree = treeNode(start[0], start[1])
    self.goal = treeNode(goal[0], goal[1])
    self.nearestNode = None
    self.iterations = num_iterations
    self.grid = grid
    self.step = step_size
    self.path_distance = 0
    self.nearest_dist = 10000
    self.num_waypoints = 0
    self.waypoints = []

  def add_child(self, locationX, locationY):
    if self.goal_found(np.array([locationX, locationY])):
      self.nearestNode.children.append(self.goal)
      self.goal.parent = self.nearestNode
    else:
      temp_node = treeNode(locationX, locationY)
      self.nearestNode.children.append(temp_node)
      temp_node.parent = self.nearestNode

  def sample_point(self):
    point = np.random.random(2) * np.array([self.grid.shape[1], self.grid.shape[0]])
    return point

  def steer_point(self, location_start: treeNode, location_end):
    length, u_vect = self.unit_vector(location_start, location_end)
    if 5 <= length <= self.step:
      point = location_end
    else:
      offset = self.step * u_vect
      point = np.array([location_start.locationX + offset[0], location_start.locationY + offset[1]])

    if point[0] >= self.grid.shape[1]:
      point[0] = self.grid.shape[1]
    if point[1] >= self.grid.shape[0]:
      point[1] = self.grid.shape[0]

    return point

  def is_in_obstacle(self, location_start: treeNode, location_end):
    length, u_vect = self.unit_vector(location_start, location_end)
    temp_point = np.array([0.0, 0.0])
    
    for i in range(round(length)):
      temp_point[0] = location_start.locationX + i * u_vect[0]
      temp_point[1] = location_start.locationY + i * u_vect[1]

      if grid[round(temp_point[1]), round(temp_point[0])] == 0:
        return True

    return False

  def unit_vector(self, location_start: treeNode, location_end):
    curr_location = np.array([location_start.locationX, location_start.locationY])
    vect = location_end - curr_location
    length = np.linalg.norm(vect)
    u_vect = vect/length

    return length, u_vect

  def find_nearest(self, root: treeNode, point):
    if not root:
      return
    dist = self.calc_distance(root, point)
    if dist <= self.nearest_dist:
      self.nearestNode = root
      self.nearest_dist = dist

    for child in root.children:
      self.find_nearest(child, point)

  def calc_distance(self, node1, point):
    dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
    return dist

  def goal_found(self, point):
    goal_dist = self.calc_distance(self.goal, point)

    if goal_dist <= self.step:
      return True
    return False

  def reset_nearest_values(self):
    self.nearestNode = None
    self.nearest_dist = 10000

  def retrace_rrt_path(self, goal):
    if goal.locationX == self.randomTree.locationX and goal.locationY == self.randomTree.locationY:
      return

    self.num_waypoints += 1
    current_point = np.array([goal.locationX, goal.locationY])
    self.waypoints.insert(0, current_point)

# fig = plt.figure("RRT Algorithm")
# plt.imshow(255-grid, cmap='binary')
# plt.plot(start[0], start[1], 'ro')
# plt.plot(goal[0], goal[1], 'bo')
# goal_region = plt.Circle((goal[0], goal[1]), near_goal_dist, color='b', fill = False)
# ax = fig.gca()
# ax.add_patch(goal_region)
# plt.xlabel('X-axis $(m)$')
# plt.ylabel('Y-axis $(m)$')

# waypoints = process_rrt(start, goal, num_iterations, step_size, near_goal_dist)
# for i in range(len(waypoints) - 1):
#     plt.plot([waypoints[i][0], waypoints[i+1][0]], [waypoints[i][1], waypoints[i+1][1]], 'ro', linestyle='--')
#     plt.pause(0.10)

# plt.show()


# ========================================= SECOND HALF ==============================================
class PathPlanningProcess:
  def __init__(self, landmark_points, robot_radius, step) -> None:
    self.step = step
    self.num_iterations = 400
    self.near_goal_dist = step / 2
    self.landmark_points = landmark_points
    self.robot_radius = robot_radius
    
    self.move = 1
    self.marker_id = -1
    self.next_waypoint = np.array([0.0, 0.0])
    self.odometry_point = np.array([0.0, 0.0])
    self.queue = []

    self.waypoint_pub = rospy.Publisher("goal_point", PointStamped, queue_size=10)
    # self.stop_sub = message_filters.Subscriber("object_stop", Int32)
    self.marker_sub = message_filters.Subscriber("marker_id", PointStamped)
    self.odometry_sub = message_filters.Subscriber("odometry_pose", PointStamped)
    self.ts = message_filters.TimeSynchronizer([self.marker_sub, self.odometry_sub], 10)
    self.ts.registerCallback(self.sub_callback)

  def sub_callback(self, marker_msg, odometry_msg):
    self.marker_id = marker_msg.point.x
    self.odometry_point = np.array([odometry_msg.point.x, odometry_msg.point.y])
    
    self.process_plan2()
    print(self.next_waypoint)
    self.waypoints_publish()  # trigger publishing

  def find_nearest_landmark(self, point):
    dist = np.zeros(len(self.landmark_points))

    for i in range(len(self.landmark_points)):
      dist[i] = np.sqrt((point[0] - self.landmark_points[i][0])**2 + (point[1] - self.landmark_points[i][1])**2)

    return np.argmin(dist)

  def gen_rrt_waypoints(self, start, goal):
    rrt = RRTAlgorithm(start, goal, self.num_iterations, grid, self.step)

    for i in range(rrt.iterations):
      rrt.reset_nearest_values()

      point = rrt.sample_point()
      rrt.find_nearest(rrt.randomTree, point)
      new = rrt.steer_point(rrt.nearestNode, point)
      
      if rrt.is_in_obstacle(rrt.nearestNode, new):
        continue

      rrt.add_child(new[0], new[1])

      if rrt.goal_found(new):
        rrt.add_child(goal[0], goal[1])
        print("Goal found!")
        print(rrt.goal.locationX, rrt.goal.locationY)
        print(new[0], new[1])
        break

    rrt.retrace_rrt_path(rrt.goal)
    rrt.waypoints.insert(0, start)

    return rrt.waypoints

  def waypoints_publish(self):
    waypoint_msg = PointStamped()
    waypoint_msg.point = Point(self.next_waypoint[0], self.next_waypoint[1], self.move)
    waypoint_msg.header = Header()
    self.waypoint_pub.publish(waypoint_msg)

  def process_plan1(self):
    if self.move == 0:
      return

    if len(self.queue) <= 1:
      current_id = self.marker_id
      if current_id >= len(self.landmark_points):
        print("invalid id")
        self.move = 0
        return

      if current_id < 0:
        current_id = self.find_nearest_landmark(self.odometry_point)

      start = self.odometry_point
      next_marker_id = (current_id + 1) % len(self.landmark_points)
      landmark = landmark_points[next_marker_id]
      goal = landmark[0:2] + (2 + self.robot_radius) * landmark[2:4]

      if len(self.queue) == 1:
        start = self.queue[0]

      waypoints = self.gen_rrt_waypoints(start, goal)
      self.queue = self.queue + waypoints[len(self.queue):]

    dist = np.linalg.norm(self.next_waypoint - self.odometry_point)
    if dist <= self.step:
      self.next_waypoint = self.queue.pop(0)

  def process_plan2(self):
    if self.move == 0:
      return

    if len(self.queue) <= 1:
      current_id = self.marker_id
      if current_id >= len(self.landmark_points):
        print("invalid id")
        return

      if current_id < 0:
        current_id = self.find_nearest_landmark(self.odometry_point)

      start = self.odometry_point
      next_marker_id = (current_id + 1) % len(self.landmark_points)
      landmark = landmark_points[next_marker_id]
      goal = landmark[0:2] + (0.05 + self.robot_radius) * landmark[2:4]

      if len(self.queue) == 1:
        start = self.queue[0]

      waypoints = []
      waypoints.append(start)
      waypoints.append((start + goal)/2)
      waypoints.append(goal)

      self.queue = self.queue + waypoints[len(self.queue):]
      
    dist = np.linalg.norm(self.next_waypoint - self.odometry_point)
    if dist <= self.step:
      self.next_waypoint = self.queue.pop(0)

if __name__=="__main__":
  # read json for map data
  with open("/home/lab701/catkin_ws/src/my_robot/src/localize_cam/map.json", "r") as file:
      map_data = json.load(file)

  pos_list = map_data[1]['pos']
  landmark_points = []
  for pose in pos_list:
    landmark = np.array([pose[0][3], pose[1][3], pose[0][2], pose[1][2]])
    landmark_points.append(landmark)

  robot_radius = 0.15
  step = 0.3

  try:
    rospy.init_node("path_plan")
    pp = PathPlanningProcess(landmark_points, robot_radius, step)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutdown!")
