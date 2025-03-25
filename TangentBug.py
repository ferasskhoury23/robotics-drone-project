import time
from typing import Generator, List, Optional, Dict
import numpy as np
from fontTools.merge.util import first
from DroneClient import *
from Vector2 import *
from quat import *



class TangentBug:

    colision_radius: float = 3 #how far a line can be from a point to count having colided

    connection_distance: float = 9  #how far two points need to be from each other to be considered part of the same obstacle

    goal_epsilon: float = 5 # how far the current position of the drone can be from the goal,for it to count as having reached the goal

    sensor_range: float = 35 # the maximum sensor range

    boundary_distance: float = 4 #the preferred distance the drone should be from the boundary while following it

    corridor_distance: float = 9

    client: DroneClient #the client

    height: float #the z coordinate

    obstacle_points: Dict[Vec2, int] = {}

    nearby_points: List[Vec2] = [] # the obstacle points within the range of the drones sensor, in body frame

    position: Vec2 = Vec2(0, 0) #the current position of the drone in world frame.

    orientation: float = 0 #the current orientation of the drone in world frame.

    orientation3D: Quaternion = Quaternion(0, 0, 0, 1) #the full 3D orientation of the drone,used only for initial conversions to the 2D plane

    goal: Vec2 = Vec2(0, 0) # flying goal

    cur_corridor_width: float = math.inf #the actual width of the corridor the drone is inside of

    ver: int = 0  #the number of ticks until the vertigo from sharp turns subsides, and the drone can move faster.

    boundary_followed: bool = False  # tracks if a boundary was followed

    min_dist: float

    pos: [] # drone route. used for printing

    lidar_data: [] # lidar collected from the sensor used for printing

    current_velocity: float #current drone velocity



    def __init__(self, client: DroneClient, height: float) -> None:
        self.client = client
        self.height = height
        self.pos = []
        self.lidar_data = []

    def conv_to_body_frame(self, point):
        relative_position = point - self.position  # Translate point relative to drone's position
        return relative_position.rotate(-self.orientation)  # Rotate into body frame

    def conv_to_world_frame(self, point):
        rotated_point = point.rotate(self.orientation)  # Rotate into world frame
        return rotated_point + self.position  # Translate back to world position

    def autoFlyTo(self, point: Vec2, limit: float = 10):
        length = point.length()
        if abs(Vec2(1, 0).angle(point)) > math.pi / 6:  # check if there sharp turn
            self.ver = 100
        velocity: float
        if length < 0.0001:
            velocity = 0
        else:
            safety_velocity = min((p.length() for p in self.nearby_points), default=math.inf)
            if self.ver <= 0:
                safety_velocity *= 1.5
            else:
                self.ver -= 1
            velocity = min(limit, safety_velocity, self.goal.length() / 2)
        world_point = self.conv_to_world_frame(point)
        self.current_velocity = velocity
        self.client.flyToPosition(world_point.x, world_point.y, self.height, velocity)


    def detectObstacles(self) -> Generator[Vec2, None, None]:
        # find points around the drone, detected by the drones LIDAR
        point_cloud = self.client.getLidarData().points

        if len(point_cloud) < 3:
            # the cloud is empty, no points where observed
            return

        for i in range(0, len(point_cloud), 3):
            point = Quaternion(
                point_cloud[i], point_cloud[i + 1], point_cloud[i + 2], 0)
            rotated = self.orientation3D * point * self.orientation3D.conjugate()
            world_point = Vec2(rotated.x, rotated.y) + self.position
            self.lidar_data.append([world_point.x, world_point.y])

            yield world_point


    def updateEnvironment(self):
        pose = self.client.getPose()
        self.orientation3D = Quaternion.from_euler_angles(pose.orientation.x_rad,
                                                          pose.orientation.y_rad,
                                                          pose.orientation.z_rad)
        position = Vec2(pose.pos.x_m, pose.pos.y_m)
        self.pos.append([pose.pos.x_m, pose.pos.y_m])
        world_goal = self.conv_to_world_frame(self.goal)
        self.position = position
        self.orientation = pose.orientation.z_rad
        self.goal = self.conv_to_body_frame(world_goal)
        self.cur_corridor_width = self.find_the_tunnel_width()

        for point in self.detectObstacles():
            self.obstacle_points[point.round()] = 0

        #remove old points
        forgotten = []
        for point_2, iterations in self.obstacle_points.items():
            if iterations > 250:
                forgotten.append(point_2)
            else:
                # the point stays for another iteration
                self.obstacle_points[point_2] += 1
        for p in forgotten:
            self.obstacle_points.pop(p, None)

        self.nearby_points = [self.conv_to_body_frame(p) for p in self.obstacle_points.keys()
                              if 1 < p.distance(self.position) < self.sensor_range]



    def findPath(self, goal: Vec2  , inside, originalGoal ):
        flag = [False]
        curTime = [-1]
        self.goal = self.conv_to_body_frame(goal)
        self.updateEnvironment()
        following_boundary = False
        boundary_following_planner = self.followBoundary(flag,curTime,originalGoal)
        self.min_dist = np.inf
        last_direction = self.goal.rotate(-self.orientation).normalize()

        while True:
            self.updateEnvironment()
            if self.goal.length() <= self.goal_epsilon:
                self.autoFlyTo(Vec2(0, 0))
                return
            if following_boundary:
                limit = 9
                point = next(boundary_following_planner, None)
                if point is None:
                    self.min_dist = np.inf
                    following_boundary = False
                else:
                    self.autoFlyTo(point, limit=limit)

            else:

                point = self.go_to_goal()
                if point is None:
                    boundary_following_planner = self.followBoundary(flag,curTime,last_direction,originalGoal)
                    following_boundary = True
                    DEBUG = (time.time() - curTime[0])


                    v = self.current_velocity
                    if (v == 0):
                        v = 3

                    if inside and flag[0] and ( (curTime[0] == -1 )or( (curTime[0]!=-1)and(DEBUG >= (8/v)) ) ) :
                        flag[0] = False
                        curTime[0] = time.time()
                        self.boundary_followed = True
                        print("I FOLLOWW BOUNDARYYYYYYYY")
                        self.updateEnvironment()
                        time.sleep(0.02)
                        self.autoFlyTo(Vec2(0, 0))
                        return

                else:
                    self.autoFlyTo(point, 9)
                    last_direction = point.rotate(-self.orientation).normalize()

            time.sleep(0.02)

    #calculates the next best point according to tangent bug to travel to
    def go_to_goal(self):
        #checks if obstacle is in path
        if any(checkoverlapCircle(Vec2(0, 0), self.goal, p, self.colision_radius) for p in self.nearby_points):
            discontinuity_points = self.findDiscontinuityPoints()
            if discontinuity_points is None:
                return

            the_closest_point_to_the_goal = min(discontinuity_points,key=lambda p: self.heuristic_distance(p))
            distance = self.heuristic_distance(the_closest_point_to_the_goal)
            if self.min_dist < distance:
                return None
            else:
                self.min_dist = distance
                return the_closest_point_to_the_goal

        else:
            return self.goal


    def checkIfPointsConnected(self, p1: Vec2, p2: Vec2):
        return p1.distance(p2) <= self.connection_distance


    def getBlockingObstacle(self, path: Vec2):
        obstacle = []
        counter_clockwise_points = []
        clockwise_points = []

        self.nearby_points.sort(key=lambda p: path.angle(p))

        for point in self.nearby_points:
            if checkoverlapCircle(Vec2(0, 0), path, point, self.colision_radius):
                obstacle.append(point)
            elif path.angle(point) > 0:
                counter_clockwise_points.append(point)
            else:
                clockwise_points.append(point)

        for point in counter_clockwise_points:
            if any(self.checkIfPointsConnected(point, p) for p in obstacle):
                obstacle.append(point)

        for point in reversed(clockwise_points):
            if any(self.checkIfPointsConnected(point, p) for p in obstacle):
                obstacle.append(point)
        return obstacle

    def findDiscontinuityPoints(self):
        obstacle = self.getBlockingObstacle(self.goal)

        cw = min(obstacle, key=lambda p: self.goal.angle(p))
        cw_avoidance_angle = getFoVCoverage(cw, self.boundary_distance)
        if cw_avoidance_angle is None:
            return None
        cw = cw.rotate(-cw_avoidance_angle)

        ccw = max(obstacle, key=lambda p: self.goal.angle(p))
        ccw_avoidance_angle = getFoVCoverage(ccw, self.boundary_distance)
        if ccw_avoidance_angle is None:
            return None
        ccw = ccw.rotate(ccw_avoidance_angle)
        #tmp = Vec2(float('inf'),float('inf'))
        return cw , ccw

    def heuristic_distance(self, point: Vec2):
        return point.length() + point.distance(self.goal)

    def getFollowedBoundary(self, followed_point: Vec2) -> Generator[Vec2, None, None]:
        corridor_ratio = self.cur_corridor_width / self.corridor_distance
        resize = min(1, 0.9 * corridor_ratio)

        for point in self.nearby_points:
            if point.distance(followed_point) < resize * self.corridor_distance:
                yield point

    def followBoundary(self,flag ,curTime,originalGoal,prev_path_hint: Optional[Vec2] = None) -> Generator[Vec2, None, None]:
        self.boundary_followed = True
        min_followed_distance = math.inf
        right_follow = None
        prev_followed_obstacle = [self.conv_to_world_frame(p) for p in self.getBlockingObstacle(self.goal)]
        while True:
            # ensure that the obstacle contains only points that are currently nearby
            followed_obstacle = set(self.conv_to_body_frame(p).round()
                                    for p in prev_followed_obstacle)
            followed_obstacle.intersection_update(p.round()
                                                  for p in self.nearby_points)
            # ensure that obstacles in the way to the followed obstacle are not ignored,
            followed_obstacle.update(
                p for p in self.nearby_points if p.length() < self.boundary_distance * 1.5)

            followed_point = min(followed_obstacle,
                                 key=lambda p: p.length(), default=None)
            if followed_point is None:
                #flag[0] = True
                #curTime[0] = time.time()
                return

            followed_obstacle.update(
                self.getFollowedBoundary(followed_point))
            prev_followed_obstacle = [
                self.conv_to_world_frame(p) for p in followed_obstacle]

            if right_follow is None:
                # helps convince pyright linter that followed point is not None in this branch
                fp = followed_point
                # if no path hint is available, choose the direction based on the path to the goal
                path_hint = prev_path_hint.rotate(
                    self.orientation) if prev_path_hint is not None else originalGoal

                # find the direction to follow that is closest to the path the drone is already going towards
                right_follow = min(
                    [True, False], key=lambda b: abs(self.getNextFollowPoint(fp, b).angle(path_hint)))

            cur_followed_distance = min(p.distance(originalGoal) for p in followed_obstacle)

            min_followed_distance = min(cur_followed_distance, min_followed_distance)

            blocking_point = min((p for p in self.nearby_points if checkoverlapCircle(
                Vec2(0, 0), originalGoal, p, self.colision_radius)),
                                 key=lambda p: p.length(), default=None)
            reachable_distance = max(originalGoal.length() - self.sensor_range, 0) \
                if blocking_point is None else min(p.distance(originalGoal)
                                                   for p in self.getFollowedBoundary(blocking_point))

            if min_followed_distance > reachable_distance:
                # end boundary following behavior, now that the goal is in reach
                flag[0] = True
                curTime[0] = time.time()
                return

            flight_direction = self.getNextFollowPoint(
                followed_point, right_follow)

            yield flight_direction

    def find_the_tunnel_width(self):
        closest_point = min(
            self.nearby_points, key=lambda p: p.length(), default=None)

        if closest_point is None:
            return math.inf

        opposing_distance = min((p.length() for p in self.nearby_points if abs(closest_point.angle(p)) > math.pi / 2), default=math.inf)
        return closest_point.length() + opposing_distance

    def getNextFollowPoint(self, followed_point: Vec2, right_follow: bool) -> Vec2:
        angle_sign = 1 if right_follow else -1
        away_point = Vec2(0, 0)
        max_angle = -math.inf

        for point in self.getFollowedBoundary(followed_point):
            # ensure that the distance from the boundary is small enough,
            # to avoid being closer to the other side of the corridor
            resize = min(1, 0.4 * self.cur_corridor_width)
            radius = min(resize * self.boundary_distance, 0.9999 * point.length())

            # rotate away from the obstacle to avoid colliding with it
            avoidance_angle = getFoVCoverage(point, radius)
            assert avoidance_angle is not None
            rotated = point.rotate(avoidance_angle * angle_sign)

            # find the point that would avoid all other points on the obstacle as well
            angle = angle_sign * followed_point.angle(rotated)
            if max_angle < angle:
                max_angle = angle
                away_point = rotated

        return away_point

        
