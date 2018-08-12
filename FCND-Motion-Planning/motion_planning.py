import argparse
import time
import msgpack
import random
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, random_samples, collides, extract_polygons, create_graph, heuristic, a_star_graph, closest_point
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print(self.target_position[2])
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        RANDOM_START = False
        GRAPH_SEARCH = False

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
	        line = f.readline()
        latLonArray = map(lambda x: x.strip(), line.split(','))
        lat0, lon0 = map(lambda x: float(x.split(' ')[1]), latLonArray)
        
        # Set the home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current global position
        current_global_position = np.array([self._longitude, self._latitude, self._altitude])
 
        # Convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
    
        if (RANDOM_START):
            random_start_pos = self.randomPoint(grid, north_offset, east_offset, 0, self.global_home)
            self.set_home_position(random_start_pos[1], random_start_pos[0], 0)
            current_global_position = np.array([self._longitude, self._latitude, self._altitude])
            current_local_position = global_to_local(current_global_position, self.global_home)
    
        # Convert start position to current position
        grid_start = (int(current_local_position[0] - north_offset), int(current_local_position[1]) - east_offset)
        
        # Random Start
        if (RANDOM_START):
            local_start = global_to_local(random_start_pos, self.global_home)
            grid_start = (int(local_start[0]) - north_offset, int(local_start[1]) - east_offset)
    
        # Set goal as latitude / longitude position: np.array([-122.39747, 37.79275, TARGET_ALTITUDE])
        goal_pos = self.randomPoint(grid, north_offset, east_offset, TARGET_ALTITUDE, self.global_home)
        
        # Convert lat/lon to local grid: (325, 455)
        local_goal = global_to_local(goal_pos, self.global_home)
        grid_goal = (int(local_goal[0]) - north_offset, int(local_goal[1]) - east_offset)

        print('Local Start and Goal: ', grid_start, grid_goal)

        if not GRAPH_SEARCH:
    
            # Run A* to find a path from start to goal
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)
    
            # Prune path to minimize number of waypoints
            pruned_path = [p for p in path]
            i = 1
            while i < len(pruned_path) - 1:
                pt1 = pruned_path[i - 1]
                pt2 = pruned_path[i]
                pt3 = pruned_path[i + 1]
                if self.collinearity_check(pt1, pt2, pt3):
                    del pruned_path[i:i+1]
                else:
                    i = i + 1

        # Try a different approach altogether! - Probabilistic Graph
        if (GRAPH_SEARCH):

            # Get samples and polygons
            samples = random_samples(data, 200)
            polygons = extract_polygons(data)
            to_keep = []
            for point in samples:
                if not collides(polygons, point):
                    to_keep.append(point)

            # # Create the graph and determine start and goal points on graph
            graph = create_graph(to_keep, 10, polygons)
            print(graph)
            graph_start = closest_point(graph, current_local_position)
            graph_goal = closest_point(graph, goal_pos)

            # Run A*
            path, _ = a_star_graph(graph, heuristic, graph_start, graph_goal)
            
            # Add original start and goal
            path.append(local_goal)
            path.insert(0, current_local_position)

            # If no path found, just takeoff and land
            if len(path) == 2:
                path.pop()

            # Adjust for offset - should refactor
            pruned_path = [[int(p[0]) - north_offset, int(p[1]) - east_offset] for p in path]

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints
        self.waypoints = waypoints

        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def collinearity_check(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        return x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 -y2) == 0

    def randomPoint(self, grid, north_offset, east_offset, TARGET_ALTITUDE, global_home):
        local_position_ne = np.array([north_offset, east_offset, TARGET_ALTITUDE])
        local_position_sw = np.array([-north_offset, -east_offset, TARGET_ALTITUDE])
        lat_ne, lon_ne, alt = local_to_global(local_position_ne, global_home)
        lat_sw, lon_sw, alt = local_to_global(local_position_sw, global_home)
        random_lat = random.uniform(lat_sw, lat_ne)
        random_lon = random.uniform(lon_ne, lon_sw)
        global_position = np.array([random_lat, random_lon, TARGET_ALTITUDE])
        n, e, d = global_to_local(global_position, global_home)
        while grid[int(n) - north_offset, int(e) - east_offset] == 1:
            random_lat = random.uniform(lat_sw, lat_ne)
            random_lon = random.uniform(lon_ne, lon_sw)
            global_position = np.array([random_lat, random_lon, TARGET_ALTITUDE])
            n, e, d = global_to_local(global_position, global_home) 
        return global_position



    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
