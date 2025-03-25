from DroneClient import DroneClient
import time
from helpers import *


def test():
    height = -70
    start = (-875, -1050)
    midway = (-700, -700)
    end = (-560, -590)
    return height , start, midway, end

if __name__ == "__main__":

    client = DroneClient()
    client.connect()
    print("Drone connected!")

    x_limit_Min = -1250
    x_limit_Max = -670
    y_limit_Min = -1200
    y_limit_Max = -700

    height , start , midway , end = test()

    client.setAtPosition(start[0], start[1], height)
    time.sleep(5)

    map_obstacles , bug , path = initialize_map(start , midway , end , height , client)

    Astar_pos = []
    temp = len(path) - 1
    i = 0
    midwayReached = False
    time_start_midway = time.time()
    while i < temp:
        start_wp = path[i]
        end_wp = path[i + 1]
        print(f"Navigating from {start_wp} to {end_wp}")
        # Try to follow the path using Tangent Bug
        inside =  (x_limit_Min < start_wp[0] < x_limit_Max and y_limit_Min < start_wp[1] < y_limit_Max)
        bug.findPath(Vec2(end_wp[0], end_wp[1]),inside,Vec2(midway[0] , midway[1]))

        #If an obstacle was encountered, update visibility graph and re-run A*
        pose = bug.client.getPose()
        if bug.boundary_followed and ( np.linalg.norm( np.array((pose.pos.x_m , pose.pos.y_m)) - np.array(end_wp))  > 10) :

            print("Obstacle encountered! Recalculating visibility graph and A* path.")
            map_obstacles.generate_full_visibility_graph((round(pose.pos.x_m) , round(pose.pos.y_m)), midway )
            new_path = AstarFind((round(pose.pos.x_m) ,round( pose.pos.y_m)), midway , map_obstacles.visibility_graph)
            #map_obstacles.generate_reduced_visibility_graph((round(pose.pos.x_m), round(pose.pos.y_m)), midway)
            #new_path = AstarFind((round(pose.pos.x_m) ,round( pose.pos.y_m)), midway , map_obstacles.reduced_graph)
            bug.boundary_followed = False

            if new_path:
                Astar_pos.append( ( round(pose.pos.x_m), round( pose.pos.y_m) ) )
                print("New A* path found! Restarting from updated path.")
                print(new_path)
                path = new_path  # Update the path
                temp = len(new_path) - 1
                i = 0
                bug.boundary_followed = False
                continue
            else:
                print("No new A* path found. Continuing with Tangent Bug.")
        i += 1

    print("---------------Midway reached!-------------")
    time_start_midway = time.time()-time_start_midway
    print(client.getPose())
    print("Time Taken From Start To Mid: ", time_start_midway)

    #fly mid --> end
    print("---------------Flying to end goal--------------")
    time_end_midway = time.time()
    bug.findPath(Vec2(end[0], end[1]), False , Vec2(end[0] , end[1]))

    time_end_midway = time.time()-time_end_midway
    print("------------------------------End goal reached!---------------------------------------")
    print("Time Taken From Mid To End: ", time_end_midway)
    print(client.getPose())
    print("Total Time: ", time_end_midway + time_start_midway)
    plot_visibility_graph(map_obstacles, None, bug.pos, Astar_pos,bug.lidar_data)
    exit(0)

