from map import *
from TangentBug import *
import matplotlib.pyplot as plt

def AstarFind( start , end , visibility_graph):
    print('------------------------------------------------------------------------------------------------------------')

    G_value = {node: float('inf') for node in visibility_graph}
    G_value[start] = 0
    #h_value = {node: np.linalg.norm(np.array(node) - np.array(end)) for node in nodes}
    pq = []
    parent = dict()
    parent[start] = None
    heapq.heappush(pq, ( np.linalg.norm(np.array(start) - np.array(end)) , start) )
    while pq:
        currentNode = heapq.heappop(pq)  # current node = ( 5.6 , (1,1) )
        if currentNode[1] == end:
            optimalPath = []
            current = currentNode[1]
            while current != start:
                optimalPath.append(current)
                current = parent[current]
            optimalPath.append(start)
            return optimalPath[::-1]

        for child,dist in visibility_graph[currentNode[1]]:

            if (G_value[child] > dist + G_value[currentNode[1]]):

                parent[child] = currentNode[1]
                G_value[child] = dist + G_value[currentNode[1]]
                f_value = G_value[child] + np.linalg.norm(np.array(child) - np.array(end)) # g + h
                if child not in [c[1] for c in pq]:
                    heapq.heappush(pq, (f_value, child))
                else:
                    pass
    return None



def initialize_map(start , mid , end , height, client : DroneClient):
    map_obstacles = MapLoading(start, mid, end)  # also generate the graph ( doesnt call Astar )
    print("Map obstacles generated!")
    bug = TangentBug(client, height)
    path = AstarFind(start, mid, map_obstacles.visibility_graph)  # returns a list of points
    print("Plotting initial graph!")
    plot_visibility_graph(map_obstacles, path, None, None )
    print("original path is :")
    print(path)
    print("------------------------------initialization finished--------------------------------------------")
    return map_obstacles , bug , path



def plot_visibility_graph(map_load : MapLoading, path , droneRoute , Astar_pos, data = None , reduced=False):
    fig, ax = plt.subplots(figsize=(10, 10))

    ax.scatter(map_load.start[0],map_load.start[1], color='green', s=115, label='Start', zorder=5,
               edgecolors='black', linewidth=1.2)
    ax.scatter(map_load.midway[0],map_load.midway[1], color='red', s=115, label='Mid', zorder=5,
               edgecolors='black', linewidth=1.2)
    ax.scatter(map_load.end[0], map_load.end[1], color='blue', s=115, label='end', zorder=5,
               edgecolors='black', linewidth=1.2)
    # Plot obstacles
    for polygon in map_load.all_polygons:
        x, y = polygon.exterior.xy  # Get the exterior coordinates of the polygon
        # color = self.random_color()  # Get a random color
        ax.fill(x, y, alpha=0.5, edgecolor='black', facecolor='red')  # Plot with random color

    #visibility graph type
    visibility_graph = map_load.reduced_graph if reduced else map_load.visibility_graph

    # Plot visibility edges
    for node, edges in visibility_graph.items():
        for neighbor, _ in edges:
            line = LineString([node, neighbor])
            x, y = line.xy
            ax.plot(x, y, color="blue", linewidth=0.2, linestyle="-", alpha=0.7, zorder=2)  # Force edges to be visible

    if path is not None:
        x_values, y_values = zip(*path)  # Extract x and y coordinates
        ax.plot(x_values, y_values, marker='o', linestyle='-', color='red', markersize=8, label="Path")

    if droneRoute is not None:
        pos_x, pos_y = zip(*droneRoute)
        plt.scatter(pos_x, pos_y, color='yellow', linewidth=0.1, marker='.', label='Position Points' , zorder=10)

    if Astar_pos is not None:
        for i in Astar_pos:
            ax.scatter(i[0], i[1], color='purple', s=115, label='Astar Run', zorder=5,
                       edgecolors='black', linewidth=1.2)

    if data is not None:
        pos_x, pos_y = zip(*data)
        plt.scatter(pos_x, pos_y, color='black', linewidth=0.5, marker='.', label='lidar Points',zorder=10)


    ax.set_xlabel("X (meters)")
    plt.gca().invert_xaxis()
    ax.set_ylabel("Y (meters)")
    ax.set_title("Visibility Graph" )
    # Remove duplicate labels in the legend
    handles, labels = ax.get_legend_handles_labels()
    unique_labels = dict(zip(labels, handles))  # Removes duplicates
    ax.legend(unique_labels.values(), unique_labels.keys(), loc="upper left")
    plt.show()