"""
https://github.com/qiao/PathFinding.js/blob/master/src/finders/AStarFinder.js
TODO: Remove sources from here ^

Navigation module to handle pathfinding.

Inspired by https://github.com/qiao/PathFinding.js/blob/master/src/finders/AStarFinder.js.
"""

import heapq
import math
import numpy as np

from itertools import count
from dataclasses import dataclass

from src.consts import *


@dataclass
class Node:
    position: tuple[int, int]
    nature: int

    parent: Node | None = None
    f: float | None = None
    g: float | None = None
    h: float | None = None
    opened: bool | None = None
    closed: bool | None = None


class Navigation:
    def __init__(self):
        self.rows, self.cols = GRID_SHAPE
        self.grid = np.zeros((self.rows, self.cols), dtype=int)
        self.plan = []
        self.node_grid = [[Node(position=(r, c), nature=CELL_VOID)
                           for c in range(self.cols)] for r in range(self.rows)]
        self.counter = count()

    def update_plan(self, current_grid, current_plan, robot_position, goal_position):
        # If grid changed, calculate new plan, otherwise return.
        if not np.array_equal(self.grid, current_grid, current_plan):
            self.grid = current_grid
            self.node_grid = [[Node(position=(r, c), nature=current_grid[r, c]) for c in range(
                current_grid.shape[1])] for r in range(current_grid.shape[0])]
            self.plan = self.generate_plan(robot_position, goal_position)

            if self.plan == current_plan:
                return (False, self.plan)
            else:
                return (True, self.plan)
        else:
            self.plan = current_plan
            return (False, self.plan)

    def a_star(self, start, end):
        self.node_grid = [[Node(position=(r, c), nature=CELL_VOID)
                           for c in range(self.cols)] for r in range(self.rows)]
        open_heap = []

        # set f and g to be 0 for the starting node
        start_node = self.node_grid[start[0]][start[1]]
        end_node = self.node_grid[end[0]][end[1]]
        start_node.f = 0
        start_node.g = 0

        heapq.heappush(
            open_heap, (start_node.f, next(self.counter), start_node))
        start_node.opened = True

        while open_heap:

            node = heapq.heappop(open_heap)[2]
            node.closed = True

            # if reached the end position, construct the path and return it
            if (node.position == end):
                return self.construct_path(end_node)

            # get neighbours of the current node
            neighbors = self.get_neighbors(node)

            for neighbor in neighbors:

                if neighbor.closed:
                    continue

                g_score = node.g + (1 if (neighbor.position[X] - node.position[X] ==
                                    0 or neighbor.position[Y] - node.position[Y] == 0) else math.sqrt(2))

                if (not neighbor.opened) or (g_score < neighbor.g):
                    neighbor.g = g_score
                    neighbor.h = (WEIGHT * self.chebyshev_distance(abs(neighbor.position[X] - end[X]), abs(
                        neighbor.position[Y] - end[Y]))) if ((neighbor.h is None) or neighbor.h == 0) else neighbor.h
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = node

                    if not neighbor.opened:
                        heapq.heappush(
                            open_heap, (neighbor.f, next(self.counter), neighbor))
                        neighbor.opened = True
                    else:
                        # update the position of the neighbor in the heap
                        for index, (f, _, n) in enumerate(open_heap):
                            if n == neighbor:
                                open_heap[index] = (
                                    neighbor.f, next(self.counter), neighbor)
                                heapq.heapify(open_heap)
                                break

        return []  # no path found

    def get_neighbors(self, node):
        neighbors = []

        movements = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (-1, 1), (1, 1), (1, -1), (-1, -1)]

        s0 = s1 = s2 = s3 = d0 = d1 = d2 = d3 = False

        """
             offsets      diagonalOffsets:
          +---+---+---+    +---+---+---+
          |   | 0 |   |    | 0 |   | 1 |
          +---+---+---+    +---+---+---+
          | 3 |   | 1 |    |   |   |   |
          +---+---+---+    +---+---+---+
          |   | 2 |   |    | 3 |   | 2 |
          +---+---+---+    +---+---+---+
        """

        i = 0
        for move in movements[:4]:
            new_pos = (node.position[X] + move[X], node.position[Y] + move[Y])

            # Check if new position is within grid bounds and not an obstacle
            if (0 <= new_pos[0] < GRID_SHAPE[0] and 0 <= new_pos[1] < GRID_SHAPE[1] and
                    self.grid[new_pos[0], new_pos[1]] == CELL_VOID):

                if i == 0:
                    s0 = True
                elif i == 1:
                    s1 = True
                elif i == 2:
                    s2 = True
                elif i == 3:
                    s3 = True

                neighbors.append(self.node_grid[new_pos[0]][new_pos[1]])

            i += 1

        for move in movements[4:]:
            new_pos = (node.position[X] + move[X], node.position[Y] + move[Y])

            # Check if new position is within grid bounds and not an obstacle
            if (0 <= new_pos[0] < GRID_SHAPE[0] and 0 <= new_pos[1] < GRID_SHAPE[1] and
                    self.grid[new_pos[0], new_pos[1]] == CELL_VOID):

                if move == (-1, 1) and s0 and s3:
                    neighbors.append(self.node_grid[new_pos[0]][new_pos[1]])
                elif move == (1, 1) and s0 and s1:
                    neighbors.append(self.node_grid[new_pos[0]][new_pos[1]])
                elif move == (1, -1) and s1 and s2:
                    neighbors.append(self.node_grid[new_pos[0]][new_pos[1]])
                elif move == (-1, -1) and s2 and s3:
                    neighbors.append(self.node_grid[new_pos[0]][new_pos[1]])

        return neighbors

    def construct_path(self, node):
        path = [(node.position)]
        while node.parent is not None:
            node = node.parent
            path.append(node.position)
        return path[::-1]  # Return reversed path

    def chebyshev_distance(self, dx, dy):
        return max(dx, dy)

    def generate_plan(self, start, end):
        path = self.a_star(start, end)

        # Cas ou a_star n'a pas trouve de chemin
        if not (len(path) > 0):
            return []

        # Remove starting position
        path = path[1:]

        # Convert path to list of (x, y) tuples goals. Goals are the nodes where the direction changes.
        plan = []
        previous_direction = None
        for i in range(1, len(path)):
            current_direction = (path[i][0] - path[i-1][0],
                                 path[i][1] - path[i-1][1])
            if current_direction != previous_direction:
                plan.append(path[i-1])
                previous_direction = current_direction
        #plan.append(path[-1])
        plan = plan[1:]
        plan.append(end)
        return plan


if __name__ == "__main__":
    import cv2
    import numpy as np

    nav = Navigation()

    # Initial positions
    start = [0, 0]
    end = [0, 1]

    # State variables
    dragging_start = False
    dragging_end = False
    path = []
    plan = []
    drawing_obstacle = False
    erasing_obstacle = False

    CELL_SIZE = 8
    WINDOW_WIDTH = GRID_SHAPE[1] * CELL_SIZE
    WINDOW_HEIGHT = GRID_SHAPE[0] * CELL_SIZE

    def mouse_callback(event, x, y, flags, param):
        global dragging_start, dragging_end, drawing_obstacle, erasing_obstacle, path, plan

        grid_x = min(x // CELL_SIZE, GRID_SHAPE[1] - 1)
        grid_y = min(y // CELL_SIZE, GRID_SHAPE[0] - 1)

        if event == cv2.EVENT_LBUTTONDOWN:
            if (grid_y, grid_x) == tuple(start):
                dragging_start = True
            elif (grid_y, grid_x) == tuple(end):
                dragging_end = True
            elif nav.grid[grid_y, grid_x] == CELL_OBSTACLE:
                erasing_obstacle = True
                nav.grid[grid_y, grid_x] = CELL_VOID
                path = []
                plan = []
            else:
                drawing_obstacle = True
                nav.grid[grid_y, grid_x] = CELL_OBSTACLE
                path = []
                plan = []

        elif event == cv2.EVENT_MOUSEMOVE:
            if dragging_start:
                start[0], start[1] = grid_y, grid_x
                path = []
                plan = []
            elif dragging_end:
                end[0], end[1] = grid_y, grid_x
                path = []
                plan = []
            elif drawing_obstacle:
                nav.grid[grid_y, grid_x] = CELL_OBSTACLE
                path = []
                plan = []
            elif erasing_obstacle:
                nav.grid[grid_y, grid_x] = CELL_VOID
                path = []
                plan = []

        elif event == cv2.EVENT_LBUTTONUP:
            dragging_start = False
            dragging_end = False
            drawing_obstacle = False
            erasing_obstacle = False

    cv2.namedWindow("Pathfinding")
    cv2.setMouseCallback("Pathfinding", mouse_callback)

    print("Controls:")
    print("- Drag green square to move start position")
    print("- Drag red square to move end position")
    print("- Click/drag on empty cells to add obstacles")
    print("- Click/drag on obstacles to remove them")
    print("- Press 's' to run A* pathfinding")
    print("- Press 'c' to clear obstacles")
    print("- Press 'q' to quit")

    while True:
        # Create visualization
        grid_visual = np.ones(
            (GRID_SHAPE[0], GRID_SHAPE[1], 3), dtype=np.uint8) * 255

        # Draw obstacles
        for r in range(GRID_SHAPE[0]):
            for c in range(GRID_SHAPE[1]):
                if nav.grid[r, c] == CELL_OBSTACLE:
                    grid_visual[r, c] = (0, 0, 0)

        # Draw path in yellow
        for pos in path:
            grid_visual[pos[0], pos[1]] = (0, 255, 255)


        # Draw start and end
        grid_visual[start[0], start[1]] = (0, 255, 0)  # Green
        grid_visual[end[0], end[1]] = (0, 0, 255)  # Red

        # Draw plan in blue
        for pos in plan:
            grid_visual[pos[0], pos[1]] = (255, 0, 0)

        # Resize for better visibility
        display = cv2.resize(grid_visual, (WINDOW_WIDTH, WINDOW_HEIGHT),
                             interpolation=cv2.INTER_NEAREST)

        # Draw grid lines
        for i in range(0, WINDOW_HEIGHT, CELL_SIZE):
            cv2.line(display, (0, i), (WINDOW_WIDTH, i), (200, 200, 200), 1)
        for i in range(0, WINDOW_WIDTH, CELL_SIZE):
            cv2.line(display, (i, 0), (i, WINDOW_HEIGHT), (200, 200, 200), 1)

        cv2.imshow("Pathfinding", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # Reset node grid
            nav.node_grid = [[Node(position=(r, c), nature=nav.grid[r, c])
                             for c in range(GRID_SHAPE[1])] for r in range(GRID_SHAPE[0])]
            nav.counter = count()

            path = nav.a_star(tuple(start), tuple(end))
            if path:
                print(f"Path found with {len(path)} steps")
                print("Path:", path)
                plan = nav.generate_plan(tuple(start), tuple(end))
                print(f"Plan generated with {len(plan)} waypoints")
                print("Plan:", plan)
            else:
                print("No path found!")
                plan = []

        elif key == ord('c'):
            nav.grid = np.zeros((GRID_SHAPE[0], GRID_SHAPE[1]), dtype=int)
            path = []
            plan = []

        elif key == ord('q'):
            break

    cv2.destroyAllWindows()
