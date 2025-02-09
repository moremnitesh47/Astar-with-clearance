def astar(self, start, goal):

        """
        Perform A* pathfinding algorithm to find a path from start to goal.

        This implementation includes clearance checking and a fallback to the closest
        reachable cell if the goal is unreachable. It uses a Manhattan distance heuristic
        and considers both orthogonal and diagonal movements.

        Args:
            start (tuple): The starting position as a tuple of (x, y) coordinates.
            goal (tuple): The goal position as a tuple of (x, y) coordinates.

        Returns:
            list: A list of tuples representing the path from start to goal (or closest
                reachable point). Each tuple contains (x, y) coordinates. Returns an
                empty list if no path is found
        """
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def cost(current, neighbor):
            base_cost = 1.414 if (neighbor[0] - current[0]) * (neighbor[1] - current[1]) != 0 else 1
            clearance = self.get_clearance(neighbor[0], neighbor[1])
            clearance_penalty = 10 if clearance == 0 else 2.0 / clearance
            return base_cost + clearance_penalty

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        closest_to_goal = start
        min_distance_to_goal = heuristic(start, goal)

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal and self.is_valid_cell(goal[0], goal[1], self.clearance_distance):
                return self.reconstruct_path(came_from, current)

            current_distance = heuristic(current, goal)
            if current_distance < min_distance_to_goal:
                closest_to_goal = current
                min_distance_to_goal = current_distance

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                if self.is_valid_cell(neighbor[0], neighbor[1], self.clearance_distance):
                    tentative_g_score = g_score[current] + cost(current, neighbor)
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        if closest_to_goal != start:
            self.get_logger().warn(f"Goal unreachable. Navigating to closest reachable cell at {closest_to_goal}.")
            return self.reconstruct_path(came_from, closest_to_goal)

        return []
    
        """

         The Astar algorithm for Path Planning used from this repositiory and restructured for our use.
                Reference:  https://github.com/Chrisbelefantis/A-Star-Algorithm/blob/master/Astar-Algorithm.py#L246 [Line 246 to 308]

        """
