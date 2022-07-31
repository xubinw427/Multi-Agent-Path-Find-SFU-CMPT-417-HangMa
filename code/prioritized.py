import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = [
            # {'agent': 1, 'loc': [(1, 4)], 'timestep': 2},
            # {'agent': 1, 'loc': [(1, 3)], 'timestep': 2},
            # {'agent': 1, 'loc': [(1, 2)], 'timestep': 2}
            # {'agent': 0, 'loc': [(1, 5)], 'timestep': 10}
            # {'agent': 0, 'loc': [(1, 5)], 'timestep': 4},
            # {'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1}
        ]
        # Count the all possible location as the map size
        map_size = 0
        for each in self.my_map:
            map_size += each.count(False)
        # print("Map size is ", map_size)
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            sum_higher_pris = 0
            for each in result:
                sum_higher_pris += len(each)
            upper_bound = map_size + sum_higher_pris
            print("Upper Bound is ", upper_bound)
            print("Length of current agent path is ", len(path))
            if path is None or len(path) > upper_bound:
                raise BaseException('No solutions')
            result.append(path)
            # Total time step of current agent
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for j in range(i + 1, self.num_of_agents):
                time_for_vertex = 0
                # Add constrain of each future agents of each loc at each time step
                # Add Vertex Constrain
                for k in path:
                    temp_vertex = {'agent': j,
                                   'loc': [k],
                                   'timestep': time_for_vertex}
                    time_for_vertex += 1
                    constraints.append(temp_vertex)
                time_for_edge = 1
                # Add Edge Constrains
                for index in range(len(path) - 1):
                    temp_edge_forward = {'agent': j,
                                         'loc': [path[index], path[index + 1]],
                                         'timestep': time_for_edge,
                                         'positive': False}
                    temp_edge_backward = {'agent': j,
                                          'loc': [path[index + 1], path[index]],
                                          'timestep': time_for_edge,
                                          'positive': False}
                    time_for_edge += 1
                    constraints.append(temp_edge_forward)
                    constraints.append(temp_edge_backward)
                # Add Additional Constrains
                duration = upper_bound + len(path)
                # print("In the loop, the time duration is ", duration)
                for t in range(len(path), duration):
                    future_cons = {'agent': j,
                                   'loc': [path[-1]],
                                   'timestep': t,
                                   'positive': False}
                    constraints.append(future_cons)
            ##############################
        # print(constraints)
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
