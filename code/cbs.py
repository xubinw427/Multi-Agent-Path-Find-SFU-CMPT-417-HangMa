import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    path_1 = len(path1)
    path_2 = len(path2)
    time_duration = max(path_1, path_2)
    for t in range(time_duration):
        loc_1 = get_location(path1, t)
        loc_2 = get_location(path2, t)
        # Check vertex cons
        if loc_1 == loc_2:
            return [loc_1], t
        # Check edge cons
        if t < time_duration - 1:
            next_t = t + 1
            loc_1_next = get_location(path1, next_t)
            loc_2_next = get_location(path2, next_t)
            if loc_1_next == loc_2 and loc_2_next == loc_1:
                return [loc_1, loc_1_next], next_t
    return None, None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    agent_num = len(paths)
    for i in range(agent_num - 1):
        for j in range(i + 1, agent_num):
            cons, time = detect_collision(paths[i], paths[j])
            if cons is not None and time is not None:
                temp_collision = {'a1': i,
                                  'a2': j,
                                  'loc': cons,
                                  'timestep': time
                                  }
                collisions.append(temp_collision)
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    cons = []
    loc = collision['loc']
    a1 = collision['a1']
    a2 = collision['a2']
    time = collision['timestep']
    coll_size = len(loc)
    # vertex constrain
    if coll_size == 1:
        temp_cons1 = {'agent': a1, 'loc': loc, 'timestep': time, 'positive': False}
        temp_cons2 = {'agent': a2, 'loc': loc, 'timestep': time, 'positive': False}
        cons.append(temp_cons1)
        cons.append(temp_cons2)
    # edge constrain
    else:
        # Add two direction constrains of agent 1
        temp_cons1 = {'agent': a1, 'loc': loc, 'timestep': time, 'positive': False}
        # temp_cons2 = {'agent': a1, 'loc': [loc[1], loc[0]], 'timestep': time, 'positive': False}
        # Add two direction constrains of agent 2
        # temp_cons3 = {'agent': a2, 'loc': loc, 'timestep': time, 'positive': False}
        temp_cons4 = {'agent': a2, 'loc': [loc[1], loc[0]], 'timestep': time, 'positive': False}
        cons.append(temp_cons1)
        # cons.append(temp_cons2)
        # cons.append(temp_cons3)
        cons.append(temp_cons4)

    return cons


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    cons = []
    loc = collision['loc']
    a1 = collision['a1']
    a2 = collision['a2']
    time = collision['timestep']
    coll_size = len(loc)
    order = random.randint(0, 1)  # 0 to force agent1 move and 1 to force agent2 move.
    # print("The forced agent is: ", order)
    forced_agent = a1 if order else a2
    # Vertex collision
    if coll_size == 1:
        temp_cons1 = {'agent': forced_agent, 'loc': loc, 'timestep': time, 'positive': True}
        temp_cons2 = {'agent': forced_agent, 'loc': loc, 'timestep': time, 'positive': False}
        cons.append(temp_cons1)
        cons.append(temp_cons2)
    # Edge collision
    else:
        temp_cons1 = {'agent': forced_agent, 'loc': loc, 'timestep': time, 'positive': True}
        temp_cons4 = {'agent': forced_agent, 'loc': [loc[1], loc[0]], 'timestep': time, 'positive': False}
        cons.append(temp_cons1)
        cons.append(temp_cons4)
    return cons


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        if disjoint:
            print("Now is using Disjoint Splitting")
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
        # print("Now Running CBS Solver")
        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            next_node = self.pop_node()
            if len(next_node['collisions']) == 0:
                self.print_results(next_node)
                return next_node['paths']
            first_colli = next_node['collisions'][0]
            # cons = disjoint_splitting(first_colli)
            # cons = standard_splitting(first_colli)
            cons = disjoint_splitting(first_colli) if disjoint else standard_splitting(first_colli)
            # print("Disjoint Constraints are:", cons)
            for constrain in cons:
                Q = {'constraints': next_node['constraints'] + [constrain],
                     'paths': copy.deepcopy(next_node['paths'])}
                agent_index = constrain['agent']
                new_path = a_star(self.my_map, self.starts[agent_index], self.goals[agent_index],
                                  self.heuristics[agent_index],
                                  agent_index, Q['constraints'])
                forbid = False
                if new_path is not None:
                    # This part was inspired at https://github.com/timaMa/cmpt417/blob/main/code/cbs.py#L177
                    if constrain['positive']:
                        rst = paths_violate_constraint(constrain, next_node['paths'])
                        for index in rst:
                            i_path = a_star(self.my_map, self.starts[index], self.goals[index], self.heuristics[index],
                                            index, Q['constraints'])
                            if i_path is not None:
                                Q['paths'][index] = i_path
                            else:
                                forbid = True
                                break
                    Q['paths'][agent_index] = new_path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    if not forbid:
                        self.push_node(Q)
        # print("Works here ")
        self.print_results(root)
        return 'No Solution'

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
