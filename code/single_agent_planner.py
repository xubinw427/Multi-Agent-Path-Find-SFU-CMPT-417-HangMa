import heapq
import copy
Test_Flag = False


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    # print(h_values)
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    table = dict()
    for cons in constraints:
        ts = cons['timestep']
        if cons['agent'] == agent:  # and not cons['positive']:
            if ts not in table:
                table[ts] = []
            table[ts].append(cons)
        elif cons['positive']:
            new_cons = copy.deepcopy(cons)
            new_cons['agent'] = agent
            new_cons['positive'] = False
            new_cons['loc'].reverse()
            if ts not in table:
                table[ts] = []
            table[ts].append(new_cons)
        if Test_Flag:
            print(cons['agent'], " cannot be at", cons['loc'], " at ", cons['timestep'])
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if Test_Flag:
        print("Now in is_constrained Function")
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        cons_list = constraint_table[next_time]
        for cons in cons_list:
            if cons['positive']:
                if len(cons['loc']) > 1:
                    if cons['loc'][0] != curr_loc or cons['loc'][1] != next_loc:
                        return True
                else:
                    if cons['loc'][0] != next_loc:
                        return True

            elif len(cons['loc']) > 1:
                if Test_Flag:
                    print("Now processing edge constrains")
                    print("Start Constrain loc is: ", cons['loc'][0])
                    print("Current loc is: ", curr_loc)
                    print("End Constrain loc is: ", cons['loc'][1])
                    print("Next loc is: ", next_loc)
                if cons['loc'][0] == curr_loc and cons['loc'][1] == next_loc:
                    return True
            else:
                if Test_Flag:
                    print("Has cons at ", cons['loc'])
                    print("Current loc is: ", curr_loc)
                    print("Next loc is: ", next_loc)
                if cons['loc'][0] == next_loc:
                    if Test_Flag:
                        print("Should Be Stop at ", cons['loc'])
                    return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    if Test_Flag:
        print("Now processing Agent ", agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    cons_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if Test_Flag:
            print("At time Step ", curr['time_step'], " Agent ", agent, " at location ", curr['loc'],
                  "```````````````````````````")
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            # There are no future constrains
            if len(cons_table) == 0:
                return get_path(curr)
            # Future Constrains included, should move from the goal loc
            # e.g. agent 0 at time step 10
            else:
                latest_cons = max(cons_table)
                curr_time = curr['time_step']
                if curr_time >= latest_cons:
                    return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            next_time = curr['time_step'] + 1
            if Test_Flag:
                print("Now is moving Agent ", agent, " From ", curr['loc'], " to ", child_loc, " at Time step",
                      next_time)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]] or is_constrained(curr['loc'], child_loc, next_time, cons_table):
                if Test_Flag:
                    if is_constrained(curr['loc'], child_loc, next_time, cons_table):
                        print("constrain processed!")
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'time_step': next_time}
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
