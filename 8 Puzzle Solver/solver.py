import copy
class Node:
    #initializing any state object of Node class
    def __init__(self, previous=None, currState = 0, initial_to_current_cost = 0, current_to_goal_cost = 0):
        self.previous = previous
        self.next = []
        self.currState = currState
        self.initial_to_current_cost = initial_to_current_cost
        self.current_to_goal_cost = current_to_goal_cost

    #check if states are equal or not
    def __eq__(self, __o: object) -> bool:
        return self.currState == __o.currState

    def __lt__(self, __o: object) -> bool:
        return (self.initial_to_current_cost + self.current_to_goal_cost) < (__o.initial_to_current_cost + __o.current_to_goal_cost)

    def add_next(self, node, expand_cost = 1):
        node.initial_to_current_cost = self.initial_to_current_cost + expand_cost
        node.previous = self
        self.next.append(node)

    #Traversing back in the tree and keep traversed nodes count
    def backtrack(self):
        prev = self.previous
        no_previous_nodes = 1

        while prev:
            no_previous_nodes = no_previous_nodes + 1
            print(prev.currState)
            prev = prev.previous
        print(str(no_previous_nodes) + "nodes backtracked")

    #finds all the states that can be achieved by moving empty tile in all possible directions
    def expand(self):
        """Returns a list of valid expanded states."""
        for i in range(len(self.currState)):
            for j in range(len(self.currState[i])):
                if self.currState[i][j] == 0:
                    i_0, j_0 = i, j

        currState_len = len(self.currState)
        
        states_to_return = []
        
        # Move Zero tile Up
        if (i_0 != 0):
            states_to_return.append(move_up(self.currState, self.previous, i_0, j_0))

        
        # Move Zero tile Right
        if (j_0 != (currState_len - 1)):
            states_to_return.append(move_right(self.currState, self.previous, i_0, j_0))

                
        # Move Zero tile Down
        if (i_0 != (currState_len - 1)):
            states_to_return.append(move_down(self.currState, self.previous, i_0, j_0))

        
        # Move Zero tile Left
        if (j_0 != 0):
            states_to_return.append(move_left(self.currState, self.previous, i_0, j_0))
  
        return states_to_return


# move zero tile up
def move_up(currState, previous, i_0, j_0):
    temp_state = copy.deepcopy(currState)
    temp_state[i_0][j_0], temp_state[i_0 - 1][j_0] = temp_state[i_0 - 1][j_0], temp_state[i_0][j_0]
    
    #check if next state is same as previous state
    sameFlag = True
    for i, row in enumerate(temp_state):
        if previous:
            if row != previous.currState[i]:
                sameFlag = False
                break

    if previous and sameFlag:
        return None 
    else:
        return temp_state

# move zero tile right
def move_right(currState, previous, i_0, j_0):
    temp_state = copy.deepcopy(currState)
    temp_state[i_0][j_0], temp_state[i_0][j_0 + 1] = temp_state[i_0][j_0 + 1], temp_state[i_0][j_0]
    
    #check if next state is same as previous state
    sameFlag = True
    for i, row in enumerate(temp_state):
        if previous:
            if row != previous.currState[i]:
                sameFlag = False
                break

    if previous and sameFlag:
        return None
    else:
        return temp_state

# move zero tile down 
def move_down(currState, previous, i_0, j_0):
    temp_state = copy.deepcopy(currState)
    temp_state[i_0][j_0], temp_state[i_0 + 1][j_0] = temp_state[i_0 + 1][j_0], temp_state[i_0][j_0]
    
    #check if next state is same as previous state
    sameFlag = True
    for i, row in enumerate(temp_state):
        if previous:
            if row != previous.currState[i]:
                sameFlag = False
                break

    if previous and sameFlag:
        return None 
    else:
        return temp_state

# move zero tile left
def move_left(currState, previous, i_0, j_0):
    temp_state = copy.deepcopy(currState)
    temp_state[i_0][j_0], temp_state[i_0][j_0 - 1] = temp_state[i_0][j_0 - 1], temp_state[i_0][j_0]
    
    #check if next state is same as previous state
    sameFlag = True
    for i, row in enumerate(temp_state):
        if previous:
            if row != previous.currState[i]:
                sameFlag = False
                break

    if previous and sameFlag:
        return None
    else:
        return temp_state

# These are two function two calculate heuristic in case of A* Algorithm 

from functools import reduce


# Calculating total number of tiles which are not at same place as in goal state
def misplaced_tiles_cost(current_state, goal_state):

    current_state = reduce(lambda x, y: x + y, current_state)
    goal_state = reduce(lambda x, y: x + y, goal_state)
  
    count = 0
  
    for i in range(len(goal_state)):
        if current_state[i] == 0:
            continue
        if (current_state[i] != goal_state[i]):
            count += 1
    return count

# Calculating Manhattan Distance for all tiles which are misplaced 
def manhattan_dist_cost(current_state, goal_state):
    tiles_distances = []

    for i in range(len(current_state)):
        for j in range(len(current_state[i])):
            if (current_state[i][j] == goal_state[i][j] or current_state[i][j] == 0):
                continue
          
            else:
                for x in range(len(goal_state)):
                    for y in range(len(goal_state[x])):
                        if current_state[i][j] == goal_state[x][y]:
                            goal_tile_i, goal_tile_j = x, y
              
                dist = abs(i - goal_tile_i) + abs(j - goal_tile_j)
                tiles_distances.append(dist)
  
    return sum(tiles_distances)

import heapq
def general_search(initial_state, goal_state, heuristic):
    root = Node(currState=initial_state)
    hq = []
    heapq.heappush(hq,root)
    already_traversed_nodes = []
    max_nodes = 1
    expanded_nodes = 0

    while hq:
        max_nodes = max(len(hq), max_nodes)
        node = heapq.heappop(hq)
        
        # checking if current state is same as goal state if it is same return this solution
        sameFlag = True
        for i, row in enumerate(node.currState):
            if row != goal_state[i]:
                sameFlag = False
                break
        if sameFlag :
            print('Solution for the given problem using specified algorithm is found and its solution is as follows :')
            node.backtrack
            print("this solution required expansion of " + str(expanded_nodes) + "nodes")
            print("The maximum nodes in the queue: " + str(max_nodes))
            print("The depth of the solution is:" + str(node.initial_to_current_cost) )

            return expanded_nodes, max_nodes
    
        else:
            already_traversed_nodes.append(node)
            expanded_states = [state for state in node.expand() if state]

        if expanded_states == []:
            continue
      
        for state in expanded_states:
            node_to_check = Node(currState=state)

            #if this state is already visited then skip it
            if ((hq and node_to_check in hq) or (already_traversed_nodes and node_to_check in already_traversed_nodes)):
                continue

            #A* with Misplaced tiles 
            if (heuristic == "a_star_misplaced_tiles"):
                node_to_check.current_to_goal_cost = misplaced_tiles_cost(node_to_check.currState, goal_state)

            #A* with manhattan distance
            if (heuristic == "a_star_manhattan_dist"):
                node_to_check.current_to_goal_cost = manhattan_dist_cost(node_to_check.currState, goal_state)

            node.add_next(node=node_to_check)

            heapq.heappush(hq, node_to_check)

        expanded_nodes += 1
    print('No solution found!')
    return -1

#Take initial and goal state as input

from time import perf_counter
print('Enter Dimensionality of problem')
no_of_rows = int(input())
initial_state = []

print('Enter initial state row wise with space seperated entries. Enter 0 for blank node')

for i in range(0, no_of_rows):
    row = input()
    row = [int(x) for x in row.split()]
    initial_state.append(row)

goal_state = []

print('Enter Goal state row wise with space seperated entries. Enter 0 for blank node')

for i in range(0, no_of_rows):
    row = input()
    row = [int(x) for x in row.split()]
    goal_state.append(row)


print("Select the algorithm with heuristic function:\n"
        + " Enter 1 for Uniform Cost Search" + "\n"
        + " Enter 2 for A* Misplaced Tile Heuristic" + "\n"
        + " Enter 3 for A* Manhattan Distance Heuristic" + "\n"
        )

#Select Algorithm to perform
alg = int(input())


if alg == 2:
    alg = "a_star_misplaced_tiles"
elif alg == 3:
    alg = "a_star_manhattan_dist"

# Calling function and measuring time to perform it
start_time = perf_counter()
general_search(initial_state, goal_state, alg)
end_time = perf_counter()
duration = (end_time - start_time)
print("Time taken to find the solution for this problem using selected algorithm is %s milliseconds ---" % round(duration*1000, 2))