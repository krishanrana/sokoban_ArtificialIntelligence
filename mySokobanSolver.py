
#import extra functions
import search
import sokoban
import time
from sokoban import Warehouse
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
puzzle_t3 ='''
#######
#@ $ .#
#. $  #
#######'''

def my_team():
    ''' Input: Null

        Output: List of team members in form (student_number, first_name, last_name)

        Description: Returns the names and student numbers of the students who completed this assignment
    '''
    return [(9240781, 'Krishan', 'Rana'), (9103023, 'Dimity', 'Miller')]


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def convert_to_string(warehouse, boxes = None, worker = None):
    ''' Input: warehouse object, list of box locations and list of worker location

        Output: A string representation of the warehouse

        Description: The function creates a string showing the location of the warehouse walls, targets, boxes and worker.
        Symbols are # = wall, . = empty target, $ = box, * = box on target, @ = worker, ! = worker on target
    '''

    #find the size of the warehouse grid
    X,Y = zip(*warehouse.walls) 
    x_size, y_size = 1+max(X), 1+max(Y)
    
    #initialize array and set walls and targets to their related symbol
    wh_array = [[" "] * x_size for y in range(y_size)]
    for (x,y) in warehouse.walls:
        wh_array[y][x] = "#"
    for (x,y) in warehouse.targets:
        wh_array[y][x] = "."

    # if worker is on a target display a "!", otherwise a "@"
    wh_array[worker[1]][worker[0]] = "!" if wh_array[worker[1]][worker[0]] == "." else "@"
   
    # if a box is on a target display a "*", otherwise a "$"
    for (x,y) in boxes:
        wh_array[y][x] = "*" if wh_array[y][x] == "." else "$"
    
    #return above array as a string
    return "\n".join(["".join(line) for line in wh_array])

def taboo_cells(warehouse):
    ''' Input: warehouse object

        Output: a string representation of the warehouse

        Description: The function finds the taboo cells of a warehouse, where a taboo cell is a possible box location that will make the 
        game unsolvable. The function first finds the cells that are out of bounds of the warehouse walls, as these should not be marked 
        as a taboo cell. Corners are then found and classed as a taboo cell, and straight lines of walls between corners without adjacent 
        targets are found and also marked as a taboo cell. Taboo cells and walls are then converted to the relevant string symbol.
        Symbols are # = wall, X = taboo cell
    '''


    #find the size of the warehouse grid      
    X,Y = zip(*warehouse.walls)
    x_size, y_size = 1+max(X), 1+max(Y)
   
    '''Finding out of bounds cells'''
    #go through cell locations and check if it is less or more than all the wall values for that row or column
    #row check - left to right and right to left
    out_of_bounds=[(x,y) for y in range(y_size) for x in range(x_size) if all(i > x for i in [X[idx] for idx in range(len(X)) if Y[idx] == y]) or all(i < x for i in [X[idx] for idx in range(len(X)) if Y[idx] == y])]
    #column check - top to bottom and bottom to top
    out_of_bounds = out_of_bounds + [(x,y) for x in range(x_size) for y in range(y_size) if (x,y) not in out_of_bounds and all(i > y for i in [Y[idx] for idx in range(len(Y)) if X[idx] == x]) or all(i < y for i in [Y[idx] for idx in range(len(Y)) if X[idx] == x])]
    
    # find any out of bounds that were not detected before by checking if any cells are touching an out of bounds cell
    all_cells = out_of_bounds + warehouse.walls + warehouse.targets + warehouse.boxes + list(warehouse.worker)
    out_of_bounds = out_of_bounds + [(x,y) for x in range(x_size) for y in range(y_size) for (a,b) in [(1, 0), (-1, 0), (0, 1), (0, -1)] if (x,y) not in out_of_bounds and (x+a, y+b) in out_of_bounds and (x, y) not in all_cells]

    #include little islands inside the map as out_of_bounds by using the can_go_there function - only check if not already categorized
    all_cells = out_of_bounds + warehouse.walls + warehouse.targets + warehouse.boxes + list(warehouse.worker)
    #want to disregard boxes for the can_go_there function
    test_warehouse = warehouse.copy()
    test_warehouse.boxes = []
    out_of_bounds = out_of_bounds +  [(x,y) for x in range(x_size) for y in range(y_size) if (x,y) not in all_cells and not can_go_there(test_warehouse, (y,x))]
   

    '''Finding taboo cells'''
    ##check for all corners - has to have a wall above or below AND a wall left or right
    corners = [(x,y) for y in range(y_size) for x in range(x_size) if (((x+1,y) in warehouse.walls or (x-1, y)in warehouse.walls)  and ((x,y+1) in warehouse.walls or (x, y-1)in warehouse.walls)) and (x,y) not in out_of_bounds and (x,y) not in warehouse.walls and (x,y) not in warehouse.targets]
    
    ##vertical lines of taboo cells with no goal by checking for vertical wall lines between corners with no break
    #record x, start y and end y
    vertical_taboo_walls = [(x,y,z) for (x,y) in corners for (m,z) in corners if x is m and y is not z and all(t ==1 for t in [1 if ((x-1,k) in warehouse.walls or (x+1,k) in warehouse.walls) and (x,k) not in warehouse.walls and (x,k) not in warehouse.targets else 0 for k in range(min(y, z), max(y,z) + 1)])]
    
    ##horizontal lines of taboo cells with no goal by checking for horizontal wall lines between corners with no break
    #record y, start x and end x 
    horizontal_taboo_walls = [(y, x, m) for (x,y) in corners for (m,z) in corners if y is z and x is not m and all(t == 1 for t in [1 if ((k,y-1) in warehouse.walls or (k,y+1) in warehouse.walls) and (k,y) not in warehouse.walls and (k,y) not in warehouse.targets else 0 for k in range(min(x, m), max(x,m) + 1)])]
    
    #add coordinates for each of the taboo walls to taboo cells and add corners
    taboo_cells = list(set([(x,l) for (x,y,z) in vertical_taboo_walls for l in range(min(y,z),max(y,z)+1) if (x,l) not in corners]))
    taboo_cells = taboo_cells + list(set([(l,y) for (y, x, m) in horizontal_taboo_walls for l in range(min(x,m), max(x,m)+1) if (l,y) not in corners]))    
    taboo_cells = taboo_cells + corners


    '''Convert to a string'''
    #initialize string array to correct size
    vis = [[" "] * x_size for j in range(y_size)]
    # allocate symbols to walls and taboo cells
    for (x,j) in warehouse.walls:
        vis[j][x] = "#"
    for (x,j) in taboo_cells:
        vis[j][x] = "X"
    
    #Returning as a string
    warehouse_string = "\n".join(["".join(line) for line in vis])
    
    #included for easy visualization
    print(warehouse_string)
  
    return warehouse_string
    
   

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


class SokobanPuzzle(search.Problem):
    '''
        Description: A class that creates a sokoban puzzle for use with the search functions. It is used for elementary, macro and
        can_go_there solving functions.
    '''

    def __init__(self, warehouse, taboo = [], x_size = False, y_size = False, new_target = None, can_go_there = False, macro = False):
        '''
            Input: 
                Compulsory: a warehouse object 
                Optional: the taboo cells of the warehouse, the x and y size of the warehouse, new_target coordinates and a boolean
                describing if the search is can_go_there or macro.

            Description: Initializes the SokobanPuzzle object given the input parameters. Walls, targets and boxes are manipulated
            depending on if the puzzle is being used for an elementary, macro or can_go_there search. 
        '''

        #if normal is True, it is a macro or elem, if normal is False, it is a can_go_there
        self.normal = not can_go_there
        
        #if macro is true, it is a macro search, if False, it is a can_go_there or elem
        self.macro = macro

        #assign warehouse which contains boxes, worker, walls and targets
        self.warehouse = warehouse
        #assign taboo cells and grid size 
        self.taboo = taboo
        self.x_size = x_size
        self.y_size = y_size
       
        #if it is a can_go_there search, change the target and consider boxes as walls
        if (self.normal == False):
            self.warehouse.targets = new_target
            self.warehouse.walls = self.warehouse.walls + self.warehouse.boxes
            self.warehouse.boxes = []
            

        #this will be our initial state - gives the location of boxes and worker 
        self.initial = tuple(self.warehouse.boxes), self.warehouse.worker 

        #used for checking the time of calculating heuristics - purely for visualization
        self.total_time = 0
        
        #dictionaries which compare the strings with the movements
        self.dict_string = {'Left': (-1, 0), 'Down': (0, 1), 'Right': (1, 0), 'Up': (0, -1)}
        self.dict_action = {(-1, 0): 'Left', (0,1): 'Down', (1, 0): 'Right', (0, -1): 'Up'}
        

    def actions(self, state):
        ''' 
            Input: a tuple containing worker and box locations

            Output: a list of the possible actions. For an elementary or can_go_there puzzle, it is in the form of 'Left', 'Down', 
            'Right' or 'Up'. For a macro search, it is in the form of ((row, column), direction_to_push).

            Description: The actions function generates all the possible actions the worker can legally take. It uses several other functions
            to produce this information, including check_for_blockage and check_for_box. There is a distinction between macro and all 
            other puzzle types.
        '''

        if (self.macro == False):
            #checks where the worker can physically move
            blockages = set(self.check_for_blockage(state))
            #checks if there are any boxes that can be pushed
            boxes = set(self.check_for_box(state, True))

            #compares all actions with the actions possible due to blockages and boxes to generate final possible_actions
            action = set(['Left', 'Down', 'Right', 'Up'])
            remaining_actions = blockages.symmetric_difference(action)
            possible_actions = list(remaining_actions.union(boxes))
            
        else:
            #macro only checks for boxes that can be pushed
            possible_actions = set(self.check_for_box(state, True))
        
        return possible_actions
        
    
    
        
    def result(self, state, action):
        ''' 
            Input: tuple of the current state of boxes and worker and an action.

            Output: tuple with the new state of boxes and worker.

            Description: The result function gives the result of applying an action to the current state. Dictionaries are used to 
            account for the many different actions and movement types. The difference in action format between macro and other puzzle types was also
            accounted for.
        '''

        boxes = list(state[0])

        #macro and other sp puzzle types send states and actions in different ways
        if self.macro == False:
            #initialize variables
            worker = state[1]
            x = worker[0]
            y = worker[1]
            action_specific = action
            #new worker position using the string dictionary
            worker = (x+self.dict_string[action_specific][0], y+self.dict_string[action_specific][1])

        else:
            #initialize variables
            worker = action[0] 
            action_specific = action[1]
            x = worker[1]
            y = worker[0]
            worker = (x, y)

        

        #new box locations using string dictionary
        #if a worker's new position is on a boxes current position, the box will be pushed in the direction of movement
        #otherwise it says in the same position
        new_boxes = [(worker[0]+self.dict_string[action_specific][0], worker[1]+self.dict_string[action_specific][1]) if (worker[0], worker[1]) == (a,b) else (a,b) for (a,b) in boxes]
        
        new_state = tuple(new_boxes), worker
        return new_state



    def actions_legal(self, state):
        ''' 
            Input: a tuple containing worker and box locations

            Output: a list of the possible actions. 

            Description: The actions_legal checks for which actions are legal, meaning they are physically possible. It is very similar
            to the actions function, but disregards taboo cells.
        '''

        #check for physically impossible movements
        blockages = set(self.check_for_blockage(state))
        #check if any boxes can be pushed, disregards taboo cells
        boxes = set(self.check_for_box(state, False))

        #compares all actions with the actions possible due to blockages and boxes to generate final possible_actions
        action = set(['Left', 'Down', 'Right', 'Up'])
        remaining_actions = blockages.symmetric_difference(action)
        possible_actions = remaining_actions.union(boxes)
        
        return list(possible_actions)

            

    def goal_test(self, state):
        ''' 
            Input: a tuple containing worker and box locations

            Output: a boolean whether the goal has been met

            Description: The goal_test function checks whether the goal has been reached. The difference between elem, macro and
            can_go_there has been accounted for.
        '''

        #for all searches besides can_go_there, check if all boxes are a target
        if self.normal:
            return set(state[0]).issubset(set(self.warehouse.targets))
        #if can_go_there: check that the worker is on the target
        else:
            return set(list([state[1]])).issubset(set(list([self.warehouse.targets])))
    
    

    def h(self, state): 
        ''' 
            Input: a tuple containing worker and box locations

            Output: a heuristic value.

            Description: The heuristic function is used for the elem and macro puzzle types. It contains 3 different heuristics which
            be toggled by changing heuristic_num between 0, 1 and 2. The first heuristic (0) finds the total distance for each box to its
            closest target. The second heuristic (1) finds the distance to the two closest boxes for each target and the distance for the
            worker to the closest box that is not in a target already. The third heuristic (2) assigns each box to its own target and sums
            the distances.
        '''

        ''' CHANGE ME TO TRY OTHER HEURISTICS '''
        heuristic_num = 0;

        #timer to see how long heuristic calcs take
        s = time.time()

        #set variables used for heuristic calculation
        boxes = list(state.state[0])
        worker = state.state[1]
        boxes_copy = boxes
        num_of_goals = len(self.warehouse.targets)
        num_of_boxes = len(self.warehouse.boxes)


        if (heuristic_num == 0):
            #distance for each box to each target
            distances_box_to_target = [(abs(list(boxes_copy[idx])[0]-a)+ abs(list(boxes_copy[idx])[1]-b)) for idx in range(len(boxes_copy)) for (a,b) in self.warehouse.targets]
            #the minimum distance for each box to a target
            min_dist = [ max(distances_box_to_target[(ng*num_of_boxes):((ng*num_of_boxes)+num_of_boxes)]) for ng in range(num_of_goals) for nb in range(num_of_boxes) if (nb+(ng*num_of_boxes))%num_of_boxes==0]
            #the heuristic is the sum of the minimum distances
            heuristic = sum(min_dist)
            
        elif heuristic_num == 1:
            #distance for each target to each box
            distances_target_to_box = [((abs(list(boxes_copy[idx])[0]-a)+ abs(list(boxes_copy[idx])[1]-b)), self.warehouse.targets.index((a,b))*num_of_boxes + idx)  for (a,b) in self.warehouse.targets for idx in range(len(boxes_copy))]
            #the closest box to each target
            closest_distance = [ min(distances_target_to_box[(nb+(ng*num_of_boxes)): ((nb+(ng*num_of_boxes))+num_of_boxes)]) for nb in range(num_of_boxes) for ng in range(num_of_goals) if (nb+(ng*num_of_boxes))%num_of_boxes == 0 ]
            
            #remove the closest distances that were used above in closest_distance
            new_distances_target_to_box = [(a) if (a,b) not in closest_distance else () for (a,b) in distances_target_to_box]
            #find the closest box to each target in the new array - this is the second closest box to each target
            next_closest_distance = [ min(new_distances_target_to_box[(nb+(ng*num_of_boxes)): ((nb+(ng*num_of_boxes))+num_of_boxes)]) for nb in range(num_of_boxes) for ng in range(num_of_goals) if (nb+(ng*num_of_boxes))%num_of_boxes == 0 ]
            

            try:
                closest_distance, _ = zip(*closest_distance)
            except:
                closest_distance, _ = [], []
            
            #distance of the worker to the boxes
            worker_dist_to_box = [(abs(x-worker[0])+abs(y-worker[1])) for (x,y) in boxes if (x,y) not in self.warehouse.targets]

            #find minimum worker distance to box. use try in case all the boxes are in targets
            try:
                min_worker_dist = min(worker_dist_to_box)
            except:
                min_worker_dist = 0

            #sum worker to box and boxes to target distances to get final heuristic
            total_distance = sum(closest_distance) + sum(next_closest_distance)
            heuristic = total_distance + min_worker_dist
        
        else:

            minimum_index = []
            #for each target, find the distances for each box to that target, as long as that box has not been used for a target yet        
            for (a,b) in self.warehouse.targets:
                distances_box_to_target =  [abs(list(boxes[i])[0]-a) + abs(list(boxes[i])[1]-b) for i in range(num_of_boxes) if i not in minimum_index]
                #if the distance is the minimum, remember the box index
                minimum_index.append(distances_box_to_target.index(min(distances_box_to_target)))
            
            minimum_distances = [(abs(boxes[minimum_index[i]][0] - self.warehouse.targets[i][0]) + abs(boxes[minimum_index[i]][1] - self.warehouse.targets[i][1])) for i in range(num_of_goals)] 
            #heuristic is the sum of the distance of each box to its allocated closest target
            heuristic = sum(minimum_distances)
            
        #used as timer to check heuristic calculation speed
        e = time.time()
        self.total_time += e-s
        
        #heuristic is multiplied by 10 to account for complexity of Sokoban's and more closely approximate a greedy search (but not completely!)       
        return heuristic*100000
    
    
    
    def check_for_blockage(self, state):
        ''' 
            Input: a tuple containing worker and box locations

            Output: a list of the directions of any physical blockages.

            Description: The check_for_blockage function checks which directions the worker physically cannot move. For example, the worker
            cannot walk into a wall. Initially boxes are also considered as a physical blockage - this is later accounted for in the 
            check_for_box function. 
        '''

        worker = state[1]
        boxes = list(state[0])
        worker_x = worker[0]
        worker_y = worker[1]

        #blockages holds positions of objects blocking a possible movement
        blockages = self.warehouse.walls + boxes
        #check if any actions will cause the worker to hit a blockage
        blockage = [self.dict_action[(a,b)] for (a,b) in self.dict_action if (worker_x+a, worker_y+b) in blockages]
        
        return blockage 


    def check_for_box(self, state, include_taboo): 
        ''' 
            Input: a tuple containing worker and box locations and a boolean about accounting for taboo cells.

            Output: a list of the directions a box that can be pushed is located.

            Description: The check_for_box function returns the direction of all the movable boxes surrounding the worker. A box is 
            deemed movable in a direction is the adjacent cell to the box is not occupied by another box or wall and is not part of
            the taboo cell list. The function also checks for existing deadlock cells, or cells which will cause a impossible configuration
            if a box is moved into it. The deadlock that has been accounted for is when two boxes are side by side on a wall which is 
            not filled with taboo cells. The function accounts for the difference between macro and elem.
            
        '''
        
        worker = state[1]
        boxes = list(state[0])
        worker_x = worker[0]
        worker_y = worker[1]


        ''' Deadlock cells'''
        #check for cells that will cause a deadlock given the current box configuration
        #don't allow for two boxes side by side on a wall - horizontal or vertical walls
        #record the index of the box that has created this new taboo cell
        new_taboo_list = [((x, y+a), boxes.index((x,y))) for (x,y) in boxes for a in [-1, 1] for b in [-1, 1] if (x+b, y) in self.warehouse.walls and (x+b, y+a) in self.warehouse.walls and (x, y+a) not in self.taboo and (x, y+a) not in self.warehouse.targets and (x,y) not in self.warehouse.targets]    
        new_taboo_list = new_taboo_list + [((x+a, y), boxes.index((x,y))) for (x,y) in boxes for a in [-1, 1] for b in [-1, 1] if (x, y+b) in self.warehouse.walls and (x+a, y+b) in self.warehouse.walls and (x+a, y) not in self.taboo and (x+a, y) not in self.warehouse.targets and (x,y) not in self.warehouse.targets]    
        
        #zip into two arrays
        try:
            new_taboo, new_taboo_idx = zip(*new_taboo_list)
        except:
            new_taboo, new_taboo_idx = [], []

        
        #boxes can't be pushed into combined - consists of other boxes or walls or taboo cells depending on given parameters
        if include_taboo: 
            combined = boxes + self.warehouse.walls + self.taboo
        else: 
            combined = boxes + self.warehouse.walls

        if (self.macro):
            #create a test_warehouse for can_go_there
            test_warehouse = self.warehouse.copy()
            test_warehouse.worker = state[1]
            test_warehouse.boxes = list(state[0])

            #check if boxes can physically move to new location and if the worker can move to the correct position to push those boxes
            #also check the boxes new location isn't in the deadlock list and if it is -  check that if that deadlock cell was created by itself 
            boxes_list = [((y,x), self.dict_action[(a,b)]) for (x,y) in boxes for (a,b) in self.dict_action if (x+a, y+b) not in combined and can_go_there(test_warehouse, (y-b,x-a)) and ((x+a,y+b) not in new_taboo or ((x+a,y+b) in new_taboo and new_taboo_idx[new_taboo.index((x+a, y+b))] == boxes.index((x,y))))]
            
        else:
            #check if boxes can physically move to new location using the current worker location
            #also check the boxes new location isn't in the deadlock list and if it is -  check that if that deadlock cell was created by itself 
            boxes_list = [self.dict_action[(a,b)] for (a,b) in self.dict_action if (worker_x+a, worker_y+b) in boxes and (worker_x+a+a, worker_y+b+b) not in combined and ((worker_x+a+a, worker_y+b+b) not in new_taboo or ((worker_x+a+a, worker_y+b+b) in new_taboo and new_taboo_idx[new_taboo.index((worker_x+a+a, worker_y+b+b))] == boxes.index((worker_x+a, worker_y+b))))]
        
        return boxes_list
    
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def check_action_seq(warehouse, action_seq):
    ''' 
        Input: a warehouse object and a list of actions.

        Output: a string representation of the warehouse if the actions were all legal, or "Failure" if not.

        Description: The check_action_seq applies the list of actions to the warehouse object and returns the resulting warehouse 
        state in a string, if all actions were legal. It does this by creating a sokoban puzzle and manually applying each of the 
        actions in the action_seq. 
    '''
    
    SPuzzle = SokobanPuzzle(warehouse)
    

    worker = warehouse.worker
    boxes = warehouse.boxes
    current_state = tuple(boxes), worker
    
    #check that all of the actions are legal - doesn't push a box into a wall or into another box
    #update current state if the action was legal
    for action in action_seq:
        if action in SPuzzle.actions_legal(current_state):
            current_state = SPuzzle.result(current_state, action)
        else:
            return "Failure"
    
    #convert the resulting states to a warehouse string
    newState = convert_to_string(warehouse, current_state[0], current_state[1])

    return newState
    
    

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solve_sokoban_elem(warehouse):
    ''' 
        Input: a warehouse object

        Output: if a solution exists, a list of the actions to solve the puzzle. otherwise, a string "Impossible"

        Description: The solve_sokoban_elem function solves a warehouse puzzle using elementary actions. This means the worker only
        moves one space at a time. Taboo cells for the warehouse are first calculated and then the SokobanPuzzle object is created.
        The time it takes to solve is recorded and printed. An astar graph search is used to solve the problem. This is justified in
        the report.
    '''

    #find the taboo_cells for the warehouse and the grid size of the warehouse
    warehouse_string = taboo_cells(warehouse)
    X,Y = zip(*warehouse.walls)
    x_size, y_size = 1+max(X), 1+max(Y)
    #find taboo cells by decoding the warehouse string
    taboo = [(x,y) for x in range(x_size) for y in range(y_size) if warehouse_string[x+y+((x_size)*y)]=="X"]
    
    #create puzzle and pass in taboo cells, x_size and y_size of the warehouse
    sp = SokobanPuzzle(warehouse, x_size = x_size, y_size = y_size, taboo = taboo)

    s_time = time.time()
    
    #find the goal using astar
    #goal_node = search.astar_graph_search(sp)
    goal_node = search.breadth_first_graph_search(sp)
    
    e_time = time.time()

    #for visualization and testing
    print "time taken is ", e_time-s_time
    print "time calculating heuristics is: ", sp.total_time

    #return solution if there is one 
    if goal_node == None:
        return ['Impossible']
    else:
        print "Number of steps: ", len(goal_node.solution())
        return goal_node.solution()


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def can_go_there(warehouse, dst):
    ''' 
        Input: a warehouse object and a tuple of the destination

        Output: if a solution exists, True. Otherwise, False

        Description: The can_go_there function checks if the worker can move to the target (dst) without pushing any boxes. This is 
        checked by classing boxes as walls and creating a SokobanPuzzle object. If this can be done, the function returns True. Otherwise,
        it returns false. Taboo cells are disregarded as boxes are not being manipulated in this function.  A breadth first graph search
        is used, this is justified in the report.
    '''
    #find grid size of the warehouse
    X,Y = zip(*warehouse.walls)
    x_size, y_size = 1+max(X), 1+max(Y)
  
    #if the target is not in the grid size, immediately return false, otherwise set target to dst
    if dst[1] >= 0 and dst[1] <= x_size and dst[0] >= 0 and dst[0] <= y_size:
        target = (dst[1], dst[0])
    else:
        return False
   
    #create sp puzzle, set parameter to indicate it is a can_go_there puzzle, set target, x size and ysize
    sp_go_there = SokobanPuzzle(warehouse, can_go_there = True, new_target= target, x_size = x_size, y_size = y_size)
       
    #use a depth_first_search
    goal_go_there = search.depth_first_graph_search(sp_go_there)

    #return true if the worker can go there
    if goal_go_there == None:
        return False
    else:
        return True
    


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solve_sokoban_macro(warehouse):
    ''' 
        Input: a warehouse object

        Output: if a solution exists, a list of the actions to solve the puzzle. Otherwise, a string "Impossible"

        Description: The solve_sokoban_macro function solves a warehouse puzzle using macro actions. This means that the actions specify
        which coordinate (row, column) the worker should move to and which direction it should push in. Taboo cells for the warehouse 
        are first calculated and then the SokobanPuzzle object is created. The time it takes to solve is recorded and printed. An astar graph search is used to solve the problem. This is justified in
        the report.
    '''

    #find the taboo_cells for the warehouse and the grid size of the warehouse
    warehouse_string = taboo_cells(warehouse)
    X,Y = zip(*warehouse.walls)
    x_size, y_size = 1+max(X), 1+max(Y)
    #find taboo cells by decoding the warehouse string
    taboo = [(x,y) for x in range(x_size) for y in range(y_size) if warehouse_string[x+y+((x_size)*y)]=="X"]

    #create puzzle, send in parameters to indicate it is a macro search and the taboo cells, x size and y size
    sp_macro = SokobanPuzzle(warehouse, macro = True, taboo = taboo, x_size = x_size, y_size = y_size)

    s_time = time.time()
    
    #if problem already solved, return []
    if set(warehouse.boxes).issubset(set(warehouse.targets)):
        return []
    
    #use astar search to find goal
    goal_node_macro = search.astar_graph_search(sp_macro)

    e_time = time.time()

    #for visualization and testing
    print "time taken is ", e_time-s_time
    print "time calculating heuristics is: ", sp_macro.total_time
    
    #return solution if a goal was found
    if goal_node_macro == None:
        return ['Impossible']
    else:
        print "Number of steps: ", len(goal_node_macro.solution())
        return goal_node_macro.solution()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if __name__ == "__main__":
    
    wh = sokoban.Warehouse()
    wh.read_warehouse_file("./warehouses/warehouse_57.txt")
    print(wh)
    
    wh_elem = wh.copy()
    wh_macro = wh.copy()
    # print(wh)
    # #print(check_action_seq(wh, ['Left', 'Up', 'Up', 'Up', 'Up', 'Right', 'Right', 'Up', 'Right', 'Right', 'Down', 'Right', 'Up', 'Left', 'Left', 'Left', 'Down', 'Left', 'Left', 'Up', 'Right', 'Down', 'Right', 'Right', 'Up', 'Right', 'Right', 'Down', 'Left', 'Right', 'Up', 'Right', 'Right', 'Down', 'Left', 'Up', 'Left', 'Left', 'Down', 'Left', 'Right', 'Up', 'Right', 'Right', 'Down', 'Left', 'Up', 'Left', 'Left', 'Down', 'Left', 'Left', 'Up', 'Right', 'Left', 'Left', 'Down', 'Down', 'Down', 'Down', 'Left', 'Down', 'Right', 'Right', 'Right', 'Left', 'Left', 'Up', 'Up', 'Up', 'Up', 'Right', 'Right', 'Right', 'Right', 'Up', 'Right', 'Right', 'Down', 'Left', 'Left', 'Left', 'Left', 'Left', 'Up', 'Left', 'Down', 'Down', 'Down', 'Down', 'Left', 'Down', 'Right', 'Right', 'Left', 'Down', 'Down', 'Right', 'Right', 'Right', 'Right', 'Up', 'Up', 'Up', 'Up', 'Left', 'Left', 'Down', 'Down', 'Down', 'Up', 'Left', 'Left', 'Down', 'Down', 'Right', 'Right', 'Up', 'Up', 'Left', 'Left', 'Up', 'Up', 'Up', 'Up', 'Right', 'Right', 'Up', 'Right', 'Right', 'Right', 'Down', 'Right', 'Right', 'Up', 'Up', 'Left', 'Down', 'Right', 'Down', 'Left', 'Left', 'Left', 'Up', 'Left', 'Left', 'Down', 'Left', 'Left', 'Down', 'Down', 'Down', 'Down', 'Right', 'Right', 'Up', 'Up', 'Right', 'Right', 'Down', 'Down', 'Down', 'Down', 'Left', 'Left', 'Up', 'Up', 'Right', 'Left', 'Down', 'Down', 'Right', 'Right', 'Up', 'Up', 'Up', 'Right', 'Up', 'Left', 'Down', 'Down', 'Left', 'Left', 'Left', 'Left', 'Down', 'Down', 'Right', 'Right', 'Up', 'Up', 'Left', 'Left', 'Up', 'Up', 'Up', 'Up', 'Right', 'Right', 'Up', 'Right', 'Right', 'Down', 'Left', 'Left', 'Left', 'Up', 'Left', 'Down', 'Down', 'Down', 'Down', 'Left', 'Down', 'Right', 'Right', 'Right']))

    print(solve_sokoban_elem(wh_elem))
    print(solve_sokoban_macro(wh_macro))
    
  