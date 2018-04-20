import numpy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        print "In AStarPlanner"

        #--------------------------------------------------------------------------------

        # AM TODO: Check for validity of start_config & goal_config
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        print('start_config: {}, goal_config: {}'.format(start_config, goal_config))
        raw_input()
        # self.robot.SetActiveDOFValues(goal_config)

        robot_start_state = GraphState()
        robot_start_state.gval_ = 0
        robot_start_state.hval_ = self.planning_env.ComputeHeuristicCost(start_id, goal_id)
        robot_start_state.fval_ = robot_start_state.gval_ + robot_start_state.hval_
        robot_start_state.id_ = start_id
        robot_start_state.pred_id_ = -1

        robot_goal_state = GraphState()
        robot_goal_state.gval_ = float("inf")
        robot_goal_state.hval_ = 0
        robot_goal_state.fval_ = robot_goal_state.gval_ + robot_goal_state.hval_
        robot_goal_state.id_ = goal_id
        robot_goal_state.pred_id_ = -1

        print ("Running AStarPlanner for start state ID %d , goal state ID %d" % (start_id, goal_id) )

        graph_manager = GraphManager(robot_start_state, robot_goal_state, self.planning_env)

        while ( not (graph_manager.is_goal_state_expanded_) and (not (len(graph_manager.open_queue_)==0)) ):
            current_min_f_state = graph_manager.popMinFValStateFromQueue()
            graph_manager.expandGraphState(current_min_f_state)

        path_ids = []
        path_actions = []
        if (graph_manager.is_goal_state_expanded_):
            path_ids = graph_manager.getPathIDsToGoal()
            path_actions = graph_manager.getPathActionList()

        else:
            print "Path Not Found!"
            raise Exception('FAILURE: Path Not Found!')

        path_ids.reverse()
        path_actions.reverse()

        if self.visualize:
            self.planning_env.InitializePlot(goal_config)
            prev_config = start_config

        for i,state_id in enumerate(path_ids):
            state_config = self.planning_env.discrete_env.NodeIdToConfiguration(state_id)
            plan.append(state_config)

            if self.visualize:
                if state_config[0]!=start_config[0] and state_config[1]!=start_config[1]:
                    self.planning_env.PlotEdge(prev_config, state_config)
                    prev_config = state_config


        path_length = 0
        for i,state_id in enumerate(path_ids):
            if not(i == len(path_ids)):
                path_length = path_length + self.planning_env.ComputeDistance(state_id, state_id + 1)

        print('Path Length: {}'.format(path_length))
        # plan.append(start_config)
        # plan.append(goal_config)

        return path_actions


class GraphState(object):

    # MEMBER VARIABLES: 
    # int id_
    # int pred_id_
    # float fval_
    # float hval_
    # float gval_

    def __init__(self, id=-1, hval=-1, gval=float("inf")):
        self.id_ = id
        self.pred_id_ = -1
        self.fval_ = gval + hval
        self.hval_ = hval
        self.gval_ = gval
        self.action_ = None

    def printInfo(self, planning_env):
        if(self.pred_id_ >= 0):
            current_state_config = planning_env.discrete_env.NodeIdToGridCoord(self.id_)
            parent_state_config = planning_env.discrete_env.NodeIdToGridCoord(self.pred_id_)
            print ("State: (%d, %d, %d) , Pred: (%d, %d, %d) , F-Value: %f , H-Value: %f , G-Value: %f" % (current_state_config[0], current_state_config[1],current_state_config[2],  parent_state_config[0], parent_state_config[1], parent_state_config[2], self.fval_, self.hval_, self.gval_))

        # print ("State ID: %d , Pred ID: %d , F-Value: %f , H-Value: %f , G-Value: %f" % (self.id_, self.pred_id_, self.fval_, self.hval_, self.gval_) )

    def setAction(self, action):
        self.action_ = action

class GraphManager(object):

    # MEMBER VARIABLES:
    # GraphState        start_state_
    # GraphState        goal_state_
    # bool              is_goal_state_expanded_
    # float             epsilon_
    # float             prim_cost_
    # list              open_queue_
    # list              closed_list_

    # bool              debug_

    # planning_env_

    def __init__(self, start_state, goal_state, planning_env):
        self.start_state_ = start_state
        self.goal_state_ = goal_state
        self.is_goal_state_expanded_ = False
        self.epsilon_ = 0.1
        self.prim_cost_ = 0.1
        self.open_queue_ = []
        self.closed_list_ = []
        self.planning_env_ = planning_env
        self.num_nodes_expanded = 0
        self.num_nodes_expanded_ = 0
        self.debug_ = False

        self.insertStateInOpenQueue(start_state)

        if(self.debug_):
            print("GraphManager initialized.")

    
    def insertStateInOpenQueue(self, g_state):
        is_state_present = any(x.id_ == g_state.id_ for x in self.open_queue_)

        if(is_state_present):
            index = [x.id_ for x in self.open_queue_].index(g_state.id_)
            self.open_queue_[index] = g_state
            self.open_queue_.sort(key = lambda x: x.fval_)

        else:
            self.open_queue_.append(g_state)
            self.open_queue_.sort(key = lambda x: x.fval_)

        self.num_nodes_expanded = self.num_nodes_expanded + 1

    def getHeuristicValue(self, g_state):
        heuristic_value = self.planning_env_.ComputeHeuristicCost(g_state.id_, self.goal_state_.id_)
        return heuristic_value
        # return 0

    def popMinFValStateFromQueue(self):
        self.open_queue_.sort(key = lambda x: x.fval_)
        g_state = self.open_queue_.pop(0)
        return g_state


    def isStateClosed(self, g_state):
        is_state_closed = any(x.id_ == g_state.id_ for x in self.closed_list_)

        if(is_state_closed):
            index = [x.id_ for x in self.closed_list_].index(g_state.id_)
            closed_state = self.closed_list_[index]
            if(closed_state.id_ != g_state.id_):
                raise Exception('ERROR: Different states are considered closed - isStateClosed')

        return is_state_closed


    def isPresentInOpen(self, g_state):
        is_state_present = any(x.id_ == g_state.id_ for x in self.open_queue_)

        return is_state_present


    def getSimilarStateFromOpen(self, g_state):
        index = [x.id_ for x in self.open_queue_].index(g_state.id_)
        g_state_in_open = self.open_queue_[index]

        return g_state_in_open


    def expandGraphState(self, current_graph_state):
        if( self.isStateClosed(current_graph_state) ):
            raise Exception('ERROR: Expanding an already closed state in expandGraphState - expandGraphState')

        if (self.debug_):
            current_graph_state.printInfo(self.planning_env_)
            raw_input()

        self.num_nodes_expanded_ = self.num_nodes_expanded_ + 1
        self.closed_list_.append(current_graph_state)

        current_graph_state_config = numpy.array(self.planning_env_.discrete_env.NodeIdToConfiguration(current_graph_state.id_))
        goal_graph_state_config = numpy.array(self.planning_env_.discrete_env.NodeIdToConfiguration(self.goal_state_.id_))
        dist_to_goal = numpy.linalg.norm(current_graph_state_config - goal_graph_state_config)

        # print current_graph_state_config
        # print goal_graph_state_config
        print "Distance from current state to goal: " + str(dist_to_goal) + "\n"

        if(dist_to_goal < self.epsilon_):
            self.is_goal_state_expanded_ = True
            print ("Goal State found! \n")
            print('Goal State: {}'.format(self.planning_env_.discrete_env.NodeIdToConfiguration(current_graph_state.id_)))
            print('Distance: {}'.format(self.planning_env_.ComputeDistance(current_graph_state.id_, self.goal_state_.id_)))
            self.goal_state_ = current_graph_state
            print('No. of nodes expanded: {}'.format(self.num_nodes_expanded))
            print('No. of nodes expanded 2: {}'.format(self.num_nodes_expanded_))


        successor_states_id, successor_states_actions = self.planning_env_.GetSuccessors(current_graph_state.id_)

        for i, successor_state_id in enumerate(successor_states_id):
            if not(self.planning_env_.checkSucc(self.planning_env_.discrete_env.NodeIdToConfiguration(successor_state_id))):
                print('ERROR: Collision')
                raw_input()

            successor_state = GraphState()
            successor_state.action_ = successor_states_actions[i]
            successor_state.id_ = successor_state_id
            successor_state.pred_id_ = -1
            successor_state.hval_ = self.getHeuristicValue(successor_state)

            if ( not (self.isStateClosed(successor_state)) ):
                similar_state_in_open = GraphState()

                if( self.isPresentInOpen(successor_state) ):
                    similar_state_in_open = self.getSimilarStateFromOpen(successor_state)

                else:
                    similar_state_in_open = successor_state

                self.prim_cost_ = self.planning_env_.ComputeDistance(successor_state_id, current_graph_state.id_)

                if (similar_state_in_open.gval_ > current_graph_state.gval_ + self.prim_cost_):
                    successor_state.gval_ = current_graph_state.gval_ + self.prim_cost_
                    successor_state.fval_ = successor_state.gval_ + successor_state.hval_
                    successor_state.pred_id_ = current_graph_state.id_
                    self.insertStateInOpenQueue(successor_state)


    def getPathIDsToGoal(self):
        path_ids = [self.goal_state_.id_]

        pred_id = self.goal_state_.pred_id_

        while (pred_id != self.start_state_.id_):

            index = [x.id_ for x in self.closed_list_].index(pred_id)
            g_state = self.closed_list_[index]
            path_ids.append(g_state.id_)
            pred_id = g_state.pred_id_

        path_ids.append(self.start_state_.id_)

        if(pred_id != self.start_state_.id_):
            raise Exception('ERROR: Start State not reached from back tracking - getPathIDsToGoal')

        return path_ids

    def getPathActionList(self):
        path_ids = [self.goal_state_.id_]
        path_actions = [self.goal_state_.action_]

        pred_id = self.goal_state_.pred_id_

        while (pred_id != self.start_state_.id_):

            index = [x.id_ for x in self.closed_list_].index(pred_id)
            g_state = self.closed_list_[index]
            path_ids.append(g_state.id_)
            path_actions.append(g_state.action_)
            pred_id = g_state.pred_id_

        path_ids.append(self.start_state_.id_)

        if(pred_id != self.start_state_.id_):
            raise Exception('ERROR: Start State not reached from back tracking - getPathIDsToGoal')


        return path_actions













