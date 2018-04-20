import numpy, operator
from RRTPlanner import RRTTree
from IPython import embed

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space




        # OUR TODOs:
        # 1) Intermediate nodes are not being added to the tree when making connection between the node of one tree with the other tree.
        #
        # 2) Path Shortening with linear interpolation in between two nodes
        #
        # 3) May have to think over the Extend function if we want to change this or not. Extend function currently is 
        #    essentially taking a step size of infinity i.e. reach q_rand as much as possible with a specific resolution 
        #    of 'max_step_size'. As a result, potentially not many intermediate nodes on an edge between q_rand and q_near.
        #
        # 4) Carry out more testings to validate the implementation. If collision with obstacle observed at corners or whatever, try
        #    decreasing 'max_step_size' in Extend function and test. 

        self.q_id_parent_dict_ftree = {}
        self.q_id_config_dict_ftree = {}

        self.q_id_parent_dict_rtree = {}
        self.q_id_config_dict_rtree = {}

        q_goal_id = -1
        q_start_id = 0
        goal_found = False
        swap_trees = False

        NUM_ITERATIONS = 5000;

        self.env = self.planning_env.robot.GetEnv()        
        with self.env:
            with self.planning_env.robot.CreateRobotStateSaver():

                for k in range(0, NUM_ITERATIONS):
                    print "Iteration: " + str(k)

                    q_rand = self.planning_env.GenerateRandomConfiguration()

                    if(swap_trees == False):

                        #Picking q_near in the tree
                        q_near_id, q_near = ftree.GetNearestVertex(q_rand)

                        q_new = self.planning_env.Extend(q_near, q_rand)

                        if(q_new is not None):
                            q_new_id = ftree.AddVertex(q_new)
                            self.q_id_config_dict_ftree[q_new_id] = q_new

                            ftree.AddEdge(q_new_id, q_near_id)
                            self.q_id_parent_dict_ftree[q_new_id] = q_near_id
                            # self.planning_env.PlotEdge(q_near, q_new)

                            #Function 'Connect'
                            q_connect = q_new
                            q_connect_id = q_new_id

                            q_near_id_connect, q_near_connect = rtree.GetNearestVertex(q_connect)
                            q_new_connect = self.planning_env.Extend(q_near_connect, q_connect)

                            if(q_new_connect is not None):

                                q_new_id_connect = rtree.AddVertex(q_new_connect)
                                self.q_id_config_dict_rtree[q_new_id_connect] = q_new_connect

                                rtree.AddEdge(q_new_id_connect, q_near_id_connect)
                                self.q_id_parent_dict_rtree[q_new_id_connect] = q_near_id_connect
                                # self.planning_env.PlotEdge(q_near, q_new)

                                if(self.planning_env.ComputeDistance(q_new_connect, q_connect) < epsilon):
                                    print "Path Found!"
                                    goal_found = True
                                    # self.planning_env.PlotEdge(q_connect, q_near_connect)

                                    q_ftree_connect_id = q_connect_id
                                    q_rtree_connect_id = q_near_id_connect
                                    plan = self.getPlan(q_ftree_connect_id, start_config, q_rtree_connect_id, goal_config)
                                    break;

                                else:
                                    swap_trees = True

                            else:
                                swap_trees = True

                        else:
                            swap_trees = True


                    else:
                        #Picking q_near in the tree
                        q_near_id, q_near = rtree.GetNearestVertex(q_rand)

                        q_new = self.planning_env.Extend(q_near, q_rand)

                        if(q_new is not None):
                            q_new_id = rtree.AddVertex(q_new)
                            self.q_id_config_dict_rtree[q_new_id] = q_new

                            rtree.AddEdge(q_new_id, q_near_id)
                            self.q_id_parent_dict_rtree[q_new_id] = q_near_id
                            # self.planning_env.PlotEdge(q_near, q_new)

                            #Function 'Connect'
                            q_connect = q_new
                            q_connect_id = q_new_id

                            q_near_id_connect, q_near_connect = ftree.GetNearestVertex(q_connect)
                            q_new_connect = self.planning_env.Extend(q_near_connect, q_connect)

                            if(q_new_connect is not None):

                                q_new_id_connect = ftree.AddVertex(q_new_connect)
                                self.q_id_config_dict_ftree[q_new_id_connect] = q_new_connect

                                ftree.AddEdge(q_new_id_connect, q_near_id_connect)
                                self.q_id_parent_dict_ftree[q_new_id_connect] = q_near_id_connect
                                # self.planning_env.PlotEdge(q_near, q_new)

                                if(self.planning_env.ComputeDistance(q_new_connect, q_connect) < epsilon):
                                    print "Path Found!"
                                    goal_found = True
                                    # self.planning_env.PlotEdge(q_connect, q_near_connect)

                                    q_rtree_connect_id = q_connect_id
                                    q_ftree_connect_id = q_near_id_connect
                                    plan = self.getPlan(q_ftree_connect_id, start_config, q_rtree_connect_id, goal_config)
                                    break;

                                else:
                                    swap_trees = False

                            else:
                                swap_trees = False

                        else:
                            swap_trees = False


                if(not goal_found):
                    print "Goal not found in " + str(NUM_ITERATIONS) + "iterations."
                else:
                    num_vertices = len(plan)
                    print "Number of Vertices: " + str(num_vertices) + " nodes."

                print('plan in RRTConnect: {}'.format(plan))
                print('plan list: {}'.format(plan[0]))

                return plan, num_vertices



    def getPlan(self, q_ftree_connect_id, start_config, q_rtree_connect_id, goal_config):
        # print q_ftree_connect_id
        # print start_config
        # print q_rtree_connect_id
        # print goal_config

        planned_path = []

        #Tracking ftree
        if(q_ftree_connect_id is not 0):
            q_current_id = q_ftree_connect_id
            q_current_config = self.q_id_config_dict_ftree[q_current_id]       
            planned_path.append(q_current_config)

            q_parent_id = self.q_id_parent_dict_ftree[q_current_id]
            q_current_id = q_parent_id

            while(q_current_id is not 0):
                q_current_config = self.q_id_config_dict_ftree[q_current_id]
                planned_path.append(q_current_config)

                q_parent_id = self.q_id_parent_dict_ftree[q_current_id]
                q_current_id = q_parent_id

        planned_path.append(start_config)

        #Tracking rtree
        if(q_rtree_connect_id is not 0):
            q_current_id = q_rtree_connect_id
            q_current_config = self.q_id_config_dict_rtree[q_current_id]       
            planned_path.insert(0, q_current_config)

            q_parent_id = self.q_id_parent_dict_rtree[q_current_id]
            q_current_id = q_parent_id

            while(q_current_id is not 0):
                q_current_config = self.q_id_config_dict_rtree[q_current_id]
                planned_path.insert(0, q_current_config)

                q_parent_id = self.q_id_parent_dict_rtree[q_current_id]
                q_current_id = q_parent_id

        planned_path.insert(0, goal_config)

        planned_path.reverse()

        return planned_path
