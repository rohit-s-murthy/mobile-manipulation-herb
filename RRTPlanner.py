import numpy
from RRTTree import RRTTree


class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        



        # OUR TODOs:
        # 1) Path Shortening with linear interpolation in between two nodes
        #
        # 2) May have to think over the Extend function if we want to change this or not. Extend function currently is 
        #    essentially taking a step size of infinity i.e. reach q_rand as much as possible with a specific resolution 
        #    of 'max_step_size'. As a result, potentially not many intermediate nodes on an edge between q_rand and q_near.
        #
        # 3) Carry out more testings to validate the implementation. If collision with obstacle observed at corners or whatever, try
        #    decreasing 'max_step_size' in Extend function and test. 


        self.q_id_parent_dict = {}
        self.q_id_config_dict = {}
        q_goal_id = -1
        q_start_id = 0
        goal_found = False

        NUM_ITERATIONS = 5000;

        self.env = self.planning_env.robot.GetEnv()        
        with self.env:
            with self.planning_env.robot.CreateRobotStateSaver():

                for k in range(0, NUM_ITERATIONS):
                    print "Iteration: " + str(k)

                    q_rand = self.planning_env.GenerateRandomConfiguration()

                    #Picking q_near in the tree
                    q_near_id, q_near = tree.GetNearestVertex(q_rand)

                    q_new = self.planning_env.Extend(q_near, q_rand)
                    if(q_new is not None):

                        q_new_id = tree.AddVertex(q_new)
                        self.q_id_config_dict[q_new_id] = q_new

                        tree.AddEdge(q_new_id, q_near_id)
                        self.q_id_parent_dict[q_new_id] = q_near_id
                        # self.planning_env.PlotEdge(q_near, q_new)

                        if(self.planning_env.ComputeDistance(q_new, goal_config) < epsilon):
                            q_goal_id = q_new_id
                            goal_found = True
                            break;


                if(not goal_found):
                    print "Goal not found in " + str(NUM_ITERATIONS) + "iterations."
                    return plan

                else:
                    print "Path Found!"
                    plan = self.getPlan(q_goal_id, start_config)
                    # print plan
                    num_vertices = len(plan)
                    print "Number of Vertices: " + str(num_vertices) + " nodes."

                
                return plan, num_vertices


    def getPlan(self, goal_id, start_config):
        planned_path = []
        q_current_id = goal_id

        q_current_config = self.q_id_config_dict[q_current_id]       
        planned_path.append(q_current_config)

        q_parent_id = self.q_id_parent_dict[q_current_id]
        q_current_id = q_parent_id


        while(q_current_id is not 0):
            q_current_config = self.q_id_config_dict[q_current_id]
            planned_path.append(q_current_config)

            q_parent_id = self.q_id_parent_dict[q_current_id]
            q_current_id = q_parent_id

        planned_path.append(start_config)

        planned_path.reverse()

        return planned_path