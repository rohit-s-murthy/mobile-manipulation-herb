import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
from IPython import embed

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotAction(self, action):

        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])  

        xpoints = [config[0] for config in action.footprint]
        ypoints = [config[1] for config in action.footprint]
        print('xpoints: {}'.format(xpoints))
        print('ypoints: {}'.format(ypoints))
        print('control: {}, {}, {}'.format(action.control.ul, action.control.ur, action.control.dt))
        pl.plot(xpoints, ypoints, 'kx-')
                     
        pl.ion()
        pl.show()      

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        # dur = 1.0
        # dur_min = 2.0
        # dur_max = 4.0
        # omega_res = 3
        # omega_max = 1.0
        # self.controls = []

        # for j in range(omega_res):
        #     for k in range(omega_res):
        #         # for i in range(dur_res):

        #             # dur = (dur_max - dur_min) * i / (dur_res - 1) + dur_min
        #             omega_l = omega_max * j / (omega_res - 1)
        #             omega_r = omega_max * k / (omega_res - 1)

        #             self.controls.append(Control( omega_l,  omega_r, dur))
        #             self.controls.append(Control(-omega_l,  omega_r, dur))
        #             self.controls.append(Control( omega_l, -omega_r, dur))
        #             self.controls.append(Control(-omega_l, -omega_r, dur))

        # wc = [0., 0., 0.]
        # grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # # Iterate through each possible starting orientation
        # for idx in range(int(self.discrete_env.num_cells[2])):

        #     self.actions[idx] = []
        #     grid_coordinate[2] = idx
        #     start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

        #     for c in self.controls:
        #         ftpt = self.GenerateFootprintFromControl(start_config, c)
        #         self.actions[idx].append(Action(c, ftpt))
        #         # if c.ul > 0.6 and c.ur!=0 and c.dt!=0:
        #         #     self.PlotAction(Action(c,ftpt))
        #         #     embed()

        #     # print(len(self.actions[idx]))
        #     # self.PlotActionFootprints(idx)

        turn_speed1 = 1.5
        turn_speed2 = 0.75
        forward_speed = 1.0
        dur = 1.0

        self.controls = []

        self.controls.append(Control(forward_speed, forward_speed, dur))
        self.controls.append(Control(-forward_speed, -forward_speed, dur))
        self.controls.append(Control(turn_speed1, -turn_speed1, dur))
        self.controls.append(Control(-turn_speed1, turn_speed1, dur))
        self.controls.append(Control(turn_speed2, -turn_speed2, dur))
        self.controls.append(Control(-turn_speed2, turn_speed2, dur))

        self.controls.append(Control(turn_speed1, turn_speed2, dur))
        self.controls.append(Control(turn_speed2, turn_speed1, dur))
        self.controls.append(Control(-turn_speed1, -turn_speed2, dur))
        self.controls.append(Control(-turn_speed2, -turn_speed1, dur))

        # self.controls.append(Control(2.5, 2.5, 1.0))
        # self.controls.append(Control(-2.5, -2.5, 1.0))
        
        # self.controls.append(Control(1.0, 1.0, 1.0))
        # self.controls.append(Control(-1.0, -1.0, 1.0))

        # self.controls.append(Control(5*numpy.pi/16, -5*numpy.pi/16, 1.0))
        # self.controls.append(Control(-5*numpy.pi/16, 5*numpy.pi/16, 1.0))

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            for c in self.controls:
                ftpt = self.GenerateFootprintFromControl(start_config, c)
                # print('ftpt: {}\n'.format(ftpt))
                self.actions[idx].append(Action(c, ftpt))
                # if c.ul > 0.6 and c.ur!=0 and c.dt!=0:
                # self.PlotAction(Action(c,ftpt))
                # embed()
            
            # self.PlotActionFootprints(idx)
        # embed()


    def checkSucc(self, config):

        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]
        # pdb.set_trace()
        # config = config.tolist()
        # self.robot.SetActiveDOFValues(config)

        robot_pose[0][3] = config[0]
        robot_pose[1][3] = config[1]
        robot_pose[0][0] = numpy.cos(config[2])
        robot_pose[0][1] = numpy.sin(config[2])
        robot_pose[0][2] = 0
        robot_pose[1][0] = -numpy.sin(config[2])
        robot_pose[1][1] = numpy.cos(config[2])
        robot_pose[2][0] = 0
        robot_pose[2][1] = 0
        robot_pose[2][2] = 1
        self.robot.SetTransform(robot_pose)

        if self.env.CheckCollision(self.robot, table):
            return False

        for i in range(self.discrete_env.dimension):
            if not(self.lower_limits[i] <= config[i] <= self.upper_limits[i]):
                return False

        return True

    def GetSuccessors(self, node_id):

        successors = []
        successors_config = []
        action_valid = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        grid_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        config_coord = self.discrete_env.NodeIdToConfiguration(node_id)
        actions = self.actions[grid_coord[2]]   # actions: List of Action objects
        # actions = self.ConstructActions(grid_coord)
        num_actions = len(actions)

        # Testing
        # config_coord = numpy.array([0., 0., 0.])
        # print(config_coord)

        for action in actions:
            ftpt = action.footprint     # ftpt: List of footprint configs
            ftpt += config_coord
            # embed()
            
            valid = False
            
            for pt in ftpt:
                if not(self.checkSucc(pt)):
                    valid = False
                    break
                else:
                    valid = True
                    
            if valid:
                successors_config.append(pt)
                action_valid.append(action)
                # print('Successor: {}, action.control.ul: {}, action.control.ur: {}, footprint: {}'.format(pt, action.control.ul, action.control.ur, action.footprint))
                # raw_input()        

        successors = [self.discrete_env.ConfigurationToNodeId(x) for x in successors_config]

        return successors, action_valid

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        diff = end_config - start_config
        weight = numpy.array([10, 10, 1])
        diff = numpy.dot(diff, weight)
        dist = numpy.linalg.norm(diff)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        diff = end_config - start_config
        weight = numpy.array([10, 10, 1])
        diff = numpy.dot(diff, weight)
        cost = numpy.linalg.norm(diff)
        
        return 100 * cost

