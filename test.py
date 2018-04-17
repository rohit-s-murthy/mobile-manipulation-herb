import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from HerbEnvironment import HerbEnvironment
from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from GraspPlanner import GraspPlanner
from AStarPlanner import AStarPlanner

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
openravepy.misc.InitOpenRAVELogging()

# openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

# Create an environment
env = openravepy.Environment()
env.SetViewer('qtcoin')
env.GetViewer()

# Load HERB into it
robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
env.Add(robot)
        
theta = -numpy.pi/4.
robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -1.25],
                          [numpy.sin(theta),  numpy.cos(theta), 0,  0.82],
                          [0.              ,  0.              , 1,  0.  ],
                          [0.              ,  0.              , 0,  1.  ]])
robot.SetTransform(robot_pose)

right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
right_manip = robot.GetManipulator('right_wam')
robot.SetActiveDOFs(right_manip.GetArmIndices())
robot.SetActiveDOFValues(right_relaxed)
        
left_manip = robot.GetManipulator('left_wam')
robot.SetActiveDOFs(left_manip.GetArmIndices())
robot.SetActiveDOFValues(left_relaxed)

# robot.controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')
# robot.ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=openravepy.IkParameterization.Type.Transform6D)
# if not robot.ikmodel.load():
#     robot.ikmodel.autogenerate()

resolution = [0.1, 0.1, 0.4]
herb_base = SimpleRobot(env, robot)
base_env = SimpleEnvironment(herb_base, resolution)

base_env.ConstructActions()
print base_env.controls
print len(base_env.controls)
