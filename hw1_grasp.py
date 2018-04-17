#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy
from scipy.spatial import ConvexHull
from operator import add
# OpenRAVE
import openravepy


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()



class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    self.order_grasps()

    #order grasps with noise
    # self.order_grasps_noisy()


  # the usual initialization for openrave
  def openrave_init(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW1 Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

    self.grasp_count = 0

  # problem specific initialization - load target and grasp module
  def problem_init(self):
    # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

    #change the location so it's not under the robot
    T = self.target_kinbody.GetTransform()
    T[0:3,3] += np.array([0.5, 0.5, 0.5])
    self.target_kinbody.SetTransform(T)
    self.env.AddKinBody(self.target_kinbody)

    # create a grasping module
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)
    
    # if you want to set options, e.g. friction
    options = openravepy.options
    options.friction = 0.1
    if not self.gmodel.load():
      self.gmodel.autogenerate(options)

    self.graspindices = self.gmodel.graspindices
    self.grasps = self.gmodel.grasps

  
  # order the grasps - call eval grasp on each, set the 'performance' index, and sort
  def order_grasps(self):
    self.grasps_ordered = self.grasps.copy()
    scores = []
    for i, grasp in enumerate(self.grasps_ordered):
      scores.insert(i, self.eval_grasp(grasp))


    minsigma_scores = [item[0] for item in scores]                # Extracting out scores for metric 'Minimum Singular Value'
    isotropy_index_scores = [item[1] for item in scores]          # Extracting out scores for metric 'Isotropy Index'
    ellipsoid_vol_scores = [item[2] for item in scores]           # Extracting out scores for metric 'Volume of Wrench Space Ellipsoid'
    min_origin_dist_scores = [item[3] for item in scores]         # Extracting out scores for metric 'Minimum Distance of Convex Hull From Origin'

    # Extracting max score value for each metric for normalization
    max_minsigma_scores = max(minsigma_scores)                    
    max_isotropy_index_scores = max(isotropy_index_scores)
    max_ellipsoid_vol_scores = max(ellipsoid_vol_scores)
    max_min_origin_dist_scores = max(min_origin_dist_scores)

    # Combining metrics 'Minimum Singular Value', 'Isotropy Index' and 'Volume of Wrench Space Ellipsoid' into 'average_scores1'
    average_scores1 = (minsigma_scores/max_minsigma_scores)/((1+30+50)/1) + (isotropy_index_scores/max_isotropy_index_scores)/((1+30+50)/30) + (ellipsoid_vol_scores/max_ellipsoid_vol_scores)/((1+30+50)/50)
    
    # Normalizing score for metric 'Minimum Distance of Convex Hull From Origin' into 'average_score2'
    average_scores2 = min_origin_dist_scores/max_min_origin_dist_scores

    # Averaging out 'average_scores1' & 'average_scores2'
    average_scores = average_scores1/(10/1) + average_scores2/(10/9)

    # Assigning the final average scores to each grasp performance
    for j, grasp in enumerate(self.grasps_ordered):
      grasp[self.graspindices.get('performance')] = average_scores[j]

    # sort!
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]

    # For visualizing the ordered grasps
    #for k, grasp in enumerate(self.grasps_ordered): 
    #  print "Score: ", grasp[self.graspindices.get('performance')]
    #  self.show_grasp(grasp)
    #  raw_input()
    #  time.sleep(5)

  
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
  def order_grasps_noisy(self):
    self.grasps_ordered_noisy = self.grasps.copy() 

    scores = []

    for r, grasp in enumerate(self.grasps_ordered_noisy):   # Traversing through each grasp
      sum=[0.0, 0.0, 0.0, 0.0];
      for i in range (5):                                   # Randomly sampling 5 noisy grasps and summing their respective scores
        grasp_in = self.sample_random_grasp(grasp) 
        sum = map(add, sum, self.eval_grasp(grasp_in)) 
      
      scores.insert(i, np.divide(sum,5.0))    # Averaging the sum of scores and inserting it into 'scores' list
      
    minsigma_scores = [item[0] for item in scores]          # Extracting out scores for metric 'Minimum Singular Value'
    isotropy_index_scores = [item[1] for item in scores]    # Extracting out scores for metric 'Isotropy Index'
    ellipsoid_vol_scores = [item[2] for item in scores]     # Extracting out scores for metric 'Volume of Wrench Space Ellipsoid'
    min_origin_dist_scores = [item[3] for item in scores]   # Extracting out scores for metric 'Minimum Distance of Convex Hull From Origin'

    # Extracting max score value for each metric for normalization
    max_minsigma_scores = max(minsigma_scores)
    max_isotropy_index_scores = max(isotropy_index_scores)
    max_ellipsoid_vol_scores = max(ellipsoid_vol_scores)
    max_min_origin_dist_scores = max(min_origin_dist_scores)

    # Combining metrics 'Minimum Singular Value', 'Isotropy Index' and 'Volume of Wrench Space Ellipsoid' into 'average_scores1'
    average_scores1 = (minsigma_scores/max_minsigma_scores)/((1+30+50)/1) + (isotropy_index_scores/max_isotropy_index_scores)/((1+30+50)/30) + (ellipsoid_vol_scores/max_ellipsoid_vol_scores)/((1+30+50)/50)
    
    # Normalizing score for metric 'Minimum Distance of Convex Hull From Origin' into 'average_score2'
    average_scores2 = min_origin_dist_scores/max_min_origin_dist_scores

    # Averaging out 'average_scores1' & 'average_scores2'
    average_scores = average_scores1/(10/1) + average_scores2/(10/9)
    
    # Assigning the final average scores to each grasp performance
    for j, grasp in enumerate(self.grasps_ordered_noisy):
      grasp[self.graspindices.get('performance')] = average_scores[j]

    #Sorting
    order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered_noisy = self.grasps_ordered_noisy[order] 

    # For visualizing the ordered grasps
    #for k, grasp in enumerate(self.grasps_ordered_noisy):
    #  print("Score = ", grasp[self.graspindices.get('performance')])
    #  self.show_grasp(grasp)
    #  raw_input()
    #  time.sleep(5)


  # function to evaluate grasps
  # returns a score, which is some metric of the grasp
  # higher score should be a better grasp
  def eval_grasp(self, grasp):
    with self.robot:

      try:
        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

        obj_position = self.gmodel.target.GetTransform()[0:3,3]

        G = np.zeros((6, contacts.shape[0])) # Initializing the wrench matrix
        
        i = 0;
        
        for i, c in enumerate(contacts): # Populating the wrench matrix with direction vector and moments
          pos = c[0:3] - obj_position
          dir = -c[3:]
          G[0:3, i] = dir
          G[3:6, i] = np.cross(pos, dir)


        # Checking if origin lies inside convex hull of the wrench matrix
        try:
          hull = ConvexHull(G.transpose())
          origin = [0.0,0.0,0.0,0.0,0.0,0.0]
          G_origin = np.vstack((G.transpose(),origin))
          hull_origin = ConvexHull(G_origin)
          if list(hull.vertices) == list(hull_origin.vertices):
            b = True
            print("True")
          else:
            b = False
            print("False")

        except:
          b = False
          print("Exception: False")


        # Calculating metrics score
        if b == True:
          facets = hull.equations
          w,b = np.split(facets, [6], axis=1)
          norm = np.linalg.norm(w, axis=1)
          origin_distance = np.divide(np.fabs(b),norm)
          min_origin_distance = np.amin(origin_distance)     
          min_origin_distance = min_origin_distance*10000    #Quality Measure 1, Calculating the minimum distance between the convex hull boundary and origin for wrench matrix that has origin inside it's column space 


          G_GT = G.dot(G.transpose())

          u, s, v = np.linalg.svd(G)
          min_sigma = np.amin(s)                             #Quality Measure 2, Minimum singular value of G
          max_sigma = np.amax(s)

          isotropy_index = min_sigma / max_sigma             #Quality Measure 3, Isotropy Index of G

          det = np.linalg.det(G_GT)
          if (det < 0):
            det = 0
          ellipsoid_vol = 1*np.sqrt(det)                     #Quality Measure 4, Volume of Ellipsoid in wrench space

          return [min_sigma, isotropy_index, ellipsoid_vol, min_origin_distance]

        self.grasp_count = self.grasp_count + 1;
        print self.grasp_count

        return [0, 0, 0, 0]

      except openravepy.planning_error,e:
        #you get here if there is a failure in planning
        #example: if the hand is already intersecting the object at the initial position/orientation
        return  0.00
      
      #heres an interface in case you want to manipulate things more specifically
      #NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



  # given grasp_in, create a new grasp which is altered randomly
  # you can see the current position and direction of the grasp by:
  # grasp[self.graspindices.get('igrasppos')]
  # grasp[self.graspindices.get('igraspdir')]
  def sample_random_grasp(self, grasp_in):
    grasp = grasp_in.copy()

    #sample random position
    RAND_DIST_SIGMA = 0.005
    pos_orig = grasp[self.graspindices['igrasppos']]
    pos_noise = np.random.normal(0, RAND_DIST_SIGMA, pos_orig.shape)    
    grasp[self.graspindices['igrasppos']]=pos_orig+pos_noise

    #sample random orientation
    RAND_ANGLE_SIGMA = np.pi/72 
    dir_orig = grasp[self.graspindices['igraspdir']]
    roll_orig = grasp[self.graspindices['igrasproll']]
    
    dir_noise = np.random.normal(0, RAND_ANGLE_SIGMA, dir_orig.shape)
    roll_noise = np.random.normal(0, RAND_ANGLE_SIGMA, roll_orig.shape)
    
    grasp[self.graspindices['igraspdir']]=dir_orig+dir_noise
    grasp[self.graspindices['igrasproll']]=roll_orig+roll_noise
    
    return grasp


  #displays the grasp
  def show_grasp(self, grasp, delay=1.5):
    with openravepy.RobotStateSaver(self.gmodel.robot):
      with self.gmodel.GripperVisibility(self.gmodel.manip):
        time.sleep(0.1) # let viewer update?
        try:
          with self.env:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
            #if mindist == 0:
            #  print 'grasp is not in force closure!'
            contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
            self.gmodel.robot.GetController().Reset(0)
            self.gmodel.robot.SetDOFValues(finalconfig[0])
            self.gmodel.robot.SetTransform(finalconfig[1])
            self.env.UpdatePublishedBodies()
            time.sleep(delay)
        except openravepy.planning_error,e:
          print 'bad grasp!',e

if __name__ == '__main__':
  robo = RoboHandler()
  #time.sleep(10000) #to keep the openrave window open

  
