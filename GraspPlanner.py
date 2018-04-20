import logging, numpy, openravepy
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import scipy
import time
from IPython import embed
from SimpleRobot import SimpleRobot

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

        self.grasp_count = 0

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps

        # this is the top ordered grasp
        #a_grasp = numpy.array([  0.00000000e+00   ,0.00000000e+00,   1.00000000e+00,   0.00000000e+00,   0.00000000e+00   ,0.00000000e+00,  -9.61945095e-01,  -1.91328851e-01,   1.95076665e-01  ,-1.91328851e-01,  -3.80549051e-02, -9.80787997e-01,   1.95076665e-01  ,-9.80787997e-01,   3.33066907e-16,  -3.64299309e-02,   1.86975157e-01   ,5.00000000e-02,   1.72805325e+09,   3.92523115e-16,   1.37940032e-01  ,-6.93521844e-01,   7.07106781e-01,  -3.64299309e-02,   1.86975157e-01   ,5.00000000e-02,  -9.61945095e-01,  -1.91328851e-01,   1.95076665e-01  ,-1.91328851e-01,  -3.80549051e-02,  -9.80787997e-01,   1.95076665e-01  ,-9.80787997e-01,   3.33066907e-16,  -3.64123740e-02,   1.86886886e-01   ,5.00000000e-02,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00   ,3.14159265e+00,   1.13559418e+00,   1.95076665e-01,  -9.80787997e-01   ,0.00000000e+00,   1.32500002e+00,   1.52500002e+00,   1.51400002e+00   ,1.00000000e+00,   1.00000000e+00,   1.00000000e+00,   2.50000000e-02   ,3.60495281e-03,  -6.19499877e-03,   3.49628257e-02, 5.00000000e-02   ,0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00   ,0.00000000e+00,   0.00000000e+00,   0.00000000e+00])

        a_grasp = numpy.array([  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.91328851e-01,  -9.61945095e-01,   1.95076665e-01,   3.80549051e-02,  -1.91328851e-01,  -9.80787997e-01,   9.80787997e-01,   1.95076665e-01,   4.44089210e-16,  -1.86975157e-01,  -3.64299309e-02,   5.00000000e-02,   1.72805325e+09,  -5.00000000e-01,   5.87932331e-01,  -3.92855666e-01,   5.00000000e-01,  -1.86975157e-01,  -3.64299309e-02,   5.00000000e-02,   1.91328851e-01,  -9.61945095e-01,   1.95076665e-01,   3.80549051e-02,  -1.91328851e-01,  -9.80787997e-01,   9.80787997e-01,   1.95076665e-01,   4.44089210e-16,  -1.86886886e-01,  -3.64123740e-02,   5.00000000e-02,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   4.71238898e+00,   1.11040077e+00,   9.80787997e-01,   1.95076665e-01,   0.00000000e+00,   1.32500002e+00,   1.52500002e+00,   1.51400002e+00,   1.00000000e+00,   1.00000000e+00,   1.00000000e+00,   2.50000000e-02,   3.58112175e-03,  -3.49628257e-02,  -6.19499877e-03,
   5.00000000e-02,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00])
        
        self.show_grasp(a_grasp)
        # self.order_grasps()

        #print self.grasps_ordered[0]

        '''
        for grasp in self.grasps_ordered[:5]:
          print grasp
          self.show_grasp(grasp)
          # these have already been checking for table collision
          # just have to do inverse reachability and pick any feasible pose
        '''
        

        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot)

        if self.irmodel.load():
          print "irmodel loaded"

        Tgrasp = self.gmodel.getGlobalGraspTransform(a_grasp, collisionfree=True)
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)

        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = 100000
        N = 1
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    table = self.robot.GetEnv().GetBodies()[1]
                    # if not self.gmodel.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                    if not self.gmodel.manip.CheckIndependentCollision(openravepy.CollisionReport()) and not self.robot.GetEnv().CheckCollision(table):
                        q = self.gmodel.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.gmodel.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.gmodel.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1


        print 'showing %d results'%N
        for ind,goal in enumerate(goals):
            raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,pose,values = goal
            self.robot.SetTransform(pose)
            self.robot.SetDOFValues(values)

        # get ordered grasps
        # iterate through ordered grasps
        # for each, check if it collides with table
        # if not, get distance from bottle to robot center
        # iterate through robot base positions, check collision
        # if there's a good one, return

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        
        angle = openravepy.openravepy_int.axisAngleFromQuat(pose[0:4])
        # embed()
        base_pose = [pose[4], pose[5], angle[2]]
        print('base_pose: {}'.format(base_pose))
        grasp_config = values[self.gmodel.manip.GetArmIndices()]    # a_grasp
        print('grasp_config: {}'.format(grasp_config))
        print('values: {}'.format(values))
        print('arm_indices: {}'.format(self.gmodel.manip.GetArmIndices()))
        print('Tgrasp: {}'.format(Tgrasp))
        raw_input('press ENTER to stop showing goal')

        return base_pose, grasp_config

    def PlanToGrasp(self, obj):
# 
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()
    
        env = openravepy.Environment()
        herb_base = SimpleRobot(env, self.base_planner.planning_env.robot)

        # embed()
        # Now plan to the base pose
        herb_base.SetCurrentConfiguration(base_pose)
        raw_input()
        herb_base.SetCurrentConfiguration(start_pose)

        print('start_pose: {}'.format(start_pose))
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)
        
        herb_base.SetCurrentConfiguration(start_pose)
        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)
        raw_input('Press ENTER to start arm planning')
        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        print('start_config: {}'.format(start_config))
        # grasp_config = [-1.73407926e+00, -1.71189130e+00, 6.60000000e-01, 1.82239572e+00, -1.63378598e+00, 6.54778697e-01, 1.41467576e+00]
        arm_plan, _ = self.arm_planner.Plan(start_config, grasp_config)

        # Random Jugaad
        arm_plan_ = [arm_plan[i] for i in range(len(arm_plan))]
        # embed()
        # arm_plan__ = [arm_plan_[i] for i in reversed(range(len(arm_plan)))]
        # embed()
        # print('arm_plan_: {}'.format(arm_plan_.reverse()))
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)
        raw_input('Press ENTER to start grasping')
        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()

    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy()
        scores = []
        for i, grasp in enumerate(self.grasps_ordered):
          score = self.eval_grasp(grasp)
          print "score", score
          scores.insert(i, score)

        minsigma_scores = [item[0] for item in scores]                # Extractin$
        isotropy_index_scores = [item[1] for item in scores]          # Extractin$
        ellipsoid_vol_scores = [item[2] for item in scores]           # Extractin$
        min_origin_dist_scores = [item[3] for item in scores]         # Extractin$

        # Extracting max score value for each metric for normalization
        max_minsigma_scores = max(minsigma_scores)
        max_isotropy_index_scores = max(isotropy_index_scores)
        max_ellipsoid_vol_scores = max(ellipsoid_vol_scores)
        max_min_origin_dist_scores = max(min_origin_dist_scores)

        # Combining metrics 'Minimum Singular Value', 'Isotropy Index' and 'Volum$
        average_scores1 = (minsigma_scores/max_minsigma_scores)/((1+30+50)/1) + (isotropy_index_scores/max_isotropy_index_scores)/((1+30+50)/30) + (ellipsoid_vol_scores/max_ellipsoid_vol_scores)/((1+30+50)/50)

        # Normalizing score for metric 'Minimum Distance of Convex Hull From Orig$
        average_scores2 = min_origin_dist_scores/max_min_origin_dist_scores

        # Averaging out 'average_scores1' & 'average_scores2'
        average_scores = average_scores1/(10/1) + average_scores2/(10/9)

        # Assigning the final average scores to each grasp performance
        for j, grasp in enumerate(self.grasps_ordered):
          grasp[self.graspindices.get('performance')] = average_scores[j]

        # sort!
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

        # For visualizing the ordered grasps
        #for k, grasp in enumerate(self.grasps_ordered): 
        #  print "Score: ", grasp[self.graspindices.get('performance')]
        #  self.show_grasp(grasp)
        #  raw_input()
        #  time.sleep(5)


    # function to evaluate grasps
    # returns a score, which is some metric of the grasp
    # higher score should be a better grasp
    def eval_grasp(self, grasp):
        with self.robot:
          # print "-----------------------"
          # print "grasp"
          try:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

            obj_position = self.gmodel.target.GetTransform()[0:3,3]

            # print "obj_pos", obj_position

            G = numpy.zeros((6, contacts.shape[0])) # Initializing the wrench matrix
        
            i = 0;
        
            for i, c in enumerate(contacts): # Populating the wrench matrix with direction vector and moments
              pos = c[0:3] - obj_position
              dir = -c[3:]
              G[0:3, i] = dir
              G[3:6, i] = numpy.cross(pos, dir)


            # Checking if origin lies inside convex hull of the wrench matrix
            try:
              # print "G", G
              hull = ConvexHull(G.transpose())
              # print "hull"
              origin = [0.0,0.0,0.0,0.0,0.0,0.0]
              G_origin = numpy.vstack((G.transpose(),origin))
              # print "G_origin"
              hull_origin = ConvexHull(G_origin)
              # print "hull_origin"
              if list(hull.vertices) == list(hull_origin.vertices):
                b = True
                print("True")
              else:
                b = False
                print("False")

            except QhullError:
              b = False
              print("Exception: False")


            # Calculating metrics score
            if b == True:
              facets = hull.equations
              w,b = numpy.split(facets, [6], axis=1)
              norm = numpy.linalg.norm(w, axis=1)
              origin_distance = numpy.divide(numpy.fabs(b),norm)
              min_origin_distance = numpy.amin(origin_distance)     
              min_origin_distance = min_origin_distance*10000    #Quality Measure 1, Calculating the minimum distance between the convex hull boundary and origin for wrench matrix that has origin inside it's column space 


              G_GT = G.dot(G.transpose())

              u, s, v = numpy.linalg.svd(G)
              min_sigma = numpy.amin(s)                             #Quality Measure 2, Minimum singular value of G
              max_sigma = numpy.amax(s)

              isotropy_index = min_sigma / max_sigma             #Quality Measure 3, Isotropy Index of G

              det = numpy.linalg.det(G_GT)
              if (det < 0):
                det = 0
              ellipsoid_vol = 1*numpy.sqrt(det)                     #Quality Measure 4, Volume of Ellipsoid in wrench space

              return [min_sigma, isotropy_index, ellipsoid_vol, min_origin_distance]

            self.grasp_count = self.grasp_count + 1;
            print self.grasp_count

            return [0, 0, 0, 0]

          except openravepy.planning_error,e:
            #you get here if there is a failure in planning
            #example: if the hand is already intersecting the object at the initial position/orientation
            return  [0, 0, 0, 0]

    
    #displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
          with self.gmodel.GripperVisibility(self.gmodel.manip):
            time.sleep(0.1) # let viewer update?
            try:
                print "showing!"
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
                #if mindist == 0:
                #  print 'grasp is not in force closure!'
                contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                self.gmodel.robot.GetController().Reset(0)
                self.gmodel.robot.SetDOFValues(finalconfig[0])
                self.gmodel.robot.SetTransform(finalconfig[1])
                print('finalconfig: {}\n, finalconfig[0]: {}\n, finalconfig[1]: {}'.format(finalconfig, finalconfig[0], finalconfig[1]))
                raw_input("Visualizing grasp, enter to continue")
            except openravepy.planning_error,e:
              print 'bad grasp!',e
