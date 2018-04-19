import logging, numpy, openravepy
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError
import scipy

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

        self.order_grasps()
       
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
        
        return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

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

    
