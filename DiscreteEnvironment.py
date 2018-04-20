import numpy as np

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = int(np.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx]))


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #

        print('config: {}'.format(config))
        coord = self.ConfigurationToGridCoord(config)
        print('coord: {}'.format(coord))
        node_id = self.GridCoordToNodeId(coord)

        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension

        coord = self.NodeIdToGridCoord(nid)

        config = self.GridCoordToConfiguration(coord)
        
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        config = np.array(config) - self.lower_limits
        coord = [0.0] * self.dimension

        coord = [x/self.resolution[i] for i,x in enumerate(config)]
        coord = np.floor(coord)
        coord = [int(x) for x in coord]

        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0.0] * self.dimension

        config = [(x+0.5)*self.resolution[i] for i,x in enumerate(coord)]
        config = np.array(config) + self.lower_limits

        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        # print('coord: {}'.format(coord))
        node_id = np.ravel_multi_index(coord, dims=self.num_cells, order='F')
        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        
        coord = np.unravel_index(node_id, self.num_cells, order='F')

        return coord

def main():
    resolution = np.array([0.1, 0.1])
    lower_limits = np.array([0.0, 0.0])
    upper_limits = np.array([1.0, 1.0])
    env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

    pos = np.array([0.73, 0.35])
    grid = env.ConfigurationToGridCoord(pos)
    node = env.GridCoordToNodeId(grid)
    node = env.ConfigurationToNodeId(pos)
    grid = env.NodeIdToGridCoord(node)
    pos = env.NodeIdToConfiguration(node)
    pos = env.GridCoordToConfiguration(grid)


if __name__ == "__main__":
    main()
