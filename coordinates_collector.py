from visual_nav.visual_coords import VisualCoords
from lidars.lidar_coords import LidarCoords
from multiprocessing import Process

class CoordinatesCollector():
    
    # coords : {'coords': (x,y), 'weight': 0.3}
    def __init__(self, state, dirve_coords):
        self.dirve_coords = dirve_coords
        self.state = state
        self.result_coords = dirve_coords
        # self.__init_coords_services()

    def get_verifyed_coords(self):
        if self.state['drive_state'] == 'stop':
            self.result_coords = self.verify_coords(self.dirve_coords)

        return self.result_coords

    def verify_coords(self, pure_coords):
        # TODO: temp - implement lidar and visual coords
        # get coords from Visual and Lidar services here
        return pure_coords

    def __del__(self):    
        self.__stop_all_services()

if __name__ == '__main__':
    # TODO: get state and drive coords!
    state = {
        'drive_state': 'stop'
    }
    dc = {'coords': (0,0), 'weight': 0.3}
    
    cc = CoordinatesCollector(state, dc)
    verifyed = cc.get_verifyed_coords()


