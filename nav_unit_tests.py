import unittest
import bikeState
import mapModel
import nav
import math
import numpy as np

class TestNav(unittest.TestCase):
 
    def test_clamp_steer_angle(self):
        new_bike = bikeState.Bike(0, 0, 0, 0, 0, 0, 0)
        waypoints = []
        new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
        new_nav = nav.Nav(new_map_model)
        
        result = nav.Nav.clamp_steer_angle(new_nav, math.pi/4)
        expected = math.pi/6
        
        result2 = nav.Nav.clamp_steer_angle(new_nav, -math.pi/4)
        expected2 = -math.pi/6
        
        self.assertEquals(result, expected)
        self.assertEquals(result2, expected2)
    
if __name__ == '__main__':
    unittest.main()
