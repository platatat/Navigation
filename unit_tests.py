"""A module for unit testing of nav repo functions."""
import unittest
import geometry
import bikeState
import mapModel
import nav
import math
import numpy as np

class TestGeometry(unittest.TestCase):
  
    def test_unit_vector(self):
        p1 = (0,0)
        p2 = (-2,1)
        result = geometry.unit_vector(p1,p2)
        expected = [-2./np.sqrt(5), 1./np.sqrt(5)]
        
        np.testing.assert_array_almost_equal(result, expected)
  
    def test_dist2(self):
        p1 = (-3,5)
        p2 = (7,-1)
        result = geometry.dist2(p1,p2)
        expected = 136
        
        self.assertEquals(result, expected)
    
    def test_distance(self):
        p1 = (-3,5)
        p2 = (7,-1)
        result = geometry.distance(p1,p2)
        expected = np.sqrt(136)
        
        self.assertEquals(result, expected)
    
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
