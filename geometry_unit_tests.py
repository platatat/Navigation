"""A module for testing geometry.py functions."""
import unittest
import geometry
import math
import numpy as np

class TestGeometry(unittest.TestCase):
  
    def test_unit_vector(self):
        p1 = (0,0)
        p2 = (-2,1)
        result = geometry.unit_vector(p1,p2)
        expected = [-2./np.sqrt(5), 1./np.sqrt(5)]
        
        p3 = (0,0)
        p4 = (0,0.5)
        result2 = geometry.unit_vector(p3,p4)
        expected2 = [0,1]
        
        np.testing.assert_array_almost_equal(result, expected)
        np.testing.assert_array_almost_equal(result2, expected2)
  
    def test_dist2(self):
        p1 = (-3,5)
        p2 = (7,-1)
        result = geometry.dist2(p1,p2)
        expected = 136
        
        self.assertEqual(result, expected)
    
    def test_distance(self):
        p1 = (-3,5)
        p2 = (7,-1)
        result = geometry.distance(p1,p2)
        expected = np.sqrt(136)
        
        self.assertEquals(result, expected)
    
    def test_get_sign(self):
        result = geometry.get_sign(0)
        result2 = geometry.get_sign(2)
        result3 = geometry.get_sign(-1)
        result4 = geometry.get_sign(-.05)
        
        self.assertEqual(result, 0)
        self.assertEqual(result2, 1)
        self.assertEqual(result3, -1)
        self.assertEqual(result4, -1)
    
    def test_nearest_point_on_path(self):
        p = geometry.nearest_point_on_path

        path = [(-1,1), (5,1)]
        
        result1 = p(path, (1,1))
        result2 = p(path, (-10,-40))
        result3 = p(path, (10,40))
        result4 = p(path, (2,2))
        result5 = p(path, (2,-1))
        
        np.testing.assert_array_almost_equal(result1, (1,1))
        np.testing.assert_array_almost_equal(result2, (-1,1))
        np.testing.assert_array_almost_equal(result3, (5,1))
        np.testing.assert_array_almost_equal(result4, (2,1))
        np.testing.assert_array_almost_equal(result5, (2,1))
        
        path2 = [(0,0), (0,0)]
        point = (-10, 18.3902)
        result6 = p(path2,point)
        
        np.testing.assert_array_almost_equal(result6, (0,0))

        path_vertical = [(0,0),(0,50)]
        point_on_vert_path = (0,1)
        point_off_vert_path = (1,1)

        np.testing.assert_array_almost_equal(p(path_vertical, point_on_vert_path), (0, 1))
        np.testing.assert_array_almost_equal(p(path_vertical, point_off_vert_path), (0, 1))
        
    def test_angle_from_path(self):
        p1 = (0,0)
        p2 = (2,5)
        bike_direction = math.pi/4
        result = geometry.angle_from_path(bike_direction, p1, p2)
        expected = 7*math.sqrt(58)/58
        #self.assertEquals(result, expected)
    
    def test_distance_from_path(self):
        path = [(-1,1), (5,1)]
        
        point1 = (1,1)
        result1 = geometry.distance_from_path(path, point1)
        expected1 = 0
        
        point2 = (0,0)
        result2 = geometry.distance_from_path(path, point2)
        expected2 = 1
        
        point3 = (-10, -40)
        result2 = geometry.distance_from_path(path, point3)
        expected3 = 41.976183723630712
        
if __name__ == '__main__':
    unittest.main()
