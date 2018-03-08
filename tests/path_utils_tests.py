#! /usr/bin/env python

import unittest
import numpy as np
from arc_utilities import path_utils as pu
import IPython





class TestPathUtils(unittest.TestCase):
    def assertApprox(self, a, b, eps = 0.00001):
        d = np.linalg.norm(np.array(a) - np.array(b))
        self.assertTrue(d < eps)

    def test_closest_point_to_line_simple(self):
        l0 = [-1,1]
        l1 = [1,1]
        line = [l0, l1]
        p, a = pu.closest_point_to_line(line, [0,0])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0,1])

        p, a = pu.closest_point_to_line(line, [-1,0])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [-1,1])

        p, a = pu.closest_point_to_line(line, [-1.1,0])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [-1,1])

        for true, computed in zip([-1,1], p):
            self.assertEqual(true, computed)

        p, a = pu.closest_point_to_line(line, [0.9,1])
        self.assertApprox(a, 0.95)
        self.assertApprox(p, [0.9, 1])


    def test_closest_point_to_line_large(self):
        l0 = np.array([1,2,3,4,5,6,7])
        l1 = -1*l0
        line = [l0, l1]

        p, a = pu.closest_point_to_line(line, [0,0,0,0,0,0,0])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0,0,0,0,0,0,0])

        p, a = pu.closest_point_to_line(line, [-7,-6,-5,0,3,2,1])
        self.assertApprox(a, 0.5)
        self.assertApprox(p, [0,0,0,0,0,0,0])

        p, a = pu.closest_point_to_line(line, [9,9,9,9,9,9,9])
        self.assertApprox(a, 0.0)
        self.assertApprox(p, [1,2,3,4,5,6,7])


    def test_closest_point_to_line(self):
        path = [[-1,0], [0,0], [0,1], [1,10]]

        p, ind, alpha = pu.closest_point(path, [0,0])
        self.assertEqual(ind, 1)
        self.assertApprox(p, [0,0])
        self.assertApprox(alpha,0)

        p, ind, alpha = pu.closest_point(path, [-10,1])
        self.assertEqual(ind, 0)
        self.assertApprox(p, [-1,0])
        self.assertApprox(alpha,0)

        p, ind, alpha = pu.closest_point(path, [10,10])
        self.assertEqual(ind, 3)
        self.assertApprox(p, [1,10])
        self.assertApprox(alpha,0)

        p, ind, alpha = pu.closest_point(path, [.1, .5])
        self.assertEqual(ind, 1)
        self.assertApprox(p, [0,.5])
        self.assertApprox(alpha,.5)
        
        
        

if __name__ == '__main__':
    unittest.main()




