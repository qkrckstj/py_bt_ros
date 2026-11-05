import math
from pathlib import Path
import unittest
import yaml
from webots_ros2_limo.factory_navigation import configure_factory, relative_regions


class FactoryNavigationTests(unittest.TestCase):
    def test_negative_height_scan_is_accepted(self):
        p=yaml.safe_load((Path(__file__).parents[1]/'resource/diff/nav2_params.yaml').read_text())
        configure_factory(p)
        for name in ('local_costmap','global_costmap'):
            c=p[name][name]['ros__parameters']
            self.assertNotIn('voxel_layer',c['plugins'])
            layer=c['obstacle_layer']
            self.assertLess(layer['min_obstacle_height'],-0.034)
            self.assertLess(layer['scan']['min_obstacle_height'],-0.034)
            self.assertIn('footprint',c)
        ctrl=p['controller_server']['ros__parameters']
        self.assertEqual(ctrl['general_goal_checker']['plugin'],'nav2_controller::PositionGoalChecker')
        self.assertEqual(ctrl['general_goal_checker']['xy_goal_tolerance'],.3)
        self.assertNotIn('RotateToGoal',ctrl['FollowPath']['critics'])
        self.assertNotIn('GoalAlign',ctrl['FollowPath']['critics'])

    def test_slam_coordinates_include_start_heading(self):
        c={'mission':{'start_pose':{'x':2.,'y':3.,'yaw':math.pi/2},
           'regions':{1:{'x':2.,'y':5.,'yaw':math.pi}}}}
        relative_regions(c)
        p=c['mission']['regions'][1]
        self.assertAlmostEqual(p['x'],2.)
        self.assertAlmostEqual(p['y'],0.)
        self.assertAlmostEqual(p['yaw'],math.pi/2)

if __name__=='__main__':unittest.main()
