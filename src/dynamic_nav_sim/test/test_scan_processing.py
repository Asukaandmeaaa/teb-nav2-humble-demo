"""Run with Humble sourced: python3 -m unittest discover -s test."""
import math
from pathlib import Path
import sys
import unittest
from collections import deque
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from filter_self_scan import SelfScanFilter
from scan_obstacle_tracker import fit_circle, ScanObstacleTracker


class ScanProcessingTests(unittest.TestCase):
    def test_scan_waits_for_exact_transform_and_expires_stale_predictions(self):
        class Capture:
            messages=[]
            def publish(self,msg):self.messages.append(msg)
        clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(1.01e9),to_msg=lambda:LaserScan().header.stamp))
        tracker=SimpleNamespace(pending_scans=deque(),tracks={1:{}},last_publish_stamp=1.,
                                publisher=Capture(),get_clock=lambda:clock)
        scan=LaserScan();scan.header.stamp.sec=1
        tracker.pending_scans.append(scan)
        tracker.scan=lambda msg:False
        ScanObstacleTracker.process_pending(tracker)
        self.assertEqual(len(tracker.pending_scans),1)
        tracker.scan=lambda msg:True
        ScanObstacleTracker.process_pending(tracker)
        self.assertEqual(len(tracker.pending_scans),0)
        self.assertFalse(tracker.publisher.messages)
        clock.now=lambda:SimpleNamespace(nanoseconds=int(1.7e9),to_msg=lambda:LaserScan().header.stamp)
        tracker.pending_scans.append(scan)
        ScanObstacleTracker.process_pending(tracker)
        self.assertFalse(tracker.pending_scans)
        self.assertFalse(tracker.tracks)
        self.assertEqual(len(tracker.publisher.messages[-1].obstacles),0)

    def test_crossing_stop_restart_and_lost_track(self):
        class IdentityTF:
            def lookup_transform(self, *args):
                result=TransformStamped();result.transform.rotation.w=1.
                return result
        class Capture:
            def publish(self, msg): self.msg=msg
        tracker=SimpleNamespace(tf=IdentityTF(),publisher=Capture(),tracks={},next_id=100)
        def observe(t,y):
            scan=LaserScan();scan.header.frame_id='laser_link'
            scan.header.stamp.sec=int(t);scan.header.stamp.nanosec=round((t-int(t))*1e9)
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/400
            scan.range_min=.05;scan.range_max=12.
            angles=scan.angle_min+np.arange(400)*scan.angle_increment
            dot=2*np.cos(angles)+(y or 0)*np.sin(angles)
            discriminant=dot*dot-(4+(y or 0)**2-.4*.4)
            distances=dot-np.sqrt(np.maximum(discriminant,0))
            distances[(discriminant<0)|(distances<0)]=np.inf
            if y is None: distances[:]=np.inf
            scan.ranges=distances.astype(np.float32).tolist()
            ScanObstacleTracker.scan(tracker,scan)
        for i in range(11): observe(i*.1,i*.05)
        measured=tracker.publisher.msg.obstacles[0]
        self.assertAlmostEqual(measured.polygon.points[0].y,.5,places=3)
        self.assertAlmostEqual(measured.velocities.twist.linear.y,.5,places=2)
        self.assertAlmostEqual(measured.radius,.51,places=2)
        for i in range(11,15): observe(i*.1,.5)
        obstacles=tracker.publisher.msg.obstacles
        self.assertEqual(len(obstacles),1)
        self.assertEqual(obstacles[0].velocities.twist.linear.y,0.)
        self.assertAlmostEqual(obstacles[0].radius,.46,places=3)
        for i in range(15,19): observe(i*.1,.5+(i-14)*.05)
        self.assertAlmostEqual(tracker.publisher.msg.obstacles[0].velocities.twist.linear.y,.5,places=2)
        for i in range(19,27): observe(i*.1,None)
        self.assertEqual(len(tracker.publisher.msg.obstacles),0)

    def test_circle_and_unobservable_geometry(self):
        angles = np.linspace(-1.2, 1.2, 25)
        points = np.column_stack((.4*np.cos(angles)+2, .4*np.sin(angles)-1))
        center, radius = fit_circle(points)
        np.testing.assert_allclose(center, [2,-1], atol=1.e-8)
        self.assertAlmostEqual(radius, .4)
        self.assertIsNone(fit_circle(np.column_stack((np.linspace(0,1,30),np.zeros(30)))))
        self.assertIsNone(fit_circle([[0,0],[1,1]]))

    def test_self_returns_are_invalid_not_clear_space(self):
        class Capture:
            def publish(self, msg): self.msg = msg
        class Filter:
            lx, ly = .2, 0.
            xmin, xmax, ymin, ymax = -.3, .3, -.26, .26
            publisher = Capture()
        scan = LaserScan()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = 0.
        scan.angle_increment = math.pi/2
        # First hit is inside the chassis; second is an external close obstacle.
        scan.ranges = [.08, .3, float('inf'), float('nan')]
        obj = Filter()
        SelfScanFilter.scan(obj, scan)
        values = obj.publisher.msg.ranges
        self.assertTrue(math.isnan(values[0]))
        self.assertAlmostEqual(values[1], .3, places=6)
        self.assertTrue(math.isinf(values[2]))
        self.assertTrue(math.isnan(values[3]))
        self.assertEqual(obj.publisher.msg.header.frame_id, 'laser_link')


if __name__ == '__main__':
    unittest.main()
