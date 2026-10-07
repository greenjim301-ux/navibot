import unittest

from backend.app.models import TaskState, Waypoint
from backend.app.route_manager import RouteManager, _Altitude


class FakeRos:
    def publish_waypoints(self, points):
        pass

    def emergency_stop(self):
        pass


class FakeWs:
    def __init__(self):
        self.messages = []

    def broadcast_threadsafe(self, message):
        self.messages.append(message)


class OptimalTrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.active_map = "map_a"
        self.ws = FakeWs()
        self.manager = RouteManager(FakeRos(), self.ws, lambda: self.active_map)
        self.manager._resolve_altitudes = lambda waypoints, *_: [_Altitude(z=0) for _ in waypoints]
        self.manager._log_dispatch = lambda *_: None
        self.points = [{"x": 1, "y": 2, "z": 0, "r": 1, "g": 0, "b": 0}]

    def start(self):
        self.manager.submit_route([Waypoint(x=5, y=0)], map_name=self.active_map)

    def assert_cleared(self):
        self.assertEqual(self.manager.get_optimal_traj(), [])
        clears = [m for m in self.ws.messages if m == {"type": "optimal_traj", "data": {"points": []}}]
        self.assertTrue(clears)

    def test_idle_planner_marker_is_not_replayed_on_new_connection(self):
        self.manager.on_optimal_traj(self.points)
        self.assertEqual(self.manager.get_optimal_traj(), [])
        self.assertEqual(self.ws.messages, [])

    def test_active_navigation_trajectory_is_available_on_reentry(self):
        self.start()
        self.manager.on_optimal_traj(self.points)
        self.assertEqual(self.manager.get_optimal_traj(), self.points)

    def test_completion_and_failure_clear_cached_trajectory(self):
        for status in [RouteManager.FINISHED_REACHED, RouteManager.FINISHED_EMERGENCY_STOP]:
            with self.subTest(status=status):
                self.start()
                self.manager.on_optimal_traj(self.points)
                self.manager.on_planning_finished(status)
                self.assert_cleared()
                self.manager.on_optimal_traj(self.points)
                self.assertEqual(self.manager.get_optimal_traj(), [])

    def test_manual_stop_clears_existing_client_and_reconnect_snapshot(self):
        self.start()
        self.manager.on_optimal_traj(self.points)
        self.manager.estop()
        self.assert_cleared()
        self.assertEqual(self.manager.get_status().state, TaskState.STOPPED)

    def test_new_route_cannot_replay_previous_routes_curve(self):
        self.start()
        self.manager.on_optimal_traj(self.points)
        self.start()
        self.assert_cleared()
        self.manager.on_optimal_traj(self.points)
        self.assertEqual(self.manager.get_optimal_traj(), self.points)

    def test_switching_active_map_hides_old_maps_curve(self):
        self.start()
        self.manager.on_optimal_traj(self.points)
        self.active_map = "map_b"
        self.assertEqual(self.manager.get_optimal_traj(), [])
        self.manager.on_optimal_traj(self.points)
        self.assert_cleared()
