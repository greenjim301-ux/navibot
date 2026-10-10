"""Run in the Noetic environment: python3 -m unittest discover -s backend/tests -v."""
import asyncio
from contextlib import ExitStack
import threading
import time
import unittest
from unittest.mock import patch

from backend.app.models import PlanPathPoint, TaskState, Waypoint
from backend.app.route_manager import RouteManager, _Altitude


class FakeRos:
    def __init__(self):
        self.routes = []
        self.stops = 0
        self.fail_publish = False
        self.fail_stop = False

    def publish_waypoints(self, points):
        if self.fail_publish:
            raise RuntimeError("planner unavailable")
        self.routes.append(points)

    def emergency_stop(self):
        self.stops += 1
        if self.fail_stop:
            raise RuntimeError("planner unavailable")


class FakeWs:
    def __init__(self):
        self.messages = []

    def broadcast_threadsafe(self, message):
        self.messages.append(message)


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for navigation state")


class MultiNavigationTests(unittest.TestCase):
    def setUp(self):
        self.ros = FakeRos()
        self.ws = FakeWs()
        self.active_map = "test"
        self.manager = RouteManager(self.ros, self.ws, lambda: self.active_map)
        # Keep the state machine real; isolate map assets and ROS transport.
        self.manager._resolve_altitudes = lambda wps, *_: [_Altitude(z=0) for _ in wps]
        self.manager._log_dispatch = lambda *_: None
        self.pose(0)
        self.goals = [Waypoint(x=3, y=0), Waypoint(x=6, y=0)]
        self.plans = []

    def tearDown(self):
        self.ros.fail_stop = False
        self.manager.estop()

    def pose(self, x, cov=0, stamp=None):
        self.manager.on_pose(x, 0, 0, 0, cov, time.time() if stamp is None else stamp)

    def plan(self, name, pose, goal):
        self.plans.append((pose.x, goal.x))
        return [PlanPathPoint(x=pose.x, y=pose.y, z=0),
                PlanPathPoint(x=goal.x, y=goal.y, z=0)]

    def start(self, plan=None):
        return self.manager.start_multi_navigation(self.goals, "test", plan or self.plan)

    def test_dispatches_one_segment_only_after_planner_confirmation(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.pose(3)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.manager.get_status().state, TaskState.RUNNING)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        wait_for(lambda: len(self.ros.routes) == 2)
        self.assertEqual(self.plans, [(0, 3), (3, 6)])
        self.assertEqual([r[-1]["x"] for r in self.ros.routes], [3, 6])
        self.pose(6)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        wait_for(lambda: self.manager.get_status().multi_navigation.state == TaskState.SUCCEEDED)
        status = self.manager.get_status()
        self.assertEqual(status.multi_navigation.current_index, 2)
        self.assertEqual(status.state, TaskState.SUCCEEDED)
        self.assertEqual(self.ws.messages[-1]["data"]["multi_navigation"]["state"], "succeeded")

    def test_next_segment_planning_does_not_report_previous_route_for_new_index(self):
        entered, release = threading.Event(), threading.Event()

        def blocked_second_plan(name, pose, goal):
            if goal.x == 6:
                entered.set()
                release.wait(3)
            return self.plan(name, pose, goal)

        try:
            self.start(blocked_second_plan)
            wait_for(lambda: len(self.ros.routes) == 1)
            self.assertEqual(self.manager.get_status().multi_navigation.route[-1].x, 3)
            self.pose(3)
            self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
            self.assertTrue(entered.wait(2))
            status = self.manager.get_status().multi_navigation
            self.assertEqual(status.current_index, 1)
            self.assertEqual(status.route, [])
            self.assertEqual(self.ws.messages[-1]["data"]["multi_navigation"]["route"], [])
            release.set()
            wait_for(lambda: len(self.ros.routes) == 2)
            self.assertEqual(self.manager.get_status().multi_navigation.route[-1].x, 6)
        finally:
            release.set()

    def test_stop_during_planning_discards_late_result(self):
        entered, release, exited = threading.Event(), threading.Event(), threading.Event()

        def blocked_plan(*args):
            entered.set()
            release.wait(3)
            try:
                return self.plan(*args)
            finally:
                exited.set()

        self.start(blocked_plan)
        self.assertTrue(entered.wait(2))
        self.manager.estop()
        release.set()
        self.assertTrue(exited.wait(2))
        with self.manager._lock:
            self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.STOPPED)
            self.assertEqual(self.ros.routes, [])
        self.assertEqual(self.ros.stops, 1)

    def test_stop_after_dispatch_cancels_remaining_goals(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.manager.estop()
        self.pose(3)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.STOPPED)
        self.assertEqual(self.manager.get_status().state, TaskState.STOPPED)

    def test_stop_transport_failure_still_invalidates_queue(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.ros.fail_stop = True
        with self.assertRaises(RuntimeError):
            self.manager.estop()
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.STOPPED)

    def test_planner_emergency_cancels_queue(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.manager.on_planning_finished(RouteManager.FINISHED_EMERGENCY_STOP)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.FAILED)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.ros.stops, 0)

    def test_plan_failure_never_dispatches(self):
        def failing_plan(*_):
            raise ValueError("no path")
        self.start(failing_plan)
        wait_for(lambda: self.manager.get_status().multi_navigation.state == TaskState.FAILED)
        self.assertEqual(self.ros.routes, [])
        self.assertIn("no path", self.manager.get_status().multi_navigation.message)

    def test_publish_failure_cancels_queue(self):
        self.ros.fail_publish = True
        self.start()
        wait_for(lambda: self.manager.get_status().multi_navigation.state == TaskState.FAILED)
        self.assertEqual(len(self.plans), 1)
        self.assertEqual(self.ros.routes, [])

    def test_invalid_localization_and_inactive_map_are_rejected(self):
        self.pose(0, cov=0.99)
        with self.assertRaises(ValueError):
            self.start()
        self.pose(0, stamp=time.time() - 60)
        with self.assertRaises(ValueError):
            self.start()
        self.pose(0)
        self.active_map = "other"
        with self.assertRaises(ValueError):
            self.start()
        self.assertEqual(self.ros.routes, [])

    def test_mutual_exclusion_with_single_navigation(self):
        self.start()
        with self.assertRaises(ValueError):
            self.manager.submit_route([Waypoint(x=9, y=0)], map_name="test")
        with self.assertRaises(ValueError):
            self.start()
        self.manager.estop()
        self.manager.submit_route([Waypoint(x=9, y=0)], map_name="test")
        with self.assertRaises(ValueError):
            self.start()

    def test_single_navigation_retains_odom_completion(self):
        self.manager.submit_route([Waypoint(x=3, y=0)], map_name="test")
        self.pose(3)
        self.assertEqual(self.manager.get_status().state, TaskState.SUCCEEDED)
        self.assertEqual(self.manager.get_status().current_index, 1)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.IDLE)

    def test_degenerate_goal_still_waits_for_confirmation(self):
        self.goals = [Waypoint(x=0, y=0), Waypoint(x=3, y=0)]
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.assertEqual(self.manager.get_status().state, TaskState.RUNNING)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        wait_for(lambda: len(self.ros.routes) == 2)
        self.assertEqual([r[-1]["x"] for r in self.ros.routes], [0, 3])

    def test_no_progress_for_ten_minutes_still_dispatches_next_segment_on_confirmation(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        real_time = time.time
        with patch('backend.app.route_manager.time.time', side_effect=lambda: real_time() + 600):
            self.pose(0)
            status = self.manager.get_status()
            self.assertEqual(status.state, TaskState.RUNNING)
            self.assertEqual(status.multi_navigation.state, TaskState.RUNNING)
            self.assertEqual(self.ros.stops, 0)
            self.pose(3)
            self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
            wait_for(lambda: len(self.ros.routes) == 2)
        self.assertEqual(self.ros.routes[-1][-1]['x'], 6)
        self.assertEqual(self.ros.stops, 0)

    def test_missed_intermediate_waypoint_does_not_block_next_segment(self):
        def segmented_plan(name, pose, goal):
            return [PlanPathPoint(x=pose.x, y=0, z=0),
                    PlanPathPoint(x=goal.x - 1, y=0, z=0),
                    PlanPathPoint(x=goal.x, y=0, z=0)]
        self.start(segmented_plan)
        wait_for(lambda: len(self.ros.routes) == 1)
        # Planner bypassed the first intermediate point; backend display lags.
        self.pose(3)
        self.assertEqual(self.manager.get_status().current_index, 0)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        wait_for(lambda: len(self.ros.routes) == 2)
        self.assertEqual(self.ros.routes[-1][-1]['x'], 6)
        self.assertEqual(self.ros.stops, 0)

    def test_single_navigation_has_no_no_progress_timeout(self):
        self.manager.submit_route([Waypoint(x=3, y=0)], map_name='test')
        real_time = time.time
        with patch('backend.app.route_manager.time.time', side_effect=lambda: real_time() + 600):
            self.pose(0)
            self.assertEqual(self.manager.get_status().state, TaskState.RUNNING)
        self.assertEqual(self.ros.stops, 0)

    def test_localization_loss_during_execution_does_not_publish_stop(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.pose(0, cov=0.99)
        wait_for(lambda: self.manager.get_status().multi_navigation.state == TaskState.FAILED)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.ros.stops, 0)
        self.assertIn('未发送停止指令', self.manager.get_status().message)
        self.pose(0)
        # The current planner segment remains active until user stop or finish.
        with self.assertRaises(ValueError):
            self.start()
        with self.assertRaises(ValueError):
            self.manager.submit_route([Waypoint(x=9, y=0)], map_name='test')
        self.manager.estop()
        self.assertEqual(self.ros.stops, 1)
        self.assertEqual(self.manager.get_status().state, TaskState.STOPPED)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.STOPPED)

    def test_failed_segment_can_finish_without_dispatching_cancelled_goals(self):
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        self.pose(0, cov=0.99)
        wait_for(lambda: self.manager.get_status().multi_navigation.state == TaskState.FAILED)
        self.pose(3)
        self.manager.on_planning_finished(RouteManager.FINISHED_REACHED)
        self.assertEqual(len(self.ros.routes), 1)
        self.assertEqual(self.ros.stops, 0)
        self.assertEqual(self.manager.get_status().state, TaskState.SUCCEEDED)
        self.assertEqual(self.manager.get_status().multi_navigation.state, TaskState.FAILED)
        self.start()
        wait_for(lambda: len(self.ros.routes) == 2)

    def test_multi_endpoint_and_shared_stop(self):
        from backend.app import main
        from backend.app.models import MultiNavigationRequest, NavStatus, RouteRequest
        with patch.object(main, "route_manager", self.manager), patch.object(main, "_plan_multi_segment", self.plan):
            response = asyncio.run(main.start_multi_navigation(
                MultiNavigationRequest(map_name="test", goals=self.goals)))
            self.assertEqual(NavStatus.model_validate(response.model_dump()).multi_navigation.state, TaskState.RUNNING)
            wait_for(lambda: len(self.ros.routes) == 1)
            response = asyncio.run(main.estop())
            self.assertEqual(response.multi_navigation.state, TaskState.STOPPED)
            response = asyncio.run(main.submit_route(
                RouteRequest(map_name="test", waypoints=[Waypoint(x=9, y=0)])))
            self.assertEqual(response.state, TaskState.RUNNING)
            self.assertEqual(self.ros.routes[-1][-1]["x"], 9)

    def test_restart_ignores_previous_pending_plan(self):
        entered, release, exited = threading.Event(), threading.Event(), threading.Event()

        def blocked_plan(*args):
            entered.set()
            release.wait(3)
            exited.set()
            return self.plan(*args)

        self.start(blocked_plan)
        self.assertTrue(entered.wait(2))
        self.manager.estop()
        self.goals = [Waypoint(x=9, y=0)]
        self.start()
        wait_for(lambda: len(self.ros.routes) == 1)
        release.set()
        self.assertTrue(exited.wait(2))
        self.assertEqual(self.ros.routes[0][-1]["x"], 9)
        self.assertEqual(self.manager.get_status().multi_navigation.goals[0].x, 9)


class SharedPlannerTests(unittest.TestCase):
    def test_preview_and_multi_dispatch_use_identical_constraints_and_points(self):
        from backend.app import main
        from backend.app.models import PlanPathRequest, Pose, XY
        pose = Pose(x=22.65, y=10.15, z=-0.25, yaw=0, stamp=time.time())
        goal = Waypoint(x=23.851, y=4.883)
        curved = [(22.65, 10.15), (22.339, 9.373), (23.85, 4.85)]
        straight = [(22.65, 10.15), (23.85, 4.85)]
        for trajectory in ([(22.65, 10.15, 0.046)], None):
            with self.subTest(trajectory=trajectory), ExitStack() as stack:
                manager = RouteManager(FakeRos(), FakeWs())
                stack.enter_context(patch.object(main, "route_manager", manager))
                stack.enter_context(patch.object(manager, "get_altitude_calibration", return_value=-0.296))
                stack.enter_context(patch.object(main.path_planner, "ground_elevation", return_value=0.046))
                stack.enter_context(patch.object(main.path_planner, "mapping_trajectory", return_value=trajectory))
                regions = [object()]
                stack.enter_context(patch.object(main.map_edit_store, "active_regions", return_value=regions))
                planner = stack.enter_context(patch.object(main.global_planner, "plan_path",
                    side_effect=lambda *args, **kw: curved if kw["trajectory"] is not None else straight))
                preview = asyncio.run(main.plan_path("test", PlanPathRequest(
                    start=XY(x=pose.x, y=pose.y), goal=XY(x=goal.x, y=goal.y))))
                dispatched = main._plan_multi_segment("test", pose, goal)
                self.assertEqual(preview.points, dispatched)
                self.assertEqual([(p.x, p.y) for p in dispatched], curved if trajectory is not None else straight)
                self.assertTrue(all(abs(p.z + 0.25) < 1e-9 for p in dispatched))
                self.assertEqual(planner.call_count, 2)
                for call in planner.call_args_list:
                    self.assertIs(call.kwargs["trajectory"], trajectory)
                    self.assertIs(call.kwargs["edit_regions"], regions)
                    self.assertEqual(call.kwargs["elevation_fn"]([(0, 0)]), [0.046])


if __name__ == "__main__":
    unittest.main()
