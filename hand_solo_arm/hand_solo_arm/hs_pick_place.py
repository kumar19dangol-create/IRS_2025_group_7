#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, json, re
from typing import List, Dict, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import BackUp          # <-- NEW
from builtin_interfaces.msg import Duration  # <-- NEW (for BackUp.time_allowance)

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint


# ---------------- Utils ----------------

def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qz * qz + qy * qy))

def make_map_pose(x: float, y: float, yaw: float) -> PoseStamped:
    qx, qy, qz, qw = yaw_to_quat(yaw)
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class HSAutoNavPickCarry(Node):
    """
    One node that:
      1) Navigates to a fixed pick waypoint at startup.
      2) Listens for /hmi/unified_status events.
      3) On new box event, runs MoveIt pick→carry, THEN navigates to a fixed place waypoint
         and performs the shown RViz place joint configuration.
      4) After placing, navigates to a final waypoint with the specified orientation,
         then returns to the existing pick waypoint.
    """

    JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

    def __init__(self):
        super().__init__('hs_auto_nav_pick_carry')

        self.cb_group = ReentrantCallbackGroup()

        # ---------- Parameters ----------
        # PICK waypoint
        self.declare_parameter('pick_x', 2.5757617011322917)
        self.declare_parameter('pick_y', -0.05907851544227876)
        self.declare_parameter('pick_yaw', -0.01854175)  # ≈ -0.78°

        # PLACE waypoint (UPDATED to your new pose)
        # orientation z=-0.7023802617558185, w=0.7118019161928605 → yaw ≈ -1.5574719967
        self.declare_parameter('place_x', 30.963558022812265)
        self.declare_parameter('place_y', -0.7113403203361557)
        self.declare_parameter('place_yaw', -1.6888530994062857)

        # FINAL waypoint (UPDATED to your new pose)
        # orientation z=-0.6377869107833034, w=0.7702128643650992 → yaw ≈ -1.38324297535
        self.declare_parameter('final_x',30.99423501660363)
        self.declare_parameter('final_y', -4.057977798371188)
        self.declare_parameter('final_yaw', -1.5649493521721334)

        self.declare_parameter('tol_xy_m', 0.20)
        self.declare_parameter('tol_yaw_deg', 5.0)
        self.declare_parameter('nav_timeout_s', 90.0)
        self.declare_parameter('stabilize_after_arrival_s', 2.0)
        self.declare_parameter('unified_status_topic', '/hmi/unified_status')
        self.declare_parameter('planning_group', 'tmr_arm')
        self.declare_parameter('joint_tolerance_deg', 2.0)

        # --- Per-size thresholds (only used if box.category not present) ---
        self.declare_parameter('small_le_kg', 5.0)
        self.declare_parameter('medium_le_kg', 10.0)

        # --- Per-size joint configs (degrees) ---
        self.declare_parameter('pick_small_deg',   [ 45.0, 40.0, 63.0, 10.0, 90.0, 30.0])
        self.declare_parameter('carry_small_deg',  [ 0.0, -0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('pick_medium_deg',  [ 5.0, 32.0, 60.0, 7.0, 90.0, 0.0])
        self.declare_parameter('carry_medium_deg', [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('pick_big_deg',     [ -32.0, 35.0, 50.0, 10.0, 90.0, 0.0])
        self.declare_parameter('carry_big_deg',    [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # PLACE joint config (degrees)
        self.declare_parameter('place_deg',        [ 0.0, 30.0, 57.0, -7.0, 90.0, 0.0])

        self.declare_parameter('wait_between_stages_s', 0.8)

        # ✅ NEW: a consistent delay just before starting the pick motion
        self.declare_parameter('delay_before_pick_s', 3.0)

        self.PICK_X   = float(self.get_parameter('pick_x').value)
        self.PICK_Y   = float(self.get_parameter('pick_y').value)
        self.PICK_YAW = float(self.get_parameter('pick_yaw').value)

        # place
        self.PLACE_X   = float(self.get_parameter('place_x').value)
        self.PLACE_Y   = float(self.get_parameter('place_y').value)
        self.PLACE_YAW = float(self.get_parameter('place_yaw').value)

        # final
        self.FINAL_X   = float(self.get_parameter('final_x').value)
        self.FINAL_Y   = float(self.get_parameter('final_y').value)
        self.FINAL_YAW = float(self.get_parameter('final_yaw').value)

        self.TOL_XY   = float(self.get_parameter('tol_xy_m').value)
        self.TOL_YAW  = math.radians(float(self.get_parameter('tol_yaw_deg').value))
        self.NAV_TIMEOUT_S = float(self.get_parameter('nav_timeout_s').value)
        self.STABILIZE_S   = float(self.get_parameter('stabilize_after_arrival_s').value)
        self.UNIFIED_TOPIC = str(self.get_parameter('unified_status_topic').value)
        self.PLANNING_GROUP = str(self.get_parameter('planning_group').value)
        self.JOINT_TOL_RAD = math.radians(float(self.get_parameter('joint_tolerance_deg').value))
        self.STAGE_WAIT = float(self.get_parameter('wait_between_stages_s').value)
        self.DELAY_BEFORE_PICK = float(self.get_parameter('delay_before_pick_s').value)

        self.SMALL_LE  = float(self.get_parameter('small_le_kg').value)
        self.MEDIUM_LE = float(self.get_parameter('medium_le_kg').value)

        # to radians conversion helpers
        def d2r_list(name: str) -> List[float]:
            return [math.radians(v) for v in self.get_parameter(name).value]

        self.PICK_SMALL  = d2r_list('pick_small_deg')
        self.CARRY_SMALL = d2r_list('carry_small_deg')
        self.PICK_MED    = d2r_list('pick_medium_deg')
        self.CARRY_MED   = d2r_list('carry_medium_deg')
        self.PICK_BIG    = d2r_list('pick_big_deg')
        self.CARRY_BIG   = d2r_list('carry_big_deg')
        self.PLACE_JOINTS = d2r_list('place_deg')

        # ---------- State ----------
        self._amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._last_box_total = 0
        self.busy = False
        self._movegroup_client_active: Optional[ActionClient] = None
        self._nav_handle = None
        our_mg = None  # not used, retained style
        self._mg_handle = None

        self._last_carry_joints: Optional[List[float]] = None   # <-- NEW

        # ---------- ROS I/O ----------
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10, callback_group=self.cb_group
        )
        self.sub_evt = self.create_subscription(
            String, self.UNIFIED_TOPIC, self._event_cb, 10, callback_group=self.cb_group
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)

        # Prepare MoveGroup action clients (we'll pick whichever is available)
        self.mg_client_move_action = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self.mg_client_move_group  = ActionClient(self, MoveGroup, 'move_group',  callback_group=self.cb_group)

        # <-- NEW: BackUp action client
        self.backup_client = ActionClient(self, BackUp, 'backup', callback_group=self.cb_group)

        # ---------- Start sequence ----------
        self.create_timer(0.2, self._startup_once)
        self._did_startup = False

        self.get_logger().info('✅ hs_auto_nav_pick_carry started. (Nav2 + MoveIt)')

    # ---------- Callbacks ----------

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_pose = msg

    def _startup_once(self):
        if self._did_startup:
            return
        self._did_startup = True

        self.get_logger().info('⏳ Waiting for Nav2 action server…')
        self.nav_client.wait_for_server()

        # Pick MoveGroup action server (prefer /move_action if present)
        if self.mg_client_move_action.wait_for_server(timeout_sec=2.0):
            self._movegroup_client_active = self.mg_client_move_action
            self.get_logger().info('✅ MoveGroup server ready at "/move_action".')
        elif self.mg_client_move_group.wait_for_server(timeout_sec=2.0):
            self._movegroup_client_active = self.mg_client_move_group
            self.get_logger().info('✅ MoveGroup server ready at "/move_group".')
        else:
            self.get_logger().warn('⚠️ No MoveGroup action server yet; arm stages will wait until available.')

        # Navigate to the pick waypoint immediately
        self._go_to_pick_waypoint()

    def _event_cb(self, msg: String):
        # Expect JSON with counts.total increasing + box info
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        total = int(data.get('counts', {}).get('total', 0))
        if total <= self._last_box_total:
            return
        self._last_box_total = total

        if self.busy:
            self.get_logger().info('ℹ️ New box event received but node is busy; ignoring.')
            return

        # Determine box size (prefer category, else use weight thresholds)
        size = self._classify_box(data)
        self.get_logger().info(f'✅ New box event (total={total}) → type="{size}". Running pick→carry→place…')

        self.busy = True
        try:
            if not self._already_close_to_pick():
                self._go_to_pick_waypoint()

            time.sleep(self.STABILIZE_S)
            self._run_pick_carry_place(size)
        finally:
            self.busy = False

    # ---------- Navigation ----------

    def _already_close_to_target(self, tx: float, ty: float, tyaw: float) -> bool:
        if self._amcl_pose is None:
            return False
        p = self._amcl_pose.pose.pose.position
        o = self._amcl_pose.pose.pose.orientation
        yaw_now = quat_to_yaw(o.x, o.y, o.z, o.w)
        dx, dy = tx - p.x, ty - p.y
        xy = math.hypot(dx, dy)
        dyaw = abs((tyaw - yaw_now + math.pi) % (2*math.pi) - math.pi)
        return (xy <= self.TOL_XY) and (dyaw <= self.TOL_YAW)

    def _already_close_to_pick(self) -> bool:
        close = self._already_close_to_target(self.PICK_X, self.PICK_Y, self.PICK_YAW)
        if close:
            self.get_logger().info('✅ [NAV] Already at pick spot.')
        return close

    def _navigate_to(self, x: float, y: float, yaw: float, label: str):
        t0 = time.time()
        while rclpy.ok() and self._amcl_pose is None and (time.time() - t0) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self._already_close_to_target(x, y, yaw):
            self.get_logger().info(f'✅ [NAV] Already at {label} waypoint.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = make_map_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'✅ [NAV] Driving to {label} waypoint: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°'
        )

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_fb)
        rclpy.spin_until_future_complete(self, send_future)
        self._nav_handle = send_future.result()
        if not self._nav_handle or not self._nav_handle.accepted:
            self.get_logger().warn(f'⚠️ [NAV] {label} goal not accepted.')
            return

        res_future = self._nav_handle.get_result_async()
        t0 = time.time()
        while rclpy.ok():
            if res_future.done():
                res = res_future.result()
                status = getattr(res, 'status', None)
                if status == 4:
                    self.get_logger().info(f'✅ [NAV] {label} result status=4 (SUCCEEDED).')
                else:
                    self.get_logger().info(f'ℹ️ [NAV] {label} result status={status}.')
                break
            if self._already_close_to_target(x, y, yaw):
                self.get_logger().info(f'✅ [NAV] {label}: close enough; proceeding.')
                break
            if (time.time() - t0) > self.NAV_TIMEOUT_S:
                self.get_logger().warn(f'⚠️ [NAV] Timeout navigating to {label} waypoint.')
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def _go_to_pick_waypoint(self):
        self._navigate_to(self.PICK_X, self.PICK_Y, self.PICK_YAW, 'pick')

    def _go_to_place_waypoint(self):
        self._navigate_to(self.PLACE_X, self.PLACE_Y, self.PLACE_YAW, 'place')

    # final waypoint helper
    def _go_to_final_waypoint(self):
        self._navigate_to(self.FINAL_X, self.FINAL_Y, self.FINAL_YAW, 'final')

    def _nav_fb(self, fb):
        try:
            d = fb.feedback.distance_remaining
            self.get_logger().info(f'🧭 [NAV] distance remaining: {d:.2f} m')
        except Exception:
            pass

    # ---------- MoveIt / Arm ----------

    def _ensure_movegroup_ready(self) -> bool:
        if self._movegroup_client_active is not None:
            return True
        if self.mg_client_move_action.wait_for_server(timeout_sec=2.0):
            self._movegroup_client_active = self.mg_client_move_action
            self.get_logger().info('✅ MoveGroup now ready at "/move_action".')
            return True
        if self.mg_client_move_group.wait_for_server(timeout_sec=2.0):
            self._movegroup_client_active = self.mg_client_move_group
            self.get_logger().info('✅ MoveGroup now ready at "/move_group".')
            return True
        self.get_logger().warn('⚠️ MoveGroup not available yet.')
        return False

    def _plan_to_joint_targets(self, joints_rad: List[float]) -> bool:
        if not self._ensure_movegroup_ready():
            return False
        if len(joints_rad) != len(self.JOINT_NAMES):
            self.get_logger().error(f'❌ Joint target length {len(joints_rad)} != {len(self.JOINT_NAMES)}.')
            return False

        req = MotionPlanRequest()
        req.group_name = self.PLANNING_GROUP
        req.num_planning_attempts = 3
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal = Constraints()
        for name, pos in zip(self.JOINT_NAMES, joints_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.JOINT_TOL_RAD
            jc.tolerance_below = self.JOINT_TOL_RAD
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        req.goal_constraints = [goal]

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.request.start_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.replan_attempts = 0
        goal_msg.planning_options.replan_delay = 0.0

        client = self._movegroup_client_active
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        self._mg_handle = send_future.result()
        if not self._mg_handle or not self._mg_handle.accepted:
            self.get_logger().warn('⚠️ [ARM] MoveGroup goal rejected.')
            return False

        res_future = self._mg_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()

        code = None
        try:
            code = result.result.error_code.val
        except Exception:
            pass

        if code == 1:
            self.get_logger().info('✅ [ARM] MoveIt SUCCESS (code=1).')
            ok = True
        else:
            self.get_logger().warn(f'⚠️ [ARM] MoveIt result code={code} (1=SUCCESS).')
            ok = False

        return ok

    # ---------- Box classification + sequence ----------

    def _classify_box(self, data: Dict) -> str:
        box = data.get('box', {}) if isinstance(data, dict) else {}
        cat = str(box.get('category', '')).strip().lower()
        if cat in ('small', 'medium', 'big'):
            return cat

        wtxt = str(box.get('weight_raw', '')).lower()
        m = re.search(r'([0-9]+(?:\.[0-9]+)?)', wtxt)
        if m:
            w = float(m.group(1))
            if 0.0 < w <= self.SMALL_LE:
                return 'small'
            if self.SMALL_LE < w <= self.MEDIUM_LE:
                return 'medium'
            return 'big'
        return 'small'

    def _get_targets_for_size(self, size: str) -> Tuple[List[float], List[float]]:
        if size == 'small':
            return self.PICK_SMALL, self.CARRY_SMALL
        if size == 'medium':
            return self.PICK_MED, self.CARRY_MED
        return self.PICK_BIG, self.CARRY_BIG

    # ---------- BackUp helper ----------

    def _back_up_distance(self, dist_m: float, speed_mps: float = 0.3) -> None:
        """Send a BackUp action to reverse straight by dist_m."""
        if not self.backup_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('⚠️ [NAV] BackUp action server not available.')
            return
        goal = BackUp.Goal()
        # Nav2 BackUp uses a target Point; x is the displacement (negative = reverse)
        goal.target.x = -abs(float(dist_m))
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(speed_mps)
        # Reasonable timeout based on distance/speed + buffer
        goal.time_allowance = Duration(sec=max(5, int(abs(dist_m) / max(speed_mps, 1e-3) + 5)))

        send_future = self.backup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn('⚠️ [NAV] BackUp goal not accepted.')
            return
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info('✅ [NAV] BackUp completed.')

    # ---------- Sequence ----------

    def _run_pick_carry_place(self, size: str):
        """
        Stages:
            pick(size) -> wait -> carry(size) -> NAV to place -> wait -> place joints
            -> NAV to final -> (reverse to place) -> set carry -> NAV back to pick (ready)
        """
        pick_joints, carry_joints = self._get_targets_for_size(size)
        self._last_carry_joints = carry_joints        # <-- NEW (remember for later)

        # short, consistent delay right before starting the pick motion
        if self.DELAY_BEFORE_PICK > 0.0:
            self.get_logger().info(f'⏳ [ARM] Waiting {self.DELAY_BEFORE_PICK:.2f}s before pick…')
            time.sleep(self.DELAY_BEFORE_PICK)

        self.get_logger().info(f'🤖 [ARM] Executing stage: pick ({size})')
        ok = self._plan_to_joint_targets(pick_joints)
        if not ok:
            self.get_logger().warn('⚠️ [ARM] Stage "pick" failed; aborting sequence.')
            return

        time.sleep(self.STAGE_WAIT)

        self.get_logger().info(f'🤖 [ARM] Executing stage: carry ({size})')
        ok = self._plan_to_joint_targets(carry_joints)
        if not ok:
            self.get_logger().warn('⚠️ [ARM] Stage "carry" failed; aborting sequence.')
            return

        self._go_to_place_waypoint()
        time.sleep(self.STABILIZE_S)

        self.get_logger().info('🤖 [ARM] Executing stage: place (RViz joints)')
        ok = self._plan_to_joint_targets(self.PLACE_JOINTS)
        if not ok:
            self.get_logger().warn('⚠️ [ARM] Stage "place" failed; sequence ends here.')
            return

        # Move to your final waypoint
        self._go_to_final_waypoint()

        # --- 1s delay at FINAL waypoint (kept) ---
        time.sleep(1.0)

        # --- Reverse straight back from FINAL to PLACE distance ---
        #dist_back = math.hypot(self.FINAL_X - self.PLACE_X, self.FINAL_Y - self.PLACE_Y)
        #if dist_back > 0.02:
        #   self.get_logger().info(f'↩️ [NAV] Backing up {dist_back:.2f} m to place waypoint…')
        #   self._back_up_distance(dist_back, speed_mps=0.3)
	
	# --- Reverse by fixed backup distance ---
	dist_back = 0.5   # <-- Fixed reverse distance in meters (set your desired value)
	self.get_logger().info(f'↩️ [NAV] Backing up fixed {dist_back:.2f} m…')
	self._back_up_distance(dist_back, speed_mps=0.3)

        # Set arm to carry pose again (safety) before leaving the shelf
        if self._last_carry_joints:
            self.get_logger().info('🤖 [ARM] Re-applying carry pose before departing.')
            self._plan_to_joint_targets(self._last_carry_joints)

        # Return to pick/ready waypoint; keep your original settle delay
        self.get_logger().info('🤖 [NAV] Returning to pick/ready waypoint…')
        self._go_to_pick_waypoint()
        time.sleep(self.STABILIZE_S)
        self.get_logger().info('✅ Ready at pick waypoint for the next job.')

        self.get_logger().info('✅ [ARM] pick→carry→place→final→backup→carry→ready sequence completed.')

    # ---------- Cleanup helpers ----------
    def cancel_all_goals(self):
        try:
            if self._nav_handle:
                self._nav_handle.cancel_goal_async()
        except Exception:
            pass
        try:
            if self._mg_handle:
                self._mg_handle.cancel_goal_async()
        except Exception:
            pass


# --------------- main ---------------

def main():
    rclpy.init()
    node = HSAutoNavPickCarry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.get_logger().info('🛑 Ctrl+C received — stopping…')
        except Exception:
            pass
    finally:
        try:
            node.cancel_all_goals()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

