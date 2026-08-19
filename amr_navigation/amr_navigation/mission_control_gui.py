#!/usr/bin/env python3

import math
import os
import time
import yaml
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.action import ActionClient

from amr_interfaces.action import Patrol  # type: ignore
from amr_interfaces.msg import ObstacleStatus  # type: ignore
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class MissionControlGUI:

    def __init__(self):

        # --------------------------------------------------
        # ROS 2
        # --------------------------------------------------

        rclpy.init()

        self.node = rclpy.create_node('mission_control_gui')

        self.patrol_client = ActionClient(self.node, Patrol, '/patrol')

        self.odom_subscriber = self.node.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.obstacle_subscriber = self.node.create_subscription(
            msg_type=ObstacleStatus,
            topic='/obstacle_status',
            callback=self.obstacle_callback,
            qos_profile=10,
        )

        # --------------------------------------------------
        # Runtime state
        # --------------------------------------------------

        self.current_goal_handle = None
        self.mission_active = False

        self.mission_started_at = None
        self.mission_completed_at = None

        self.current_cycle = 0
        self.current_waypoint = 0
        self.completed_cycles = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.trajectory_points = []
        self.max_trajectory_points = 2500
        self.last_trail_x = None
        self.last_trail_y = None
        self.trail_min_step = 0.03

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.obstacle_detected = False

        self.last_odom_time = 0.0
        self.last_obstacle_time = 0.0

        # --------------------------------------------------
        # Load configuration
        # --------------------------------------------------

        self.config_file = os.path.expanduser(
            '~/amr_ws/src/autonomous-mobile-robot/amr_navigation/config/patrol.yaml'
        )

        self.waypoints = []
        self.cycles = 1
        self.load_config()

        # --------------------------------------------------
        # GUI
        # --------------------------------------------------

        self.root = tk.Tk()
        self.root.title('AMR Mission Control Dashboard')
        self.root.geometry('1080x760')
        self.root.minsize(980, 700)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.create_styles()
        self.create_widgets()

        self.update_mission_status('READY')
        self.refresh_dashboard()

    # ======================================================
    # Configuration
    # ======================================================

    def load_config(self):

        try:
            with open(self.config_file, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)

            patrol = config['patrol']
            self.cycles = int(patrol['cycles'])

            self.waypoints = []

            for waypoint in patrol['waypoints']:
                self.waypoints.append(
                    {
                        'x': float(waypoint['x']),
                        'y': float(waypoint['y']),
                        'theta': float(waypoint['theta']),
                    }
                )

        except Exception as error:
            messagebox.showerror(
                'Configuration Error',
                f'Could not load patrol.yaml:\n\n{error}',
            )
            self.cycles = 1
            self.waypoints = []

    # ======================================================
    # ROS callbacks
    # ======================================================

    def odom_callback(self, msg: Odometry):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_theta = yaw

        if self.last_trail_x is None or self.last_trail_y is None:
            self.trajectory_points.append((self.robot_x, self.robot_y))
            self.last_trail_x = self.robot_x
            self.last_trail_y = self.robot_y
        else:
            dx = self.robot_x - self.last_trail_x
            dy = self.robot_y - self.last_trail_y
            if math.hypot(dx, dy) >= self.trail_min_step:
                self.trajectory_points.append((self.robot_x, self.robot_y))
                self.last_trail_x = self.robot_x
                self.last_trail_y = self.robot_y

        if len(self.trajectory_points) > self.max_trajectory_points:
            self.trajectory_points = self.trajectory_points[-self.max_trajectory_points :]

        self.last_odom_time = time.time()

    def obstacle_callback(self, msg: ObstacleStatus):

        self.front_distance = float(msg.front_distance)
        self.left_distance = float(msg.left_distance)
        self.right_distance = float(msg.right_distance)
        self.obstacle_detected = bool(msg.obstacle_detected)

        self.last_obstacle_time = time.time()

    # ======================================================
    # GUI setup
    # ======================================================

    def create_styles(self):

        style = ttk.Style()

        style.configure('Dashboard.TLabelframe', padding=12)
        style.configure('KpiValue.TLabel', font=('TkDefaultFont', 13, 'bold'))
        style.configure('KpiTitle.TLabel', font=('TkDefaultFont', 10))
        style.configure('Headline.TLabel', font=('TkDefaultFont', 20, 'bold'))
        style.configure('Banner.TLabel', font=('TkDefaultFont', 14, 'bold'))

    def create_widgets(self):

        header = ttk.Frame(self.root)
        header.pack(fill='x', padx=14, pady=(12, 8))

        ttk.Label(
            header,
            text='AMR Mission Control',
            style='Headline.TLabel',
        ).pack(side='left')

        self.live_clock_label = ttk.Label(header, text='--:--:--')
        self.live_clock_label.pack(side='right')

        telemetry_row = ttk.Frame(self.root)
        telemetry_row.pack(fill='x', padx=14, pady=6)

        mission_frame = ttk.LabelFrame(
            telemetry_row,
            text='Mission Progress',
            style='Dashboard.TLabelframe',
        )
        mission_frame.pack(side='left', fill='both', expand=True, padx=(0, 8))

        robot_frame = ttk.LabelFrame(
            telemetry_row,
            text='Robot Status',
            style='Dashboard.TLabelframe',
        )
        robot_frame.pack(side='left', fill='both', expand=True, padx=(4, 8))

        obstacle_frame = ttk.LabelFrame(
            telemetry_row,
            text='Obstacle Status',
            style='Dashboard.TLabelframe',
        )
        obstacle_frame.pack(side='left', fill='both', expand=True, padx=(4, 0))

        # Mission KPIs
        self.cycle_value = self.kpi(mission_frame, 0, 'Cycle', '-- / --')
        self.waypoint_value = self.kpi(mission_frame, 1, 'Waypoint', '-- / --')
        self.distance_value = self.kpi(mission_frame, 2, 'Distance', '-- m')
        self.nav_state_value = self.kpi(mission_frame, 3, 'State', 'IDLE')

        # Robot KPIs
        self.robot_link_value = self.kpi(robot_frame, 0, 'Robot', 'OFFLINE')
        self.robot_state_value = self.kpi(robot_frame, 1, 'State', 'IDLE')
        self.robot_pose_value = self.kpi(robot_frame, 2, 'Position', 'x=0.00 y=0.00 th=0.00')
        self.mission_elapsed_value = self.kpi(robot_frame, 3, 'Mission Time', '--')

        # Obstacle KPIs
        self.front_value = self.kpi(obstacle_frame, 0, 'Front', '-- m')
        self.left_value = self.kpi(obstacle_frame, 1, 'Left', '-- m')
        self.right_value = self.kpi(obstacle_frame, 2, 'Right', '-- m')
        self.obstacle_flag_value = self.kpi(obstacle_frame, 3, 'Detected', 'NO')

        center_row = ttk.Frame(self.root)
        center_row.pack(fill='both', expand=True, padx=14, pady=(4, 8))

        left_panel = ttk.Frame(center_row)
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 8))

        table_frame = ttk.LabelFrame(
            left_panel,
            text='Mission Waypoints',
            style='Dashboard.TLabelframe',
        )
        table_frame.pack(side='top', fill='both', expand=True, pady=(0, 8))

        map_frame = ttk.LabelFrame(
            left_panel,
            text='Robot Trajectory',
            style='Dashboard.TLabelframe',
        )
        map_frame.pack(side='top', fill='both', expand=True)

        side_panel = ttk.Frame(center_row)
        side_panel.pack(side='left', fill='y')

        mission_actions = ttk.LabelFrame(
            side_panel,
            text='Mission Control',
            style='Dashboard.TLabelframe',
        )
        mission_actions.pack(fill='x', pady=(0, 8))

        completion_frame = ttk.LabelFrame(
            side_panel,
            text='Completion Summary',
            style='Dashboard.TLabelframe',
        )
        completion_frame.pack(fill='x')

        # Waypoint table
        columns = ('index', 'x', 'y', 'theta')
        self.tree = ttk.Treeview(
            table_frame,
            columns=columns,
            show='headings',
            height=14,
        )

        self.tree.heading('index', text='#')
        self.tree.heading('x', text='X')
        self.tree.heading('y', text='Y')
        self.tree.heading('theta', text='Theta')

        self.tree.column('index', width=50, anchor='center')
        self.tree.column('x', width=120, anchor='center')
        self.tree.column('y', width=120, anchor='center')
        self.tree.column('theta', width=120, anchor='center')

        scrollbar = ttk.Scrollbar(table_frame, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)

        self.tree.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        self.refresh_table()

        # Waypoint buttons
        waypoint_buttons = ttk.Frame(table_frame)
        waypoint_buttons.pack(fill='x', pady=(8, 0))

        ttk.Button(waypoint_buttons, text='Add Waypoint', command=self.add_waypoint).pack(
            side='left', padx=4
        )
        ttk.Button(waypoint_buttons, text='Edit Selected', command=self.edit_waypoint).pack(
            side='left', padx=4
        )
        ttk.Button(waypoint_buttons, text='Delete Selected', command=self.delete_waypoint).pack(
            side='left', padx=4
        )
        ttk.Button(waypoint_buttons, text='Reload YAML', command=self.reload_yaml).pack(
            side='left', padx=4
        )

        self.map_canvas = tk.Canvas(
            map_frame,
            height=260,
            background='#fcfcfc',
            highlightthickness=1,
            highlightbackground='#d0d0d0',
        )
        self.map_canvas.pack(fill='both', expand=True)

        map_buttons = ttk.Frame(map_frame)
        map_buttons.pack(fill='x', pady=(8, 0))

        ttk.Button(map_buttons, text='Clear Trail', command=self.clear_trajectory).pack(
            side='left', padx=4
        )

        # Mission action controls
        ttk.Label(mission_actions, text='Patrol Cycles').pack(anchor='w', pady=(2, 2))

        self.cycles_var = tk.IntVar(value=self.cycles)
        self.cycles_spinbox = tk.Spinbox(
            mission_actions,
            from_=1,
            to=100,
            textvariable=self.cycles_var,
            width=12,
        )
        self.cycles_spinbox.pack(anchor='w', pady=(0, 10))

        self.start_button = ttk.Button(
            mission_actions,
            text='START PATROL',
            command=self.start_patrol,
        )
        self.start_button.pack(fill='x', pady=(2, 6))

        self.cancel_button = ttk.Button(
            mission_actions,
            text='CANCEL MISSION',
            command=self.cancel_patrol,
            state='disabled',
        )
        self.cancel_button.pack(fill='x')

        # Completion section
        self.complete_banner = ttk.Label(
            completion_frame,
            text='MISSION NOT STARTED',
            style='Banner.TLabel',
        )
        self.complete_banner.pack(anchor='w', pady=(0, 8))

        self.completed_cycles_label = ttk.Label(
            completion_frame,
            text='Cycles completed: --',
        )
        self.completed_cycles_label.pack(anchor='w')

        self.completed_waypoints_label = ttk.Label(
            completion_frame,
            text='Waypoints completed: -- / --',
        )
        self.completed_waypoints_label.pack(anchor='w', pady=(4, 0))

        status_frame = ttk.LabelFrame(self.root, text='System Status', style='Dashboard.TLabelframe')
        status_frame.pack(fill='x', padx=14, pady=(2, 14))

        self.status_label = ttk.Label(status_frame, text='READY', font=('TkDefaultFont', 12, 'bold'))
        self.status_label.pack(anchor='w')

    def kpi(self, parent, row, title, value):

        title_label = ttk.Label(parent, text=title, style='KpiTitle.TLabel')
        title_label.grid(row=row, column=0, sticky='w', pady=(2, 0))

        value_label = ttk.Label(parent, text=value, style='KpiValue.TLabel')
        value_label.grid(row=row, column=1, sticky='w', padx=(10, 0), pady=(2, 0))

        return value_label

    # ======================================================
    # Mission and table helpers
    # ======================================================

    def refresh_table(self):

        for item in self.tree.get_children():
            self.tree.delete(item)

        for index, waypoint in enumerate(self.waypoints, start=1):
            self.tree.insert(
                '',
                'end',
                values=(
                    index,
                    f"{waypoint['x']:.3f}",
                    f"{waypoint['y']:.3f}",
                    f"{waypoint['theta']:.3f}",
                ),
            )

    def add_waypoint(self):

        self.waypoints.append({'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.refresh_table()

    def edit_waypoint(self):

        selected = self.tree.selection()

        if not selected:
            messagebox.showwarning('No Selection', 'Select a waypoint first.')
            return

        index = self.tree.index(selected[0])
        waypoint = self.waypoints[index]

        self.open_waypoint_editor(index, waypoint)

    def open_waypoint_editor(self, index, waypoint):

        window = tk.Toplevel(self.root)
        window.title(f'Edit Waypoint {index + 1}')
        window.geometry('320x250')

        fields = {}

        for row, name in enumerate(['x', 'y', 'theta']):
            ttk.Label(window, text=name.upper()).grid(row=row, column=0, padx=10, pady=10)

            entry = ttk.Entry(window)
            entry.insert(0, str(waypoint[name]))
            entry.grid(row=row, column=1, padx=10, pady=10)

            fields[name] = entry

        def save():
            try:
                waypoint['x'] = float(fields['x'].get())
                waypoint['y'] = float(fields['y'].get())
                waypoint['theta'] = float(fields['theta'].get())

                self.refresh_table()
                window.destroy()

            except ValueError:
                messagebox.showerror('Invalid Value', 'X, Y and Theta must be numbers.')

        ttk.Button(window, text='Save', command=save).grid(
            row=3,
            column=0,
            columnspan=2,
            pady=15,
        )

    def delete_waypoint(self):

        selected = self.tree.selection()

        if not selected:
            messagebox.showwarning('No Selection', 'Select a waypoint first.')
            return

        index = self.tree.index(selected[0])
        del self.waypoints[index]

        self.refresh_table()

    def reload_yaml(self):

        self.load_config()
        self.cycles_var.set(self.cycles)
        self.refresh_table()

        self.update_mission_status('Configuration reloaded from YAML.')

    def clear_trajectory(self):

        self.trajectory_points = []
        self.last_trail_x = self.robot_x
        self.last_trail_y = self.robot_y
        self.trajectory_points.append((self.robot_x, self.robot_y))

        self.update_mission_status('Trajectory trail cleared.')

    # ======================================================
    # Patrol action
    # ======================================================

    def start_patrol(self):

        if not self.waypoints:
            messagebox.showerror('Invalid Mission', 'At least one waypoint is required.')
            return

        cycles = self.cycles_var.get()

        if cycles <= 0:
            messagebox.showerror('Invalid Mission', 'Patrol cycles must be greater than zero.')
            return

        self.update_mission_status('Waiting for Patrol Server...')
        self.start_button.config(state='disabled')
        self.cancel_button.config(state='normal')

        self.mission_active = True
        self.mission_started_at = time.time()
        self.mission_completed_at = None
        self.current_cycle = 0
        self.current_waypoint = 0
        self.completed_cycles = 0
        self.complete_banner.config(text='MISSION RUNNING')
        self.completed_cycles_label.config(text='Cycles completed: 0')
        self.completed_waypoints_label.config(text=f'Waypoints completed: 0 / {len(self.waypoints)}')

        goal = Patrol.Goal()
        goal.waypoints = []

        for waypoint in self.waypoints:
            pose = Pose2D()
            pose.x = float(waypoint['x'])
            pose.y = float(waypoint['y'])
            pose.theta = float(waypoint['theta'])
            goal.waypoints.append(pose)

        goal.patrol_cycles = cycles

        if not self.patrol_client.wait_for_server(timeout_sec=3.0):
            self.update_mission_status('ERROR: Patrol server unavailable.')
            self.reset_buttons()
            self.mission_active = False
            self.complete_banner.config(text='MISSION FAILED')
            return

        self.update_mission_status('Sending patrol mission...')

        future = self.patrol_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()

        except Exception as error:
            self.update_mission_status(f'ERROR: {error}')
            self.reset_buttons()
            self.mission_active = False
            self.complete_banner.config(text='MISSION FAILED')
            return

        if not goal_handle.accepted:
            self.update_mission_status('Patrol mission rejected.')
            self.reset_buttons()
            self.mission_active = False
            self.complete_banner.config(text='MISSION REJECTED')
            return

        self.update_mission_status('Patrol mission running...')

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.current_cycle = int(feedback.current_cycle)
        self.current_waypoint = int(feedback.current_waypoint)

        self.update_mission_status(
            f'Running - Cycle {self.current_cycle}, Waypoint {self.current_waypoint}'
        )

    def result_callback(self, future):

        try:
            result = future.result().result

        except Exception as error:
            self.update_mission_status(f'ERROR: {error}')
            self.reset_buttons()
            self.mission_active = False
            self.complete_banner.config(text='MISSION FAILED')
            return

        self.mission_active = False
        self.mission_completed_at = time.time()
        self.completed_cycles = int(result.cycles_completed)

        total_waypoints = len(self.waypoints)

        if result.success:
            self.update_mission_status(
                f'MISSION COMPLETE | Cycles completed: {result.cycles_completed}'
            )
            self.complete_banner.config(text='MISSION COMPLETE')
            self.completed_waypoints_label.config(
                text=f'Waypoints completed: {total_waypoints} / {total_waypoints}'
            )
        else:
            self.update_mission_status(f'FAILED - {result.message}')
            self.complete_banner.config(text='MISSION FAILED')
            self.completed_waypoints_label.config(
                text=f'Waypoints completed: {self.current_waypoint} / {total_waypoints}'
            )

        self.completed_cycles_label.config(text=f'Cycles completed: {result.cycles_completed}')

        self.reset_buttons()

    def cancel_patrol(self):

        if self.current_goal_handle is None:
            return

        self.update_mission_status('Cancelling patrol...')

        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, _future):

        self.update_mission_status('Patrol cancellation requested.')

        self.mission_active = False
        self.mission_completed_at = time.time()
        self.complete_banner.config(text='MISSION CANCELLED')

        self.reset_buttons()

    # ======================================================
    # Derived state and dashboard refresh
    # ======================================================

    def compute_navigation_state(self):

        if not self.mission_active:
            return 'IDLE'

        if self.obstacle_detected:
            return 'AVOID_OBSTACLE'

        if self.current_waypoint > 0:
            return 'FOLLOW_GOAL'

        return 'WAITING_GOAL'

    def compute_distance_to_current_waypoint(self):

        if not self.mission_active:
            return None

        if self.current_waypoint <= 0 or self.current_waypoint > len(self.waypoints):
            return None

        target = self.waypoints[self.current_waypoint - 1]

        dx = target['x'] - self.robot_x
        dy = target['y'] - self.robot_y

        return math.hypot(dx, dy)

    def format_distance(self, value):

        if value is None or not math.isfinite(value):
            return '-- m'

        return f'{value:.2f} m'

    def robot_online(self):

        now = time.time()
        if self.last_odom_time == 0.0:
            return False

        return (now - self.last_odom_time) <= 1.5

    def obstacle_online(self):

        now = time.time()
        if self.last_obstacle_time == 0.0:
            return False

        return (now - self.last_obstacle_time) <= 1.5

    def map_to_canvas(self, x, y, min_x, min_y, scale, pad, canvas_h):

        canvas_x = pad + (x - min_x) * scale
        canvas_y = canvas_h - (pad + (y - min_y) * scale)
        return canvas_x, canvas_y

    def draw_trajectory_map(self):

        if not hasattr(self, 'map_canvas'):
            return

        self.map_canvas.delete('all')

        self.map_canvas.update_idletasks()
        canvas_w = max(300, self.map_canvas.winfo_width())
        canvas_h = max(200, self.map_canvas.winfo_height())
        pad = 24

        points = list(self.trajectory_points)
        points.append((self.robot_x, self.robot_y))
        waypoint_points = [(wp['x'], wp['y']) for wp in self.waypoints]
        all_points = points + waypoint_points

        if not all_points:
            self.map_canvas.create_text(
                canvas_w / 2,
                canvas_h / 2,
                text='Waiting for robot and waypoint data...',
                fill='#666666',
            )
            return

        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)

        span_x = max(0.5, max_x - min_x)
        span_y = max(0.5, max_y - min_y)

        min_x -= 0.15 * span_x
        max_x += 0.15 * span_x
        min_y -= 0.15 * span_y
        max_y += 0.15 * span_y

        scale_x = (canvas_w - (2 * pad)) / max(1e-9, (max_x - min_x))
        scale_y = (canvas_h - (2 * pad)) / max(1e-9, (max_y - min_y))
        scale = min(scale_x, scale_y)

        self.map_canvas.create_rectangle(pad, pad, canvas_w - pad, canvas_h - pad, outline='#d8d8d8')

        # Draw axis lines when visible in current map window.
        if min_x <= 0.0 <= max_x:
            x0, y0 = self.map_to_canvas(0.0, min_y, min_x, min_y, scale, pad, canvas_h)
            x1, y1 = self.map_to_canvas(0.0, max_y, min_x, min_y, scale, pad, canvas_h)
            self.map_canvas.create_line(x0, y0, x1, y1, fill='#e2e2e2', dash=(4, 4))

        if min_y <= 0.0 <= max_y:
            x0, y0 = self.map_to_canvas(min_x, 0.0, min_x, min_y, scale, pad, canvas_h)
            x1, y1 = self.map_to_canvas(max_x, 0.0, min_x, min_y, scale, pad, canvas_h)
            self.map_canvas.create_line(x0, y0, x1, y1, fill='#e2e2e2', dash=(4, 4))

        if len(points) >= 2:
            canvas_trail = []
            for trail_x, trail_y in points:
                px, py = self.map_to_canvas(trail_x, trail_y, min_x, min_y, scale, pad, canvas_h)
                canvas_trail.extend([px, py])
            self.map_canvas.create_line(*canvas_trail, fill='#2f6db5', width=2)

        for index, waypoint in enumerate(self.waypoints, start=1):
            wx, wy = self.map_to_canvas(waypoint['x'], waypoint['y'], min_x, min_y, scale, pad, canvas_h)
            self.map_canvas.create_oval(wx - 4, wy - 4, wx + 4, wy + 4, fill='#d9534f', outline='')
            self.map_canvas.create_text(wx + 10, wy - 8, text=f'W{index}', anchor='w', fill='#444444')

        rx, ry = self.map_to_canvas(self.robot_x, self.robot_y, min_x, min_y, scale, pad, canvas_h)
        self.map_canvas.create_oval(rx - 6, ry - 6, rx + 6, ry + 6, fill='#2ca25f', outline='')

        heading_length = 16
        hx = rx + heading_length * math.cos(self.robot_theta)
        hy = ry - heading_length * math.sin(self.robot_theta)
        self.map_canvas.create_line(rx, ry, hx, hy, fill='#1c7a47', width=2)

        self.map_canvas.create_text(
            pad + 2,
            pad + 2,
            anchor='nw',
            fill='#555555',
            text='Trajectory: blue | Robot: green | Waypoints: red',
        )

    def refresh_dashboard(self):

        now = time.time()
        self.live_clock_label.config(text=time.strftime('%H:%M:%S', time.localtime(now)))

        cycle_target = self.cycles_var.get() if self.cycles_var.get() > 0 else 0

        cycle_value = '-- / --'
        if self.mission_active:
            cycle_value = f'{max(self.current_cycle, 1)} / {cycle_target}'
        elif self.completed_cycles > 0:
            cycle_value = f'{self.completed_cycles} / {cycle_target}'

        waypoint_total = len(self.waypoints)
        waypoint_value = '-- / --'
        if waypoint_total > 0:
            if self.mission_active and self.current_waypoint > 0:
                waypoint_value = f'{self.current_waypoint} / {waypoint_total}'
            elif self.completed_cycles > 0:
                waypoint_value = f'{waypoint_total} / {waypoint_total}'
            else:
                waypoint_value = f'0 / {waypoint_total}'

        distance = self.compute_distance_to_current_waypoint()
        nav_state = self.compute_navigation_state()

        self.cycle_value.config(text=cycle_value)
        self.waypoint_value.config(text=waypoint_value)
        self.distance_value.config(text=self.format_distance(distance))
        self.nav_state_value.config(text=nav_state)

        online_text = 'ONLINE' if self.robot_online() else 'OFFLINE'
        self.robot_link_value.config(text=f'● {online_text}')
        self.robot_state_value.config(text=nav_state)
        self.robot_pose_value.config(
            text=f'x={self.robot_x:.2f}, y={self.robot_y:.2f}, th={self.robot_theta:.2f}'
        )

        elapsed = '--'
        if self.mission_started_at is not None:
            end_time = now if self.mission_active else (self.mission_completed_at or now)
            seconds = int(max(0.0, end_time - self.mission_started_at))
            elapsed = f'{seconds // 60:02d}:{seconds % 60:02d}'
        self.mission_elapsed_value.config(text=elapsed)

        front_display = self.format_distance(self.front_distance) if self.obstacle_online() else '-- m'
        left_display = self.format_distance(self.left_distance) if self.obstacle_online() else '-- m'
        right_display = self.format_distance(self.right_distance) if self.obstacle_online() else '-- m'

        self.front_value.config(text=front_display)
        self.left_value.config(text=left_display)
        self.right_value.config(text=right_display)

        obstacle_text = 'YES' if (self.obstacle_online() and self.obstacle_detected) else 'NO'
        self.obstacle_flag_value.config(text=obstacle_text)

        self.draw_trajectory_map()

        self.root.after(150, self.refresh_dashboard)

    # ======================================================
    # Status and control helpers
    # ======================================================

    def update_mission_status(self, text):

        self.status_label.config(text=text)
        self.root.update_idletasks()

    def reset_buttons(self):

        self.start_button.config(state='normal')
        self.cancel_button.config(state='disabled')

    # ======================================================
    # ROS spinning
    # ======================================================

    def ros_spin(self):

        rclpy.spin_once(self.node, timeout_sec=0.01)

        if self.root.winfo_exists():
            self.root.after(10, self.ros_spin)

    # ======================================================
    # Close
    # ======================================================

    def close(self):

        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()

        self.root.destroy()

    # ======================================================
    # Run
    # ======================================================

    def run(self):

        self.root.after(10, self.ros_spin)
        self.root.mainloop()


def main():

    app = MissionControlGUI()
    app.run()


if __name__ == '__main__':
    main()
