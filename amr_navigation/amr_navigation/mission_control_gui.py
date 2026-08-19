#!/usr/bin/env python3

import os
import yaml
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from amr_interfaces.action import Patrol # type: ignore


class MissionControlGUI:

    def __init__(self):

        # --------------------------------------------------
        # ROS 2
        # --------------------------------------------------

        rclpy.init()

        self.node = rclpy.create_node(
            'mission_control_gui'
        )

        self.patrol_client = ActionClient(
            self.node,
            Patrol,
            '/patrol'
        )

        # --------------------------------------------------
        # Load configuration
        # --------------------------------------------------

        self.config_file = os.path.expanduser(
            '~/amr_ws/src/autonomous-mobile-robot/amr_navigation/config/patrol.yaml'
        )

        self.waypoints = []
        self.load_config()

        # --------------------------------------------------
        # GUI
        # --------------------------------------------------

        self.root = tk.Tk()

        self.root.title(
            'AMR Mission Control'
        )

        self.root.geometry(
            '750x600'
        )

        self.root.protocol(
            'WM_DELETE_WINDOW',
            self.close
        )

        self.create_widgets()

        self.update_status(
            'READY'
        )

    # ======================================================
    # Configuration
    # ======================================================

    def load_config(self):

        try:

            with open(
                self.config_file,
                'r'
            ) as file:

                config = yaml.safe_load(file)

            patrol = config['patrol']

            self.cycles = patrol['cycles']

            self.waypoints = []

            for waypoint in patrol['waypoints']:

                self.waypoints.append({
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'theta': waypoint['theta']
                })

        except Exception as e:

            messagebox.showerror(
                'Configuration Error',
                f'Could not load patrol.yaml:\n\n{e}'
            )

            self.cycles = 1
            self.waypoints = []

    # ======================================================
    # GUI
    # ======================================================

    def create_widgets(self):

        title = ttk.Label(
            self.root,
            text='AMR Mission Control',
            font=('Arial', 20, 'bold')
        )

        title.pack(
            pady=15
        )

        # --------------------------------------------------
        # Waypoint table
        # --------------------------------------------------

        table_frame = ttk.Frame(
            self.root
        )

        table_frame.pack(
            fill='both',
            expand=True,
            padx=20
        )

        columns = (
            'index',
            'x',
            'y',
            'theta'
        )

        self.tree = ttk.Treeview(
            table_frame,
            columns=columns,
            show='headings',
            height=12
        )

        self.tree.heading(
            'index',
            text='#'
        )

        self.tree.heading(
            'x',
            text='X'
        )

        self.tree.heading(
            'y',
            text='Y'
        )

        self.tree.heading(
            'theta',
            text='Theta'
        )

        self.tree.column(
            'index',
            width=50,
            anchor='center'
        )

        self.tree.column(
            'x',
            width=150,
            anchor='center'
        )

        self.tree.column(
            'y',
            width=150,
            anchor='center'
        )

        self.tree.column(
            'theta',
            width=150,
            anchor='center'
        )

        scrollbar = ttk.Scrollbar(
            table_frame,
            orient='vertical',
            command=self.tree.yview
        )

        self.tree.configure(
            yscrollcommand=scrollbar.set
        )

        self.tree.pack(
            side='left',
            fill='both',
            expand=True
        )

        scrollbar.pack(
            side='right',
            fill='y'
        )

        self.refresh_table()

        # --------------------------------------------------
        # Waypoint buttons
        # --------------------------------------------------

        waypoint_buttons = ttk.Frame(
            self.root
        )

        waypoint_buttons.pack(
            pady=10
        )

        ttk.Button(
            waypoint_buttons,
            text='Add Waypoint',
            command=self.add_waypoint
        ).pack(
            side='left',
            padx=5
        )

        ttk.Button(
            waypoint_buttons,
            text='Edit Selected',
            command=self.edit_waypoint
        ).pack(
            side='left',
            padx=5
        )

        ttk.Button(
            waypoint_buttons,
            text='Delete Selected',
            command=self.delete_waypoint
        ).pack(
            side='left',
            padx=5
        )

        ttk.Button(
            waypoint_buttons,
            text='Reload YAML',
            command=self.reload_yaml
        ).pack(
            side='left',
            padx=5
        )

        # --------------------------------------------------
        # Cycles
        # --------------------------------------------------

        cycles_frame = ttk.Frame(
            self.root
        )

        cycles_frame.pack(
            pady=10
        )

        ttk.Label(
            cycles_frame,
            text='Patrol Cycles:'
        ).pack(
            side='left',
            padx=5
        )

        self.cycles_var = tk.IntVar(
            value=self.cycles
        )

        self.cycles_spinbox = tk.Spinbox(
            cycles_frame,
            from_=1,
            to=100,
            textvariable=self.cycles_var,
            width=8
        )

        self.cycles_spinbox.pack(
            side='left'
        )

        # --------------------------------------------------
        # Mission buttons
        # --------------------------------------------------

        mission_frame = ttk.Frame(
            self.root
        )

        mission_frame.pack(
            pady=15
        )

        self.start_button = ttk.Button(
            mission_frame,
            text='START PATROL',
            command=self.start_patrol
        )

        self.start_button.pack(
            side='left',
            padx=10
        )

        self.cancel_button = ttk.Button(
            mission_frame,
            text='CANCEL',
            command=self.cancel_patrol,
            state='disabled'
        )

        self.cancel_button.pack(
            side='left',
            padx=10
        )

        # --------------------------------------------------
        # Status
        # --------------------------------------------------

        status_frame = ttk.LabelFrame(
            self.root,
            text='Mission Status'
        )

        status_frame.pack(
            fill='x',
            padx=20,
            pady=10
        )

        self.status_label = ttk.Label(
            status_frame,
            text='READY',
            font=('Arial', 12, 'bold')
        )

        self.status_label.pack(
            pady=10
        )

    # ======================================================
    # Table
    # ======================================================

    def refresh_table(self):

        for item in self.tree.get_children():

            self.tree.delete(item)

        for index, waypoint in enumerate(
            self.waypoints,
            start=1
        ):

            self.tree.insert(
                '',
                'end',
                values=(
                    index,
                    f"{waypoint['x']:.3f}",
                    f"{waypoint['y']:.3f}",
                    f"{waypoint['theta']:.3f}"
                )
            )

    # ======================================================
    # Waypoint editing
    # ======================================================

    def add_waypoint(self):

        self.waypoints.append({
            'x': float(0.0),
            'y': float(0.0),
            'theta': float(0.0)
        })

        self.refresh_table()

    def edit_waypoint(self):

        selected = self.tree.selection()

        if not selected:

            messagebox.showwarning(
                'No Selection',
                'Select a waypoint first.'
            )

            return

        index = self.tree.index(
            selected[0]
        )

        waypoint = self.waypoints[index]

        self.open_waypoint_editor(
            index,
            waypoint
        )

    def open_waypoint_editor(
        self,
        index,
        waypoint
    ):

        window = tk.Toplevel(
            self.root
        )

        window.title(
            f'Edit Waypoint {index + 1}'
        )

        window.geometry(
            '300x250'
        )

        fields = {}

        for row, name in enumerate(
            ['x', 'y', 'theta']
        ):

            ttk.Label(
                window,
                text=name.upper()
            ).grid(
                row=row,
                column=0,
                padx=10,
                pady=10
            )

            entry = ttk.Entry(
                window
            )

            entry.insert(
                0,
                str(waypoint[name])
            )

            entry.grid(
                row=row,
                column=1,
                padx=10,
                pady=10
            )

            fields[name] = entry

        def save():

            try:

                waypoint['x'] = float(
                    fields['x'].get()
                )

                waypoint['y'] = float(
                    fields['y'].get()
                )

                waypoint['theta'] = float(
                    fields['theta'].get()
                )

                self.refresh_table()

                window.destroy()

            except ValueError:

                messagebox.showerror(
                    'Invalid Value',
                    'X, Y and Theta must be numbers.'
                )

        ttk.Button(
            window,
            text='Save',
            command=save
        ).grid(
            row=3,
            column=0,
            columnspan=2,
            pady=15
        )

    def delete_waypoint(self):

        selected = self.tree.selection()

        if not selected:

            messagebox.showwarning(
                'No Selection',
                'Select a waypoint first.'
            )

            return

        index = self.tree.index(
            selected[0]
        )

        del self.waypoints[index]

        self.refresh_table()

    # ======================================================
    # YAML
    # ======================================================

    def reload_yaml(self):

        self.load_config()

        self.cycles_var.set(
            self.cycles
        )

        self.refresh_table()

        self.update_status(
            'Configuration reloaded.'
        )

    # ======================================================
    # Patrol Action
    # ======================================================

    def start_patrol(self):

        if not self.waypoints:

            messagebox.showerror(
                'Invalid Mission',
                'At least one waypoint is required.'
            )

            return

        cycles = self.cycles_var.get()

        if cycles <= 0:

            messagebox.showerror(
                'Invalid Mission',
                'Patrol cycles must be greater than zero.'
            )

            return

        self.update_status(
            'Waiting for Patrol Server...'
        )

        self.start_button.config(
            state='disabled'
        )

        self.cancel_button.config(
            state='normal'
        )

        # --------------------------------------------------
        # Create action goal
        # --------------------------------------------------

        goal = Patrol.Goal()

        goal.waypoints = []

        for waypoint in self.waypoints:

            pose = Pose2D()

            pose.x = float(waypoint['x'])
            pose.y = float(waypoint['y'])
            pose.theta = float(waypoint['theta'])

            goal.waypoints.append(
                pose
            )

        goal.patrol_cycles = cycles

        # --------------------------------------------------
        # Wait for server
        # --------------------------------------------------

        if not self.patrol_client.wait_for_server(
            timeout_sec=3.0
        ):

            self.update_status(
                'ERROR: Patrol server unavailable.'
            )

            self.reset_buttons()

            return

        self.update_status(
            'Sending patrol mission...'
        )

        future = self.patrol_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ======================================================
    # Goal response
    # ======================================================

    def goal_response_callback(
        self,
        future
    ):

        try:

            goal_handle = future.result()

        except Exception as e:

            self.update_status(
                f'ERROR: {e}'
            )

            self.reset_buttons()

            return

        if not goal_handle.accepted:

            self.update_status(
                'Patrol mission rejected.'
            )

            self.reset_buttons()

            return

        self.update_status(
            'Patrol mission running...'
        )

        self.current_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    # ======================================================
    # Feedback
    # ======================================================

    def feedback_callback(
        self,
        feedback_msg
    ):

        feedback = feedback_msg.feedback

        self.update_status(
            f'Running — '
            f'Cycle {feedback.current_cycle}, '
            f'Waypoint {feedback.current_waypoint}'
        )

    # ======================================================
    # Result
    # ======================================================

    def result_callback(
        self,
        future
    ):

        try:

            result = future.result().result

        except Exception as e:

            self.update_status(
                f'ERROR: {e}'
            )

            self.reset_buttons()

            return

        if result.success:

            self.update_status(
                f'COMPLETED — '
                f'{result.cycles_completed} cycle(s)'
            )

        else:

            self.update_status(
                f'FAILED — {result.message}'
            )

        self.reset_buttons()

    # ======================================================
    # Cancel
    # ======================================================

    def cancel_patrol(self):

        if not hasattr(
            self,
            'current_goal_handle'
        ):

            return

        self.update_status(
            'Cancelling patrol...'
        )

        future = (
            self.current_goal_handle
            .cancel_goal_async()
        )

        future.add_done_callback(
            self.cancel_callback
        )

    def cancel_callback(
        self,
        future
    ):

        self.update_status(
            'Patrol cancellation requested.'
        )

        self.reset_buttons()

    # ======================================================
    # Helpers
    # ======================================================

    def update_status(
        self,
        text
    ):

        self.status_label.config(
            text=text
        )

        self.root.update_idletasks()

    def reset_buttons(self):

        self.start_button.config(
            state='normal'
        )

        self.cancel_button.config(
            state='disabled'
        )

    # ======================================================
    # ROS spinning
    # ======================================================

    def ros_spin(self):

        rclpy.spin_once(
            self.node,
            timeout_sec=0.01
        )

        if self.root.winfo_exists():

            self.root.after(
                10,
                self.ros_spin
            )

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

        self.root.after(
            10,
            self.ros_spin
        )

        self.root.mainloop()


def main():

    app = MissionControlGUI()

    app.run()


if __name__ == '__main__':

    main()