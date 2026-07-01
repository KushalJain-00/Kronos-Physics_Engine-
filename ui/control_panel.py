import dearpygui.dearpygui as dpg
import threading
from core.particles import Particle
from core.rigidbody import RigidBody

class ControlPanel:
    def __init__(self , world):
        self.world = world
        self.running = False
        self.thread = None
        self.frame_count = 0
        self.vel_x_history = []
        self.vel_y_history = []
        self.max_history = 200
    
    def build(self):
        dpg.create_context()
        dpg.create_viewport(title="Kronos Control Panel", width=380, height=600, x_pos=0, y_pos=0)
        with dpg.window(label="World Controls", width=380, height=500):
            dpg.add_text("Physcis Engine")
            dpg.add_separator()

            dpg.add_button(label="Pause / Resume", callback=self._toggle_pause)
            dpg.add_button(label="Clear World", callback=self._clear_world)
            dpg.add_separator()
            
            # world properties
            dpg.add_text("World Properties")
            dpg.add_slider_float(label="Gravity", default_value=9.8, min_value=0, max_value=50, callback=self._set_gravity)
            dpg.add_slider_float(label="Restitution", default_value=0.9, min_value=0, max_value=1, callback=self._set_restitution)
            dpg.add_slider_float(label="Drag", default_value=0, min_value=0, max_value=1, callback=self._set_drag)

            dpg.add_separator()
            dpg.add_text("Simulation Stats")
            dpg.add_text("FPS: 0", tag="fps_display")
            dpg.add_text("Particles: 0", tag="particle_count")
            dpg.add_text("Rigid Bodies: 0", tag="body_count")
            dpg.add_text("Constraints: 0", tag="constraint_count")
            dpg.add_text("Kinetic Energy: 0", tag="ke_display")
            dpg.add_text("Potential Energy: 0", tag="pe_display")

            dpg.add_separator()
            dpg.add_text("Selected Object")
            dpg.add_text("Selected: None", tag="selected_info")
            dpg.add_text("Velocity: -", tag="selected_velocity")
            dpg.add_text("Acceleration: -", tag="selected_acceleration")

            dpg.add_separator()
            dpg.add_text("Live Graph - Selected Object")
            with dpg.plot(label = "Velocity over time" , height = 200 , width = 350):
                # DearPyGui requires series to be children of a plot axis.
                # Create explicit X and Y axes and attach the series to the Y axis.
                with dpg.plot_axis(dpg.mvXAxis, label="Frame"):
                    pass
                with dpg.plot_axis(dpg.mvYAxis, label="Velocity"):
                    # initialize series with a single zero point to avoid exceptions
                    dpg.add_line_series([0], [0], label = "Vel_x" , tag = "vel_x_series")
                    dpg.add_line_series([0], [0], label = "Vel_y" , tag = "vel_y_series")

        dpg.setup_dearpygui()
        dpg.show_viewport()
    
    def _toggle_pause(self, sender, app_data):
        with self.world.lock:
            self.world.paused = not getattr(self.world, 'paused', False)
            
    def _clear_world(self, sender, app_data):
        self.world.clear()
    
    def _set_gravity(self, sender, value):
        with self.world.lock:
            self.world.gravity.y = -value

    def _set_restitution(self, sender, value):
        with self.world.lock:
            self.world.restitution = value

    def _set_drag(self, sender, value):
        with self.world.lock:
            self.world.drag_coefficient = value

    def _update_stats(self):
        with self.world.lock:
            particle_snapshots = [
                (p.mass, p.velocity.x, p.velocity.y, p.position.y, p.radius)
                for p in self.world.particles
            ]
            bodies = list(self.world.rigid_bodies)
            constraints = len(self.world.constraints)
            gravity_strength = -self.world.gravity.y if self.world.gravity.y < 0 else self.world.gravity.y
        dpg.set_value("particle_count", f"Particles: {len(particle_snapshots)}")
        dpg.set_value("body_count", f"Rigid Bodies: {len(bodies)}")
        dpg.set_value("constraint_count", f"Constraints: {constraints}")
        
        # energy
        ke = sum(0.5 * mass * (vx*vx + vy*vy) for mass, vx, vy, _, _ in particle_snapshots)
        pe = sum(mass * gravity_strength * (y - radius) for mass, _, _, y, radius in particle_snapshots)
        dpg.set_value("ke_display", f"Kinetic Energy: {ke:.1f}")
        dpg.set_value("pe_display", f"Potential Energy: {pe:.1f}")

        self._show_selected_info()

    def _show_selected_info(self):
        selected_snapshot = None
        with self.world.lock:
            selected = self.world.selected
            if selected is not None:
                selected_snapshot = {
                    "type": "particle" if isinstance(selected, Particle) else "rigidbody" if isinstance(selected, RigidBody) else "unknown",
                    "velocity": (selected.velocity.x, selected.velocity.y),
                    "acceleration": (selected.acceleration.x, selected.acceleration.y),
                    "position": (selected.position.x, selected.position.y),
                    "mass": selected.mass,
                }
        if selected_snapshot is None:
            dpg.set_value("selected_info", "Selected: None")
            dpg.set_value("selected_velocity", "Velocity: -")
            dpg.set_value("selected_acceleration", "Acceleration: -")
            self.frame_count = 0
            self.vel_x_history = []
            self.vel_y_history = []
            dpg.set_value("vel_x_series", [[], []])
            dpg.set_value("vel_y_series", [[], []])
            return

        vx, vy = selected_snapshot["velocity"]
        ax, ay = selected_snapshot["acceleration"]
        px, py = selected_snapshot["position"]
        mass = selected_snapshot["mass"]

        dpg.set_value("selected_velocity", f"Velocity: ({vx:.2f}, {vy:.2f})")
        dpg.set_value("selected_acceleration", f"Acceleration: ({ax:.2f}, {ay:.2f})")

        if selected_snapshot["type"] == "particle":
            dpg.set_value("selected_info", f"Selected Particle: Mass={mass}, Position=({px:.1f}, {py:.1f})")
        elif selected_snapshot["type"] == "rigidbody":
            dpg.set_value("selected_info", f"Selected RigidBody: Mass={mass}, Position=({px:.1f}, {py:.1f})")
        else:
            dpg.set_value("selected_info", "Selected: Unknown")

        self.frame_count += 1
        self.vel_x_history.append(vx)
        self.vel_y_history.append(vy)

        if len(self.vel_x_history) > self.max_history:
            self.vel_x_history.pop(0)
            self.vel_y_history.pop(0)

        x_axis = list(range(len(self.vel_x_history)))
        dpg.set_value("vel_x_series", [x_axis, self.vel_x_history])
        dpg.set_value("vel_y_series", [x_axis, self.vel_y_history])

    def run(self):
        self.build()
        while dpg.is_dearpygui_running():
            self._update_stats()
            dpg.render_dearpygui_frame()
        dpg.destroy_context()
        
    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()