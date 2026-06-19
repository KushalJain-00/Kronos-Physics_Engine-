import dearpygui.dearpygui as dpg
import threading
from core.particles import Particle
from core.rigidbody import RigidBody

class ControlPanel:
    def __init__(self , world):
        self.world = world
        self.running = False
        self.thread = None
    
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
        
        dpg.setup_dearpygui()
        dpg.show_viewport()
    
    def _toggle_pause(self):
        with self.world.lock:
            self.world.paused = not getattr(self.world, 'paused', False)
    
    def _clear_world(self):
        with self.world.lock:
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
            particles = list(self.world.particles)
            bodies = list(self.world.rigid_bodies)
            constraints = len(self.world.constraints)
        dpg.set_value("particle_count", f"Particles: {len(particles)}")
        dpg.set_value("body_count", f"Rigid Bodies: {len(bodies)}")
        dpg.set_value("constraint_count", f"Constraints: {constraints}")
        
        # energy
        ke = sum(0.5 * p.mass * (p.velocity.x**2 + p.velocity.y**2) for p in particles)
        pe = sum(p.mass * 9.8 * (p.position.y - p.radius) for p in self.world.particles)
        dpg.set_value("ke_display", f"Kinetic Energy: {ke:.1f}")
        dpg.set_value("pe_display", f"Potential Energy: {pe:.1f}")

        self._show_selected_info()

    def _show_selected_info(self):
        with self.world.lock:
            selected = self.world.selected
        if selected is not None:
            dpg.set_value("selected_velocity", f"Velocity: ({selected.velocity.x:.2f}, {selected.velocity.y:.2f})")
            dpg.set_value("selected_acceleration", f"Acceleration: ({selected.acceleration.x:.2f}, {selected.acceleration.y:.2f})")
        if selected is None:
            dpg.set_value("selected_info", "Selected: None")
        elif isinstance(selected, Particle):
            dpg.set_value("selected_info", f"Selected Particle: Mass={selected.mass}, Position=({selected.position.x:.1f}, {selected.position.y:.1f})")
        elif isinstance(selected, RigidBody):
            dpg.set_value("selected_info", f"Selected RigidBody: Mass={selected.mass}, Position=({selected.position.x:.1f}, {selected.position.y:.1f})")

    def run(self):
        self.build()
        while dpg.is_dearpygui_running():
            self._update_stats()
            dpg.render_dearpygui_frame()
        dpg.destroy_context()
        
    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()