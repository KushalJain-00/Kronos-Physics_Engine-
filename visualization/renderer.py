import math
import random
from collections import deque

import dearpygui.dearpygui as dpg

from core.constraints import ChainConstraint, HingeConstraint
from core.particles import Particle
from core.rigidbody import RigidBody
from core.vectors import Vector2D
from simulation.timestep import FixedTimestep


class Renderer:
    """Single-window DearPyGui studio: drawlist viewport + docked control panels."""

    def __init__(self, world, scenes=None, scene=None):
        self.world = world
        self.scenes = scenes or {}
        self.current_scene = scene
        self.timestep = FixedTimestep(0.008)
        self.cam_x, self.cam_y = world.width / 2, world.height / 2
        self.zoom = 1.0
        self.view_w, self.view_h = 980, 800
        self.spawn_mode = "select"
        self.mass, self.radius, self.size = 5.0, 10.0, 40.0
        self.time_scale = 1.0
        self.show_vectors = True
        self.show_contacts = True
        self.trails = {}
        self.dragging = None
        self.grab_offset = (0.0, 0.0)
        self._drag_hist = deque(maxlen=4)
        self._pan_last = None
        self.sim_time = 0.0
        self.fps_frames = 0
        self.fps_time = 0.0
        self.frame_count = 0
        self.vel_x_history = []
        self.vel_y_history = []

    def to_screen(self, wx, wy):
        return ((wx - self.cam_x) * self.zoom + self.view_w / 2,
                self.view_h / 2 - (wy - self.cam_y) * self.zoom)

    def from_screen(self, sx, sy):
        return (self.cam_x + (sx - self.view_w / 2) / self.zoom,
                self.cam_y - (sy - self.view_h / 2) / self.zoom)

    def _mouse_world(self):
        mx, my = dpg.get_mouse_pos()
        wx_pos = dpg.get_item_pos("simulation_window")
        return self.from_screen(mx - wx_pos[0], my - wx_pos[1])

    def _over_viewport(self):
        return dpg.is_item_hovered("simulation_window")

    def _line(self, a, b, color, thickness=2):
        dpg.draw_line(a, b, color=color, thickness=thickness)

    def _circle(self, x, y, r, color, fill=None, thickness=1):
        dpg.draw_circle((x, y), r, color=color, fill=fill, thickness=thickness)

    def _point_in_polygon(self, x, y, vertices):
        inside = False
        for i in range(len(vertices)):
            j = (i + 1) % len(vertices)
            xi, yi = vertices[i]
            xj, yj = vertices[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
        return inside

    def build(self):
        dpg.create_context()
        dpg.create_viewport(title="Kronos Studio", width=1280, height=800)
        with dpg.window(label="Controls", tag="controls_window", pos=(0, 0), width=300,
                        no_close=True, no_collapse=True, no_move=True):
            if self.scenes:
                dpg.add_combo(items=list(self.scenes.keys()), label="Scene", default_value=None,
                              width=200, callback=self._load_scene)
            dpg.add_button(label="Pause", tag="pause_btn", callback=self._toggle_pause, width=280)
            dpg.add_button(label="Step", callback=self._step_once, width=280)
            dpg.add_button(label="Reset scene", callback=self._reset_scene, width=280)
            dpg.add_separator()
            dpg.add_text("World Properties")
            dpg.add_slider_float(label="Gravity", default_value=9.8, min_value=0, max_value=50, width=280,
                                 callback=lambda s, a: setattr(self.world.gravity, "y", -a))
            dpg.add_slider_float(label="Restitution", default_value=self.world.restitution,
                                 min_value=0, max_value=1, width=280,
                                 callback=lambda s, a: setattr(self.world, "restitution", a))
            dpg.add_slider_float(label="Friction", default_value=self.world.mu,
                                 min_value=0, max_value=1, width=280,
                                 callback=lambda s, a: setattr(self.world, "mu", a))
            dpg.add_slider_float(label="Drag", default_value=0, min_value=0, max_value=1, width=280,
                                 callback=lambda s, a: setattr(self.world, "drag_coefficient", a))
            dpg.add_slider_float(label="Wind X", default_value=0, min_value=-50, max_value=50, width=280,
                                 callback=lambda s, a: setattr(self.world.wind, "x", a))
            dpg.add_slider_float(label="Wind Y", default_value=0, min_value=-50, max_value=50, width=280,
                                 callback=lambda s, a: setattr(self.world.wind, "y", a))
            dpg.add_slider_float(label="Time Scale", default_value=1.0, min_value=0.25, max_value=4,
                                 width=280, callback=lambda s, a: setattr(self, "time_scale", a))
            dpg.add_slider_int(label="Substeps", default_value=self.world.substeps,
                               min_value=1, max_value=8, width=280,
                               callback=lambda s, a: setattr(self.world, "substeps", int(a)))
            dpg.add_checkbox(label="Show Vectors", default_value=True,
                             callback=lambda s, a: setattr(self, "show_vectors", a))
            dpg.add_checkbox(label="Show Contacts", default_value=True,
                             callback=lambda s, a: setattr(self, "show_contacts", a))
            dpg.add_separator()
            dpg.add_text("Spawn Tools")
            dpg.add_combo(items=["Select", "Particle", "Box", "Circle"], label="Tool", default_value="Select",
                          width=200, callback=lambda s, a: setattr(self, "spawn_mode", a.lower()))
            dpg.add_slider_float(label="Mass", default_value=5.0, min_value=1, max_value=100, width=280,
                                 callback=lambda s, a: setattr(self, "mass", a))
            dpg.add_slider_float(label="Radius", default_value=10.0, min_value=3, max_value=40, width=280,
                                 callback=lambda s, a: setattr(self, "radius", a))
            dpg.add_slider_float(label="Box Size", default_value=40.0, min_value=10, max_value=100, width=280,
                                 callback=lambda s, a: setattr(self, "size", a))
        with dpg.window(label="Simulation", tag="simulation_window", pos=(300, 0), width=980, height=800,
                        no_close=True, no_collapse=True, no_move=True, no_title_bar=True):
            dpg.add_drawlist(tag="viewport", width=980, height=800)
        with dpg.window(label="Inspector", tag="inspector_window", pos=(980, 0), width=300,
                        no_close=True, no_collapse=True, no_move=True):
            dpg.add_text("Selected: None", tag="selected_info")
            dpg.add_text("Velocity: -", tag="selected_velocity")
            dpg.add_text("Angular velocity: -", tag="selected_angular")
            with dpg.plot(label="Velocity over time", height=200, width=280):
                dpg.add_plot_axis(dpg.mvXAxis, label="Frame", tag="plot_x")
                with dpg.plot_axis(dpg.mvYAxis, label="Velocity", tag="plot_y"):
                    dpg.add_line_series([0], [0], label="Vel_x", tag="vel_x_series")
                    dpg.add_line_series([0], [0], label="Vel_y", tag="vel_y_series")
            dpg.add_separator()
            dpg.add_text("FPS: 0", tag="fps_display")
            dpg.add_text("Sim time: 0.00s", tag="sim_time_display")
            dpg.add_text("Particles: 0", tag="particle_count")
            dpg.add_text("Rigid Bodies: 0", tag="body_count")
            dpg.add_text("Constraints: 0", tag="constraint_count")
            dpg.add_text("Links: 0", tag="link_count")
            dpg.add_text("Kinetic Energy: 0", tag="ke_display")
            dpg.add_text("Potential Energy: 0", tag="pe_display")
        with dpg.handler_registry():
            dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left, callback=self._on_click)
            dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Right, callback=self._on_spawn)
            dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Left, callback=self._on_drag)
            dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Middle, callback=self._on_pan)
            dpg.add_mouse_release_handler(button=dpg.mvMouseButton_Left, callback=self._on_release)
            dpg.add_mouse_release_handler(button=dpg.mvMouseButton_Middle, callback=self._clear_pan)
            dpg.add_mouse_wheel_handler(callback=self._on_wheel)

    def _toggle_pause(self):
        self.world.paused = not self.world.paused
        dpg.set_item_label("pause_btn", "Resume" if self.world.paused else "Pause")

    def _step_once(self):
        was_paused = self.world.paused
        self.world.paused = False
        self.world.step(self.timestep.physics_dt * self.time_scale)
        self.world.paused = was_paused

    def _load_scene(self, sender, app_data):
        if not app_data:
            return
        self.world.clear()
        self.scenes[app_data](self.world)
        self.cam_x, self.cam_y = self.world.width / 2, self.world.height / 2
        self.zoom = 1.0
        self.trails.clear()
        self.dragging = None
        self.sim_time = 0.0
        self.current_scene = app_data

    def _reset_scene(self):
        if self.current_scene:
            self._load_scene(None, self.current_scene)

    def _on_click(self, sender, app_data):
        if not self._over_viewport():
            return
        wx, wy = self._mouse_world()
        for p in self.world.particles:
            if (p.position.x - wx) ** 2 + (p.position.y - wy) ** 2 <= p.radius ** 2:
                self.world.selected = p
                return
        for body in self.world.rigid_bodies:
            vertices = body.get_world_vertices()
            if vertices and self._point_in_polygon(wx, wy, vertices):
                self.world.selected = body
                return
        self.world.selected = None

    def _on_drag(self, sender, app_data):
        if not self._over_viewport():
            return
        if self.dragging is None and self.world.selected is not None:
            wx, wy = self._mouse_world()
            obj = self.world.selected
            self.dragging = obj
            self.grab_offset = (wx - obj.position.x, wy - obj.position.y)
            self._drag_hist.clear()
            self._drag_hist.append((obj.position.x, obj.position.y))

    def _on_release(self, sender, app_data):
        if self.dragging is not None:
            obj = self.dragging
            if len(self._drag_hist) > 1:
                dx = self._drag_hist[-1][0] - self._drag_hist[0][0]
                dy = self._drag_hist[-1][1] - self._drag_hist[0][1]
                dt = (len(self._drag_hist) - 1) / 60.0
                obj.velocity = Vector2D(dx / dt, dy / dt)
            self.dragging = None

    def _on_pan(self, sender, app_data):
        if not self._over_viewport():
            return
        if self._pan_last is None:
            self._pan_last = app_data
            return
        dx = app_data[0] - self._pan_last[0]
        dy = app_data[1] - self._pan_last[1]
        self.cam_x -= dx / self.zoom
        self.cam_y += dy / self.zoom
        self._pan_last = app_data

    def _clear_pan(self, sender, app_data):
        self._pan_last = None

    def _on_wheel(self, sender, app_data):
        if not self._over_viewport():
            return
        wx, wy = self._mouse_world()
        self.zoom = max(0.2, min(8.0, self.zoom * 1.1 ** app_data))
        nx, ny = self._mouse_world()
        self.cam_x += wx - nx
        self.cam_y += wy - ny

    def _on_spawn(self, sender, app_data):
        if not self._over_viewport():
            return
        if self.spawn_mode == "select":
            return
        wx, wy = self._mouse_world()
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        if self.spawn_mode == "particle":
            p = Particle(wx, wy, self.mass, color=color)
            p.radius = self.radius
            self.world.add_particle(p)
            return
        body = RigidBody(wx, wy, self.mass, 0, color)
        if self.spawn_mode == "box":
            s = self.size / 2
            body.set_shape([(-s, -s), (s, -s), (s, s), (-s, s)])
        else:
            s = self.size / 2
            body.set_shape([(s * math.cos(2 * math.pi * i / 16), s * math.sin(2 * math.pi * i / 16))
                            for i in range(16)])  # ponytail: polygon circle, no dedicated circle shape
        self.world.add_rigid_bodies(body)

    def update_draw(self):
        cw = dpg.get_viewport_client_width() or 1280
        ch = dpg.get_viewport_client_height() or 800
        self.view_w, self.view_h = max(200, cw - 600), ch
        dpg.configure_item("controls_window", width=300, height=ch)
        dpg.configure_item("simulation_window", pos=(300, 0), width=self.view_w, height=ch)
        dpg.configure_item("inspector_window", pos=(cw - 300, 0), width=300, height=ch)
        dpg.configure_item("viewport", width=self.view_w, height=self.view_h)

        if self.dragging is not None:
            wx, wy = self._mouse_world()
            obj = self.dragging
            obj.position.x = wx - self.grab_offset[0]
            obj.position.y = wy - self.grab_offset[1]
            obj.velocity = Vector2D(0, 0)
            if hasattr(obj, "angular_velocity"):
                obj.angular_velocity = 0
            self._drag_hist.append((obj.position.x, obj.position.y))

        dpg.delete_item("viewport", children_only=True)
        dpg.push_container_stack("viewport")
        dpg.draw_rectangle((0, 0), (self.view_w, self.view_h), color=(18, 18, 24), fill=(18, 18, 24))
        self._draw_grid()
        for spring in self.world.springs:
            a = self.to_screen(spring.p1.position.x, spring.p1.position.y)
            b = self.to_screen(spring.p2.position.x, spring.p2.position.y)
            self._line(a, b, (100, 150, 255), thickness=2)
        self._draw_constraints()
        self._draw_particles()
        self._draw_bodies()
        if self.show_vectors:
            for p in self.world.particles:
                self._draw_vector(p)
            for body in self.world.rigid_bodies:
                self._draw_vector(body)
        if self.show_contacts:
            for contact in getattr(self.world, "debug_contacts", []) or []:
                normal = contact["normal"]
                nx, ny = getattr(normal, "x", normal[0]), getattr(normal, "y", normal[1])
                px, py = contact["point"]
                sx, sy = self.to_screen(px, py)
                self._circle(sx, sy, 3, (255, 255, 0))
                ex, ey = self.to_screen(px + nx * 15, py + ny * 15)
                self._line((sx, sy), (ex, ey), (255, 140, 0), thickness=2)
        self._draw_selection()
        dpg.pop_container_stack()

    def _draw_grid(self):
        wl = self.cam_x - self.view_w / 2 / self.zoom
        wr = self.cam_x + self.view_w / 2 / self.zoom
        wb = self.cam_y - self.view_h / 2 / self.zoom
        wt = self.cam_y + self.view_h / 2 / self.zoom
        for gx in range(int(wl // 50) * 50, int(wr) + 50, 50):
            self._line(self.to_screen(gx, wb), self.to_screen(gx, wt), (45, 45, 55), thickness=1)
        for gy in range(int(wb // 50) * 50, int(wt) + 50, 50):
            self._line(self.to_screen(wl, gy), self.to_screen(wr, gy), (45, 45, 55), thickness=1)

    def _draw_constraints(self):
        for constraint in self.world.constraints:
            if isinstance(constraint, ChainConstraint):
                for segment in constraint.segments:
                    a = segment._get_world_anchor(segment.body_a, segment.anchor_a)
                    b = segment._get_world_anchor(segment.body_b, segment.anchor_b)
                    self._line(self.to_screen(*a), self.to_screen(*b), (255, 100, 100), thickness=2)
                if constraint.visible_links:
                    for link in constraint.links:
                        self._circle(*self.to_screen(link.position.x, link.position.y), 2, (0, 255, 0))
            elif hasattr(constraint, "_get_world_anchor") and hasattr(constraint, "anchor_a"):
                a = constraint._get_world_anchor(constraint.body_a, constraint.anchor_a)
                b = constraint._get_world_anchor(constraint.body_b, constraint.anchor_b)
                self._line(self.to_screen(*a), self.to_screen(*b), (255, 100, 100), thickness=2)
                if isinstance(constraint, HingeConstraint):
                    self._circle(*self.to_screen(*a), 5, (255, 100, 100))

    def _draw_particles(self):
        for p in self.world.particles:
            trail = self.trails.setdefault(id(p), deque(maxlen=25))
            trail.append((p.position.x, p.position.y))
        if sum(len(t) for t in self.trails.values()) < 2000:  # ponytail: cap trail cost
            for p in self.world.particles:
                trail = self.trails[id(p)]
                n = len(trail)
                for i, (tx, ty) in enumerate(trail):
                    frac = (i + 1) / n
                    c = (p.color[0], p.color[1], p.color[2], int(120 * frac))
                    self._circle(*self.to_screen(tx, ty), p.radius * 0.6 * frac, c, fill=c)
        for p in self.world.particles:
            sx, sy = self.to_screen(p.position.x, p.position.y)
            darker = tuple(int(c * 0.6) for c in p.color)
            self._circle(sx, sy, p.radius, darker, fill=p.color, thickness=2)
            if getattr(p, "pinned", False):
                dpg.draw_rectangle((sx - 3, sy - 3), (sx + 3, sy + 3), color=(255, 255, 255), fill=(255, 255, 255))

    def _draw_bodies(self):
        for body in self.world.rigid_bodies:
            points = [self.to_screen(x, y) for x, y in body.get_world_vertices()]
            if len(points) >= 3:
                lighter = tuple(min(255, int(c * 1.4)) for c in body.color)
                dpg.draw_polygon(points, color=lighter, fill=body.color, thickness=2)

    def _draw_vector(self, obj):
        sx, sy = self.to_screen(obj.position.x, obj.position.y)
        ex, ey = self.to_screen(obj.position.x + obj.velocity.x * 0.3,
                                obj.position.y + obj.velocity.y * 0.3)
        self._line((sx, sy), (ex, ey), (0, 255, 100), thickness=2)

    def _draw_selection(self):
        sel = self.world.selected
        if sel is None:
            return
        sx, sy = self.to_screen(sel.position.x, sel.position.y)
        if isinstance(sel, Particle):
            self._circle(sx, sy, sel.radius + 3, (255, 255, 255), thickness=2)
        elif isinstance(sel, RigidBody):
            points = [self.to_screen(x, y) for x, y in sel.get_world_vertices()]
            if len(points) >= 3:
                dpg.draw_polygon(points, color=(255, 255, 255), thickness=3)

    def update_stats(self):
        now = dpg.get_total_time()
        self.fps_frames += 1
        if now - self.fps_time >= 1.0:
            dpg.set_value("fps_display", f"FPS: {self.fps_frames / (now - self.fps_time):.0f}")
            self.fps_frames = 0
            self.fps_time = now
        dpg.set_value("sim_time_display", f"Sim time: {self.sim_time:.2f}s")
        dpg.set_value("particle_count", f"Particles: {len(self.world.particles)}")
        dpg.set_value("body_count", f"Rigid Bodies: {len(self.world.rigid_bodies)}")
        dpg.set_value("constraint_count", f"Constraints: {len(self.world.constraints)}")
        dpg.set_value("link_count", f"Links: {len(self.world.links)}")
        g = abs(self.world.gravity.y)
        ke = sum(0.5 * o.mass * (o.velocity.x ** 2 + o.velocity.y ** 2) for o in self.world.particles)
        ke += sum(0.5 * o.mass * (o.velocity.x ** 2 + o.velocity.y ** 2) for o in self.world.links)
        pe = sum(o.mass * g * o.position.y for o in self.world.particles)
        dpg.set_value("ke_display", f"Kinetic Energy: {ke:.1f}")
        dpg.set_value("pe_display", f"Potential Energy: {pe:.1f}")
        self._update_selected()

    def _update_selected(self):
        sel = self.world.selected
        if sel is None:
            dpg.set_value("selected_info", "Selected: None")
            dpg.set_value("selected_velocity", "Velocity: -")
            dpg.set_value("selected_angular", "Angular velocity: -")
            if self.vel_x_history:
                self.vel_x_history.clear()
                self.vel_y_history.clear()
                self.frame_count = 0
                dpg.set_value("vel_x_series", [[], []])
                dpg.set_value("vel_y_series", [[], []])
            return
        vx, vy = sel.velocity.x, sel.velocity.y
        ang = getattr(sel, "angular_velocity", None)
        dpg.set_value("selected_info",
                      f"Selected {type(sel).__name__}: Mass={sel.mass:.1f}, Pos=({sel.position.x:.1f}, {sel.position.y:.1f})")
        dpg.set_value("selected_velocity", f"Velocity: ({vx:.2f}, {vy:.2f})")
        dpg.set_value("selected_angular", f"Angular velocity: {ang:.2f}" if ang is not None else "Angular velocity: -")
        self.frame_count += 1
        self.vel_x_history.append(vx)
        self.vel_y_history.append(vy)
        if len(self.vel_x_history) > 200:
            self.vel_x_history.pop(0)
            self.vel_y_history.pop(0)
        x_axis = list(range(len(self.vel_x_history)))
        dpg.set_value("vel_x_series", [x_axis, self.vel_x_history])
        dpg.set_value("vel_y_series", [x_axis, self.vel_y_history])
        dpg.fit_axis_data("plot_y")

    def run(self):
        self.build()
        dpg.setup_dearpygui()
        dpg.show_viewport()
        last = dpg.get_total_time()
        self.fps_time = last
        while dpg.is_dearpygui_running():
            now = dpg.get_total_time()
            steps = self.timestep.advance(now - last)
            last = now
            for _ in range(steps):
                if not self.world.paused:
                    self.world.step(self.timestep.physics_dt * self.time_scale)
                    self.sim_time += self.timestep.physics_dt * self.time_scale
            self.update_draw()
            self.update_stats()
            dpg.render_dearpygui_frame()
        dpg.destroy_context()
