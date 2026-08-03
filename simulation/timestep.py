class FixedTimestep:
    def __init__(self, physics_dt=0.008, max_frame_dt=0.05):
        self.physics_dt = physics_dt
        self.max_frame_dt = max_frame_dt
        self.accumulator = 0.0

    def advance(self, frame_dt) -> int:
        self.accumulator += min(frame_dt, self.max_frame_dt)
        steps = int(self.accumulator // self.physics_dt)
        self.accumulator -= steps * self.physics_dt
        return steps
