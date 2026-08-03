class SpatialHash:
    def __init__(self, cell_size=100.0):
        self.cell_size = cell_size
        self.cells = {}

    def clear(self):
        self.cells = {}

    def insert(self, obj, x, y, extent=0.0):
        min_cx = int((x - extent) // self.cell_size)
        max_cx = int((x + extent) // self.cell_size)
        min_cy = int((y - extent) // self.cell_size)
        max_cy = int((y + extent) // self.cell_size)
        for cx in range(min_cx, max_cx + 1):
            for cy in range(min_cy, max_cy + 1):
                self.cells.setdefault((cx, cy), []).append(obj)

    def pairs(self):
        seen = set()
        result = []
        for (cx, cy), objs in self.cells.items():
            buckets = [self.cells.get((cx + dx, cy + dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
            for a in objs:
                for bucket in buckets:
                    if bucket is None:
                        continue
                    for b in bucket:
                        if a is b:
                            continue
                        key = (id(a), id(b)) if id(a) < id(b) else (id(b), id(a))
                        if key not in seen:
                            seen.add(key)
                            result.append((a, b))
        return result
