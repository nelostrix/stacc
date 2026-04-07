# stacc/control/limiter.py


class RateLimiter:
    def __init__(self, max_rate: float):
        self.max_rate = max_rate
        self.prev = 0.0

    def compute(self, value: float, dt: float):
        delta = value - self.prev
        max_delta = self.max_rate * dt

        delta = max(-max_delta, min(max_delta, delta))
        self.prev += delta
        return self.prev
