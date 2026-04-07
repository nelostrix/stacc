# stacc/control/filters.py


class LowPassFilter:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.prev = None

    def compute(self, value: float):
        if self.prev is None:
            self.prev = value
        self.prev = self.alpha * value + (1 - self.alpha) * self.prev
        return self.prev
