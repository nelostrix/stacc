# stacc/control/feedforward.py


class Feedforward:
    def __init__(self, kv=0.0, ka=0.0, ks=0.0):
        self.kv = kv
        self.ka = ka
        self.ks = ks

    def compute(self, velocity=0.0, acceleration=0.0):
        return self.kv * velocity + self.ka * acceleration + self.ks
