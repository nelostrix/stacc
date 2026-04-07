# stacc/control/wrappers.py


class PIDWithFeedforward:
    def __init__(self, pid, feedforward):
        self.pid = pid
        self.feedforward = feedforward

    def compute(self, measurement, dt, velocity=0.0, acceleration=0.0):
        u_pid = self.pid.compute(measurement, dt, is_error=False)
        u_ff = self.feedforward.compute(velocity, acceleration)
        return u_pid + u_ff

    def reset(self):
        self.pid.reset()


class FilteredController:
    def __init__(self, controller, filter_):
        self.controller = controller
        self.filter = filter_

    def compute(self, *args, **kwargs):
        u = self.controller.compute(*args, **kwargs)
        return self.filter.compute(u)

    def reset(self):
        self.controller.reset()


class LimitedController:
    def __init__(self, controller, limiter):
        self.controller = controller
        self.limiter = limiter

    def compute(self, *args, dt, **kwargs):
        u = self.controller.compute(*args, dt=dt, **kwargs)
        return self.limiter.compute(u, dt)

    def reset(self):
        self.controller.reset()
