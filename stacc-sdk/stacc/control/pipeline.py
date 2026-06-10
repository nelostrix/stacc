# stacc/control/pipeline.py


class ControllerPipeline:
    def __init__(self, *controllers):
        self.controllers = controllers

    def compute(self, *args, **kwargs):
        value = None

        for ctrl in self.controllers:
            if value is None:
                value = ctrl.compute(*args, **kwargs)
            else:
                value = ctrl.compute(value=value, dt=kwargs.get("dt"))

        return value

    def reset(self):
        for ctrl in self.controllers:
            if hasattr(ctrl, "reset"):
                ctrl.reset()
