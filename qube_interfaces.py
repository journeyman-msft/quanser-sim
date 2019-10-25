from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import time
import math
import numpy as np
import random

from qube_simulator import forward_model_euler, forward_model_ode
from qube_render import QubeRendererVypthon

class QubeSimulator(object):
    """Simulator that has the same interface as the hardware wrapper."""

    def __init__(
        self, forward_model="ode", frequency=250, integration_steps=1, max_voltage=18.0
    ):
        if isinstance(forward_model, str):
            if forward_model == "ode":
                self._forward_model = forward_model_ode
            elif forward_model == "euler":
                self._forward_model = forward_model_euler
            else:
                raise ValueError(
                    "'forward_model' must be one of ['ode', 'euler'] or a callable."
                )
        elif callable(forward_model):
            self._forward_model = forward_model
        else:
            raise ValueError(
                "'forward_model' must be one of ['ode', 'euler'] or a callable."
            )

        self._dt = 1.0 / frequency
        self._integration_steps = integration_steps
        self._max_voltage = max_voltage
        self.state = (
            np.array([0, 0, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def step(self, action, led=None):
        action = np.clip(action, -self._max_voltage, self._max_voltage)
        self.state = self._forward_model(
            *self.state, action, self._dt, self._integration_steps
        )
        return self.state

    def reset_up(self):
        self.state = (
            np.array([0, 0, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )
        return self.state

    def reset_down(self):
        self.state = (
            np.array([0, np.pi, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )
        return self.state

    def reset_encoders(self):
        pass

    def close(self, type=None, value=None, traceback=None):
        pass

if __name__ == '__main__':
    MAX_MOTOR_VOLTAGE = 3
    frequency = 250
    integration_steps = int(np.ceil(1000 / frequency))
    qube = QubeSimulator(forward_model="ode", frequency=frequency,
                integration_steps=integration_steps, 
                max_voltage=MAX_MOTOR_VOLTAGE)
    
    for episode in range(2):
        print('episode: ', episode)
        state = qube.reset_up()
        theta0 = qube.state[0]
        alpha0 = qube.state[1]
        if episode == 0:
            viewer = QubeRendererVypthon(theta0, alpha0, frequency)
        
        for i in range(2048):
            action = random.uniform(-3, 3)
            state = qube.step(action)
            viewer.render(state[0], state[1])
        viewer.close()