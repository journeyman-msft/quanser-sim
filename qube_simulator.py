from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import time
import math
import numpy as np
import random
from scipy.integrate import odeint

from qube_render import QubeRendererVypthon

class QubeSimulator(object):
    """Simulator that has the same interface as the hardware wrapper."""

    def __init__(self, frequency=250):
        self.frequency = frequency
        self._dt = 1.0 / self.frequency
        self._integration_steps = int(np.ceil(1000 / self.frequency))
        self._max_voltage = 18.0

        # Motor
        self.Rm = 8.4  # Resistance
        self.kt = 0.042  # Current-torque (N-m/A)
        self.km = 0.042  # Back-emf constant (V-s/rad)

        # Rotary Arm
        self.mr = 0.095  # Mass (kg)
        self.Lr = 0.085  # Total length (m)
        self.Jr = self.mr * self.Lr ** 2 / 12  # Moment of inertia about pivot (kg-m^2)
        self.Dr = 0.00027  # Equivalent viscous damping coefficient (N-m-s/rad)

        # Pendulum Link
        self.mp = 0.024  # Mass (kg)
        self.Lp = 0.129  # Total length (m)
        self.Jp = self.mp * self.Lp ** 2 / 12  # Moment of inertia about pivot (kg-m^2)
        self.Dp = 0.00005  # Equivalent viscous damping coefficient (N-m-s/rad)

        self.g = 9.81  # Gravity constant

        self.state = (
            np.array([0, 0, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )

    def step(self, action, led=None):
        action = np.clip(action, -self._max_voltage, self._max_voltage)
        self.state = self._forward_model_ode(
            *self.state, action, self._dt, self._integration_steps
        )
        return self.state

    def reset_up(self, config=None):
        self.state = (
            np.array([0, 0, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )
        if config:
            self.Lp = config["Lp"]
            self.mp = config["mp"]
            self.state = np.array([config["theta"], config["alpha"],
                                   config["theta_dot"], config["alpha_dot"]])

        return self.state

    def reset_down(self, config=None):
        self.state = (
            np.array([0, np.pi, 0, 0], dtype=np.float64) + np.random.randn(4) * 0.01
        )
        if config:
            self.Lp = config["Lp"]
            self.mp = config["mp"]
            self.state = np.array([config["theta"], config["alpha"],
                                   config["theta_dot"], config["alpha_dot"]])
            

        return self.state
    
    def _diff_forward_model_ode(self, state, t, action, dt):
        theta, alpha, theta_dot, alpha_dot = state
        Vm = action
        tau = -(self.km * (Vm - self.km * theta_dot)) / self.Rm  # torque

        # fmt: off
        # From Rotary Pendulum Workbook
        theta_dot_dot = (-self.Lp*self.Lr*self.mp*(-8.0*self.Dp*alpha_dot + self.Lp**2*self.mp*theta_dot**2*np.sin(2.0*alpha) + 4.0*self.Lp*self.g*self.mp*np.sin(alpha))*np.cos(alpha) + (4.0*self.Jp + self.Lp**2*self.mp)*(4.0*self.Dr*theta_dot + self.Lp**2*alpha_dot*self.mp*theta_dot*np.sin(2.0*alpha) + 2.0*self.Lp*self.Lr*alpha_dot**2*self.mp*np.sin(alpha) - 4.0*tau))/(4.0*self.Lp**2*self.Lr**2*self.mp**2*np.cos(alpha)**2 - (4.0*self.Jp + self.Lp**2*self.mp)*(4.0*self.Jr + self.Lp**2*self.mp*np.sin(alpha)**2 + 4.0*self.Lr**2*self.mp))
        alpha_dot_dot = (2.0*self.Lp*self.Lr*self.mp*(4.0*self.Dr*theta_dot + self.Lp**2*alpha_dot*self.mp*theta_dot*np.sin(2.0*alpha) + 2.0*self.Lp*self.Lr*alpha_dot**2*self.mp*np.sin(alpha) - 4.0*tau)*np.cos(alpha) - 0.5*(4.0*self.Jr + self.Lp**2*self.mp*np.sin(alpha)**2 + 4.0*self.Lr**2*self.mp)*(-8.0*self.Dp*alpha_dot + self.Lp**2*self.mp*theta_dot**2*np.sin(2.0*alpha) + 4.0*self.Lp*self.g*self.mp*np.sin(alpha)))/(4.0*self.Lp**2*self.Lr**2*self.mp**2*np.cos(alpha)**2 - (4.0*self.Jp + self.Lp**2*self.mp)*(4.0*self.Jr + self.Lp**2*self.mp*np.sin(alpha)**2 + 4.0*self.Lr**2*self.mp))
        # fmt: on

        diff_state = np.array([theta_dot, alpha_dot, theta_dot_dot, alpha_dot_dot]).reshape(
            (4,)
        )
        diff_state = np.array(diff_state, dtype="float64")
        return diff_state


    def _forward_model_ode(self, theta, alpha, theta_dot, alpha_dot, Vm, dt, integration_steps):
        t = np.linspace(0.0, dt, 2)  # TODO: add and check integration steps here

        state = np.array([theta, alpha, theta_dot, alpha_dot])
        next_state = np.array(odeint(self._diff_forward_model_ode, state, t, args=(Vm, dt)))[1, :]
        theta, alpha, theta_dot, alpha_dot = next_state

        theta = ((theta + np.pi) % (2 * np.pi)) - np.pi
        alpha = ((alpha + np.pi) % (2 * np.pi)) - np.pi

        return theta, alpha, theta_dot, alpha_dot

if __name__ == '__main__':
    qube = QubeSimulator(frequency=250)
    
    for episode in range(2):
        # Optional config
        config = {
            # Parameters
            "Lp": 0.129,
            "mp": 0.024,
            # Initial Conditions
            "theta": 0 + np.random.randn() * 0.01,
            "alpha": 0 + np.random.randn() * 0.01, # make sure pi if reset_down
            "theta_dot": 0 + np.random.randn() * 0.01,
            "alpha_dot": 0 + np.random.randn() * 0.01
        }

        print('episode: ', episode)
        state = qube.reset_up(config)
        theta0 = qube.state[0]
        alpha0 = qube.state[1]
        if episode == 0:
            viewer = QubeRendererVypthon(theta0, alpha0, qube.frequency)
        
        for i in range(2048):
            action = random.uniform(-3, 3)
            state = qube.step(action)
            viewer.render(state[0], state[1])
        viewer.close()