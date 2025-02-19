# Quanser Qube - Servo2 sim
For more information about the Qube click [here](https://www.quanser.com/products/qube-servo-2/)

![](img/QUBE-Servo_2_angled_pendulum.jpg)

## Simulator structure
```
├── README.md
├── render_qube.py
├── qube_simulator.py
└── requirements.txt
```

### reset_down()
Info:

- Reset starts the pendulum from the bottom (at rest).
- The task is to flip up the pendulum and hold it upright.
- Episode ends once the theta angle is greater than 90 degrees.
- Reward should be a function of the angles theta (arm angle) and alpha (pendulum), and the alpha angular velocity.
    - Encourages the the arm to stay centered, the pendulum to stay upright, and to stay stationary.


### reset_up()
Info:

- Reset starts the pendulum from the top (flipped up/inverted).
- The task is to hold the pendulum upright.
- Episode ends once the alpha angle is greater the 20 degrees or theta angle is greater than 90 degrees.
- Reward should be a function of the angles theta (arm angle) and alpha (pendulum), and the alpha angular velocity.
    - Encourages the the arm to stay centered, the pendulum to stay upright, and to stay stationary.

## Action
`action = Vm`

## States
`state = [theta, alpha, theta_dot, alpha_dot]`

## Usage

```
    qube = QubeSimulator(frequency=250)

    ## LQR benchmark controller gains
    #K = np.array([-2.0, 35.0, -1.5, 3.0])
    
    for episode in range(2):
        # Optional config
        config = {
            # Parameters
            "Lp": 0.129,
            "mp": 0.024,
            # Initial Conditions
            "theta": random.randint(0, 360) * 2 * np.pi / 360,
            "alpha": 0 + np.random.randn() * 0.05, # make sure pi if reset_down
            "theta_dot": 0 + np.random.randn() * 0.05,
            "alpha_dot": 0 + np.random.randn() * 0.05
        }

        print('episode: ', episode)
        state = qube.reset_up(config)
        
        for i in range(2048):
            action = random.uniform(-3, 3)
            #action = K.dot(state)
            state = qube.step(action)
            qube.view()
```
