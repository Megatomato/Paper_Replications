from ghost import ReferenceGhost
from controller import KanayamaControl
from common.models import UnicycleModel
from common.environment import MapEnv
import numpy as np

def main():
    ghost = ReferenceGhost(startx=0, starty=0, starttheta=0, scale=1, T=10)
    controller = KanayamaControl(Kx=10, Ky=64, Ktheta=16)
    robot = UnicycleModel(x=-2, y=1, theta=30)
    env = MapEnv()

    dt = 0.1
    sim_time = 20
    time_steps = int(sim_time / dt)

    for step in range(time_steps):
        t = step * dt
        ref_state = ghost.refStep(t)
        control_inputs = controller.compute_control(robot.state, ref_state)

        robot.step(control_inputs, dt)

    


if __name__ == "__main__":
    main()
    