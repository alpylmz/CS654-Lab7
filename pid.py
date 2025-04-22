from dataclasses import dataclass
import pybullet as p
import time
import numpy as np
import utils
import argparse


@dataclass
class Point:
    x: float
    y: float

@dataclass
class World:
    """A class to hold the world objects"""
    plate: int
    sphere: int

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--setpoint", type=float, nargs=2, default=(0.0,0.0))
    parser.add_argument("--kp", type=float, default=0.65) # set these values to tune the PD controller
    parser.add_argument("--kd", type=float, default=0.2) # set these values to tune the PD controller
    parser.add_argument("--noise", action="store_true", help="Add noise to the measurements")
    parser.add_argument("--filtered", action="store_true", help="filter the measurements")
    cmd_args = parser.parse_args()
    print(cmd_args)
    cmd_args.setpoint = Point(*cmd_args.setpoint)
    return cmd_args


def run_controller(kp, kd, setpoint, noise, filtered, world: World):

    def set_plate_angles(theta_x, theta_y):
        p.setJointMotorControl2(world.plate, 1, p.POSITION_CONTROL, targetPosition=np.clip(theta_x, -0.1, 0.1), force=5, maxVelocity=2)
        p.setJointMotorControl2(world.plate, 0, p.POSITION_CONTROL, targetPosition=np.clip(-theta_y, -0.1, 0.1), force=5, maxVelocity=2)

    # you can set the variables that should stay accross control loop here

    prev_error_x = 0.0
    prev_error_y = 0.0
    #pd_controller.prev_error_x = 0
    #pd_controller.prev_error_y = 0

    def pd_controller(x, y, kp, kd, setpoint):
        """Implement a PD controller, you can access the setpoint via setpoint.x and setpoint.y
        the plate is small around 0.1 to 0.2 meters. You will have to calculate the error and change in error and 
        use those to calculate the angle to apply to the plate."""
        error_x = setpoint.x - x
        error_y = setpoint.y - y

        d_error_x = (error_x - pd_controller.prev_error_x) / 0.01
        d_error_y = (error_y - pd_controller.prev_error_y) / 0.01

        angle_x = kp * error_x + kd * d_error_x
        angle_y = kp * error_y + kd * d_error_y

        pd_controller.prev_error_x = error_x
        pd_controller.prev_error_y = error_y

        return angle_x, angle_y
    
    pd_controller.prev_error_x = prev_error_x
    pd_controller.prev_error_y = prev_error_y


    def filter_val(val, axis):
        """
        A simple low-pass filter.
        Uses the formula: filtered = alpha * new_value + (1 - alpha) * previous_filtered_value.
        The parameter `axis` should be either 'x' or 'y'.
        """
        alpha = 0.1
        if axis == 'x':
            if not hasattr(filter_val, "prev_x"):
                filter_val.prev_x = val
            filtered = alpha * val + (1 - alpha) * filter_val.prev_x
            filter_val.prev_x = filtered
        elif axis == 'y':
            if not hasattr(filter_val, "prev_y"):
                filter_val.prev_y = val
            filtered = alpha * val + (1 - alpha) * filter_val.prev_y
            filter_val.prev_y = filtered
        else:
            filtered = val
        return filtered

    def every_10ms(i: int, t: float):
        '''This function is called every ms and performs the following:
        1. Get the measurement of the position of the ball
        2. Calculate the forces to be applied to the plate
        3. Apply the forces to the plate
        '''
        (x,y,z), orientation = p.getBasePositionAndOrientation(world.sphere)
        if noise:
            x += utils.noise(t) # the noise added has a frequency between 30 and 50 Hz
            y += utils.noise(t, seed = 43) # so that the noise on y is different than the one on x
        
        if filtered:
            x = filter_val(x)
            y = filter_val(y)

        (angle_x, angle_y) = pd_controller(x, y, kp, kd, setpoint)
        set_plate_angles(angle_x, angle_y)

        if i%10 == 0: # print every 100 ms
            print(f"t: {t:.2f}, x: {x:.3f},\ty: {y:.3f},\tax: {angle_x:.3f},\tay: {angle_y:.3f}")

    utils.loop_every(0.01, every_10ms) # we run our controller at 100 Hz using a linux alarm signal

def run_simulation( initial_ball_position = Point(np.random.uniform(-0.2, 0.2),
                                                  np.random.uniform(-0.2, 0.2))):
    p.connect(p.GUI)
    p.setAdditionalSearchPath("assets")
    plate = p.loadURDF("plate.urdf")
    sphere = p.createMultiBody(0.2
        , p.createCollisionShape(p.GEOM_SPHERE, radius=0.04)
        , basePosition = [initial_ball_position.x,initial_ball_position.y,0.5]
    )

    #zoom to the plate
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0,0,0])

    p.setJointMotorControl2(plate, 0, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2)
    p.setJointMotorControl2(plate, 1, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2)

    p.setGravity(0, 0, -9.8)

    #update the simulation at 100 Hz
    p.setTimeStep(0.01)
    p.setRealTimeSimulation(1)
    return World(plate=plate, sphere=sphere)


if __name__ == "__main__":
    cmd_args = parse_args()
    world = run_simulation()
    run_controller(**vars(cmd_args), world=world)
    
    #time.sleep(10000)

    try:
        while True:          # keep main thread alive
            time.sleep(1)    # but make it interruptible
    except KeyboardInterrupt:
        print("Exiting…")
        p.disconnect()
