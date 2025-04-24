"""
Classes that handle the vehicle dynamics aspect of the simulation.
"""

from __future__ import print_function

import glob
import os
import sys
import time
import math
import csv
from collections import deque

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_m
from pygame.locals import K_q
from pygame.locals import K_s
from pygame.locals import K_w


import time
from scipy.integrate import ode
import numpy as np


class VehicleDynamics:
    """Template class for Vehicle Dynamics."""
    def __init__(self, actor, **kwargs):
        """
        Initialize a VehicleDynamics object.

        :param actor: carla Actor object whose physics are to be controlled.
        :type actor: carla.Actor
        :param world: carla World object for additional calculations.
        :type world: carla.World
        """
        self.player = actor
        # client = carla.Client('localhost', 2000)
        # client.set_timeout(5)
        self.world = self.player.get_world()
        #self.world = world
        self._kwargs = kwargs
        print(f"VehicleDynamics initialized with world: {self.world}")
        print(self.get_vehicle_size(self.player))

    def get_vehicle_size(self, vehicle):
        bbox = vehicle.bounding_box
        extent = bbox.extent
    
        length = extent.x * 2
        width = extent.y * 2
        height = extent.z * 2
    
        return length, width, height
    
    def grade_to_degrees(self, grade_percentage):
        radians = math.atan(grade_percentage / 100)
        degrees = math.degrees(radians)
        return degrees

    def get_ground_height(self, location, distance=1.0):
        if self.world is None:
            print('self.world is none')
            return location.z

        # Get the bicycle's transform to extract its direction
        bicycle_transform = self.player.get_transform()
        forward_vector = bicycle_transform.get_forward_vector()

        # Calculate the left perpendicular vector
        up_vector = carla.Vector3D(0, 0, 1)
        left_perpendicular = up_vector.cross(forward_vector)
        left_perpendicular = left_perpendicular.make_unit_vector()

        # Calculate the left point
        left_point = carla.Location(
            x=location.x + distance * left_perpendicular.x,
            y=location.y + distance * left_perpendicular.y,
            z=location.z
        )

        # Cast ray from the left point
        start = left_point + carla.Location(z=+0.5)
        end = left_point + carla.Location(z=-100)
        hit = self.world.cast_ray(start, end)

        if hit:
            hit_location = hit[0].location
            #print(f"Ray cast hit at z={hit_location.z}")
            return hit_location.z
        else:
            #print(f"No ground detected at left side ({left_point.x}, {left_point.y})")
            return left_point.z

    def reset_position(self):
        raise NotImplementedError("This VehicleDynamics class has no reset_position functionality.")


    def tick(self, **kwargs):
        """Function to be called every simulation step."""
        raise NotImplementedError("Child class must override tick() method.")

class VehicleDynamicsSingleTrack(VehicleDynamics):
    """
    Vehicle Dynamics Model implementing Single Track Model
    developed by Georgios Grigoropoulos and Patrick Malcolm

    For more information on the various parameters of the model, see the full single track model documentation:
      * Vehicle dynamics by Schramm et al. 2018 (DOI: 10.1007/978-3-662-54483-9)
      * https://www.coursera.org/lecture/intro-self-driving-cars/lesson-5-lateral-dynamics-of-bicycle-model-1Opvo
      * https://www.bvl.de/files/1951/1988/1852/2239/10.23773-2017_1.pdf
    """
    def __init__(self, actor, start_yaw=None, rpm_factor=None, start_v=0, start_delta=0, **kwargs):
        """
        Initialize a VehicleDynamicsSingleTrack instance.

        :param actor: carla actor object to control
        :param start_yaw: the starting orientation of the actor. If none, will default to actor's yaw.
        :param rpm_factor: rpm corresponding to 1 m/s
        :param start_v: starting velocity (should never need overridden)
        :param start_delta: starting steering input value (should never need overridden)
        """
        super().__init__(actor)
        transform = self.player.get_transform()
        self.x, self.y, self.z = transform.location.x, transform.location.y, transform.location.z
        self._start_x, self._start_y, self._start_z = self.x, self.y, self.z
        self._start_v, self._start_delta = start_v, start_delta
        self.first_tick = True

        if rpm_factor is None:
            _drum_circumference_simulator = 177.165  # in mm (6.975")
            _drum_radius_simulator = (_drum_circumference_simulator / (2 * np.pi)) / 1000
            _wheel_outer_radius = 0.345  # measured outer radius of rear wheel
            _ratio_drum_wheel_radius = _drum_radius_simulator / _wheel_outer_radius
            self.rpm_factor = 1 / (_ratio_drum_wheel_radius * _drum_radius_simulator * 2 * np.pi / 60)  # m**-1 adjusted for the calculation in the next step
        else:
            self.rpm_factor = rpm_factor
        self.player.set_simulate_physics(False)

        self.b = 0.0  # side slip angle
        if start_yaw is None:
            self.yaw = self.player.get_transform().rotation.yaw
        else:
            self.yaw = start_yaw
        self._start_yaw = self.yaw
        self.sol = ode(self._single_track_func).set_integrator("dopri5", max_step=0.1)
        self.sol.set_initial_value([0, self.b]).set_f_params(start_v, start_delta)

        self.L = 0.1  # in m
        self.f = 0.3  # proportion of width of bike tire in contact with the road surface
        self.br = 0.032 * self.f
        self.ka = 2000000 *self.rpm_factor  # determined experimentally 00000000
        self.c = 0.5 * self.br * self.ka * self.L ** 2  # approximate length of surface covered by the wheel
        self.cav = self.c  # front wheel cornering stiffness
        self.cah = 0   # rear wheel cornering stiffness self.c*0.000000001
        self.lb = 1.02  # bike body length in meters
        self.lv = 0.43  # distance from the center of gravity to the front wheel
        self.lh = self.lb - self.lv  # distances from the center of gravity to the front
        self.theta = 0.05  # yaw inertia (check Table 11.8 page 315; this value is from Meijaard et al.) 0.28
        self.m_bike = 17.3  # mass of bike
        self.m_rider = 65  # mass of rider
        self.m = self.m_bike + self.m_rider  # mass of bike + mass of rider
        # adding self.alt_rpm_factor for our own implementaion to bypass the rpm_factor provided here
        self.alt_rpm_factor = np.float(1/3.6)
        # logging functionality added here with _init_logger and _log_data
        # Initialize logger
        self.log_file = f"vehicle_dynamics_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_buffer = deque(maxlen=1000)  # Increased buffer size to accommodate 30-second interval
        self.last_write_time = time.time()
        self.write_interval = 30.0  # Write to file every 30 seconds
        self._init_logger()

    def _init_logger(self):
        if not os.path.exists('experiment_log'):
            os.makedirs('experiment_log')
        self.log_path = os.path.join('experiment_log', self.log_file)
        with open(self.log_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'X', 'Y', 'Z'])

    def _log_data(self, x, y, z):
        current_time = time.time()
        self.log_buffer.append([current_time, x, y, z])

        if current_time - self.last_write_time >= self.write_interval:
            self._write_buffer_to_file()
            self.last_write_time = current_time

    def _write_buffer_to_file(self):
        if self.log_buffer:
            with open(self.log_path, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(self.log_buffer)
            self.log_buffer.clear()



    def reset_position(self):
        self.x, self.y, self.z = self._start_x, self._start_y, self._start_z
        self.yaw = self._start_yaw
        self.sol = ode(self._single_track_func).set_integrator("dopri5", max_step=0.1)
        self.sol.set_initial_value([0, self.b]).set_f_params(self._start_v, self._start_delta)

    def _single_track_func(self, t, x, v, delta):
        """
        Vehicle dynamics by Schramm et al. 2018 (DOI: 10.1007/978-3-662-54483-9)

        :param t: current time
        :param x: vector of inputs [yaw_rate, side_slip_angle]
        :param v: velocity
        :param delta: steering input angle
        """
        A11 = -(1 / v) * (self.cav * (self.lv ** 2) + self.cah * (self.lh ** 2)) / self.theta
        A12 = -(self.cav * self.lv - self.cah * self.lh) / self.theta
        A21 = -1 - (1 / v ** 2) * (self.cav * self.lv - self.cah * self.lh) / self.m
        A22 = -(1 / v) * (self.cav + self.cah) / self.m
        B11 = self.cav * self.lv / self.theta
        B12 = (1 / v) * (self.cav / self.m)
        A = np.array([[A11, A12], [A21, A22]])
        B = np.array([B11, B12])
        u = delta
        # calculate and return x'=Ax+Bu
        xprime = np.dot(A, x) + np.dot(B, u)
        return np.array([xprime[0], xprime[1]])

    def tick(self, speed_input, steering_input, time_step, start_z):
        """
        Perform a simulation step.

        :param speed_input: speed input value from the sensor (RPM)
        :param steering_input: steering input value from the sensor (degrees)
        :param time_step: time since last tick (milliseconds)
        """
        # convert units
        delta = steering_input*np.pi/180
        # artificial correction factor
        v = np.float(speed_input * 98.15 * 0.75 / self.rpm_factor)
        #v = np.float(speed_input * self.alt_rpm_factor)

        # Perform the integration
        dt = time_step / 1000
        if v == 0:
            yaw_rate, self.b = 0, 0  # no change in yaw rate or side slip angle when not moving
        else:
            self.sol.set_f_params(v, delta)
            yaw_rate, self.b = self.sol.integrate(self.sol.t+dt)

        # store results at class level
        self.yaw += (yaw_rate*180/np.pi) * dt
        # Move and rotate the actor appropriately
        self.x += v * np.cos(self.yaw*np.pi/180)
        self.y += v * np.sin(self.yaw*np.pi/180)
        # originally it was set to 0 assuming flat maps (self.z = 0)
        if self.first_tick:
            self.z = start_z
            self.first_tick = False
        else:
            self.z = self.get_ground_height(carla.Location(self.x, self.y, self.z))
        new_transform = carla.Transform(location=carla.Location(self.x, self.y, self.z),
                                        rotation=carla.Rotation(yaw=self.yaw))
        self.player.set_transform(new_transform)
        #Log the data after setting the new transform
        self._log_data(self.x, self.y, self.z)

    def __del__(self):
        # Ensure any remaining data in the buffer is written when the object is destroyed
        self._write_buffer_to_file()






class VehicleDynamicsKeyboard(VehicleDynamics):
    """Vehicle Dynamics Model using keyboard as input. For debugging purposes."""
    def __init__(self, actor, ):
        super().__init__(actor)
        if isinstance(self.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0

    def tick(self, event, keys, milliseconds):
        if hasattr(event, "key"):
            if event.key == K_q:
                self._control.gear = 1 if self._control.reverse else -1

            elif event.key == K_m:
                self._control.manual_gear_shift = not self._control.manual_gear_shift
                self._control.gear = self.player.get_control().gear
            elif self._control.manual_gear_shift and event.key == K_COMMA:
                self._control.gear = max(-1, self._control.gear - 1)
            elif self._control.manual_gear_shift and event.key == K_PERIOD:
                self._control.gear = self._control.gear + 1

        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]
        self.player.apply_control(self._control)

