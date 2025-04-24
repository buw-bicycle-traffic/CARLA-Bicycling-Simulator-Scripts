#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Bike Simulator Main Client
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

# additional imports
from socket import *  # used for Arduino connection
import os  # used to define the display position

import carla

from carla import ColorConverter as cc

import argparse
import yaml
import collections
import datetime
import logging
import math
import random
import re
import weakref
import warnings

import subprocess

from CameraManager import CameraManager
from BikeSensor import BikeSensor
from VehicleDynamics import VehicleDynamicsKeyboard, VehicleDynamicsSingleTrack
from DataSync import Server as DataSyncServer

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_c
from pygame.locals import K_q
from pygame.locals import K_r

import numpy as np

spawn_point_z = 0
# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
# World will now make display size and surface resolution accessible by including
# them in the class instantiation. restart() has been updated to take additional
# boolean inputs 'bike' and 'engine'. If set to True restart() will load a bicycle
# Blueprint or apply the new engine setup, respectively. The new engine setup
# has been added to restart().


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.display_size = args.display_size  # set up correct display size for CameraManager
        self.resolution = args.resolution  # set up the correct resolution for CameraManager
        self.actor_role_name = args.rolename  # allow to use a custom player name
        self.map = self.world.get_map()
        # following section was copied from manual_control.py
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        self.hud = hud
        scenario_cfg = {}
        if args.scenario_config is not None:
            with open(args.scenario_config) as f:
                scenario_cfg = yaml.load(f, Loader=yaml.Loader)
        self.instructions_hud = InstructionImageHUD(config=scenario_cfg.get("instruction_images", {}))
        self.data_sync_server = None
        self.data_sync_instr_image_index = None
        if "message_servers" in scenario_cfg:
            if "instruction_images" in scenario_cfg:
                dscfg = scenario_cfg["message_servers"]["instruction_images"]
                self.data_sync_server = DataSyncServer(ip=dscfg["ip"], port=dscfg["port"], fmt=dscfg["fmt"])
                self.data_sync_instr_image_index = dscfg["index"]
        self.player = None
        self._player_blueprint_name = "vehicle.diamondback.century"
        #self._player_blueprint_name = "vehicle.bh.crossbike"
        self._player_start_pos = None
        self._player_start_yaw = None
        self._spawn_point = None
        self._spawn_point_name = None
        self._spawn_point_index = args.spawn_point_index
        if "ego" in scenario_cfg:
            if "blueprint" in scenario_cfg["ego"]:
                self._player_blueprint_name = scenario_cfg["ego"]["blueprint"]
            if "start_pos" in scenario_cfg["ego"]:
                self._player_start_pos = tuple(scenario_cfg["ego"]["start_pos"])
            if "role_name" in scenario_cfg["ego"]:
                self.actor_role_name = scenario_cfg["ego"]["role_name"]
            if "start_yaw" in scenario_cfg["ego"]:
                self._player_start_yaw = scenario_cfg["ego"]["start_yaw"]
            if "spawn_point" in scenario_cfg["ego"]:
                self._spawn_point = carla.Transform(
                carla.Location(*scenario_cfg["ego"]["spawn_point"]["location"]),
                carla.Rotation(**scenario_cfg["ego"]["spawn_point"]["rotation"]))
            if "spawn_point_name" in scenario_cfg["ego"]:
                self._spawn_point_name = scenario_cfg["ego"]["spawn_point_name"]

        self.collision_sensor = None
        self.camera_manager = None
        self._actor_filter = args.filter
        self.camera_params = {k: getattr(args, k, CameraManager.DEFAULT_PARAMS[k]) for k in
                              CameraManager.DEFAULT_PARAMS}
        self.restart()  # On World instantiation use bicycle and new engine setup
        self.world.on_tick(hud.on_world_tick)
    
    def find_spawn_point_by_name(self, name):
        spawn_points = self.world.get_map().get_spawn_points()
        for i, spawn_point in enumerate(spawn_points):
            if spawn_point.name == name:
                return spawn_point
        return None

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        blueprint = self.world.get_blueprint_library().find(self._player_blueprint_name)
        blueprint.set_attribute("role_name", self.actor_role_name)
        # Spawn the player.
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            
            if self._spawn_point_name:
                spawn_point = self.find_spawn_point_by_name(self._spawn_point_name)
                print(spawn_point)
                if spawn_point is None:
                    print(f"Spawn point '{self._spawn_point_name}' not found. Using default.")
                    spawn_point = spawn_points[0] if spawn_points else carla.Transform()
            elif self._spawn_point is not None:
                spawn_point = self._spawn_point
            elif self._player_start_pos is not None:
                spawn_point = carla.Transform(carla.Location(*self._player_start_pos),
                                            carla.Rotation(roll=0, pitch=0, yaw=self._player_start_yaw))
            else:
                print('else')
                print(self._spawn_point_index)
                #spawn_point = spawn_points[422] if spawn_points else carla.Transform() # BUW Map
                #spawn_point = spawn_points[12] if spawn_points else carla.Transform() # Stairstepper Map
                spawn_point = spawn_points[self._spawn_point_index] if spawn_points else carla.Transform() # index-based
            
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.world.tick()
            self.player.set_simulate_physics(True)
            #print(self.player.get_location())
            self.player.set_location(spawn_point.location)
            global spawn_point_z
            spawn_point_z = self.player.get_location().z - 4 
            #print(spawn_point_z)

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.camera_manager = CameraManager(self.player, **self.camera_params)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification("Ego Vehicle Type: " + actor_type)

    def tick(self, clock):
        self.hud.tick(self, clock)
        if self.data_sync_server is not None:
            msgs = self.data_sync_server.get_messages()
        else:
            msgs = []
        if len(msgs) > 0:
            msg = msgs[-1]
            instruction = msg if self.data_sync_instr_image_index is None else msg[self.data_sync_instr_image_index]
            instruction = instruction if instruction != 0 else None
            self.instructions_hud.set_current(instruction)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)
        self.instructions_hud.render(display)

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def calculate_ground(self, location):

        start = location 
        end = carla.Location(x=start.x, y=start.y, z=start.z - 100)  # Cast ray 100 meters down (value extremely large to cover all cases)
        
        hit = self.world.cast_ray(start, end)
        
        if hit:
            return hit[0].location
        else:
            return None  # No ground found within 100 meters

    def calculate_grade(self, location, direction, distance=1.0):

        # Normalize the direction vector
        direction = direction.make_unit_vector()

        # Calculate perpendicular vector pointing to the left
        up_vector = carla.Vector3D(0, 0, 1)
        left_perpendicular = up_vector.cross(direction)
        left_perpendicular = left_perpendicular.make_unit_vector()

        start_location = carla.Location(
            x=location.x + distance * left_perpendicular.x,
            y=location.y + distance * left_perpendicular.y,
            z=location.z + 5
        )    
        

        end_location = carla.Location(
           x=start_location.x + 1 * distance * direction.x,
           y=start_location.y + 1 * distance * direction.y,
           z=start_location.z + 5  # Adjust the height to avoid hitting the vehicle itself
        )
        
        # Get ground points for start and end locations
        start_ground = self.calculate_ground(start_location)
        end_ground = self.calculate_ground(end_location)
        
        if start_ground is None or end_ground is None:
            return None  # Unable to calculate grade
        
        # Calculate the height difference
        height_diff = end_ground.z - start_ground.z
        
        # Calculate the grade as a percentage
        grade = (height_diff / distance) * 100
        
        return grade

# ==============================================================================
# -- Control -----------------------------------------------------------
# ==============================================================================
# DualControl was updated to allow for the simulator sensor outputs to be the
# controller.

class DualControl(object):
    def __init__(self, world, dynamics_model="single-track", rpm_factor=None, **dynamics_model_kwargs):
        self.keyboard_control_mode = False
        self._steer_cache = 0.0
        self._dynamics_model = dynamics_model
        self.world = world 
        if self._dynamics_model == "single-track":
            print('spawn_point_z' + str(spawn_point_z))
            self._vehicle_dynamics_tick = VehicleDynamicsSingleTrack(world.player, start_z=spawn_point_z, 
                                                                             rpm_factor=rpm_factor, **dynamics_model_kwargs)
        else:
            raise NotImplementedError("Invalid dynamics model specified in DualControl.")
        self._keyboard_vehicle_dynamics = VehicleDynamicsKeyboard(world.player)

    def parse_events(self, world, clock, bike_sensor):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_r:
                    self._vehicle_dynamics_single_track.reset_position()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif K_0 < event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_c:
                    self.keyboard_control_mode = not self.keyboard_control_mode
                    world.hud.notification("Keyboard Control Mode" if self.keyboard_control_mode else "Simulator Control Mode")
                else:
                    self._keyboard_vehicle_dynamics.tick(event, pygame.key.get_pressed(), clock.get_time())

        if self.keyboard_control_mode:
            for event in events:
                self._keyboard_vehicle_dynamics.tick(event, pygame.key.get_pressed(), clock.get_time())
        else:
            self._parse_vehicle_controller_input(bike_sensor, clock, world)

    def _parse_vehicle_controller_input(self, bike_sensor, clock, world):
        # request sensor outputs from arduino
        speed, steering = bike_sensor.get_speed_and_steering()
        speed = speed * 1
        current_location = world.player.get_location()
        forward_vector = world.player.get_transform().get_forward_vector()
        current_grade = self.world.calculate_grade(current_location, forward_vector)
        if isinstance(current_grade, float):
            if current_grade > 200:
                current_grade = 200
            if current_grade < -200:
                current_grade = -200
            bike_sensor.set_grade(current_grade)
        # send the sensor readings to the vehicle dynamics module
        if self._dynamics_model == "single-track":
            self._vehicle_dynamics_tick.tick(speed_input=speed, steering_input=steering, time_step=clock.get_time(), start_z=spawn_point_z)

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class InstructionImageHUD(object):
    _defaults = {"size": (100, 100), "pos": (0, 0)}

    def __init__(self, config):
        user_defaults = config.pop("default", {})
        self._images = {}
        self._positions = {}
        for key in config:
            cfg = {**self._defaults, **user_defaults, **config[key]}
            self._images[key] = pygame.transform.smoothscale(pygame.image.load(cfg["path"]), tuple(cfg["size"]))
            self._positions[key] = tuple(cfg["pos"])
        self.current = None

    def clear(self):
        self.current = None

    def set_current(self, image_key):
        self.current = image_key

    def render(self, display):
        if self.current is None:
            return
        if self.current in self._images:
            display.blit(self._images[self.current], self._positions[self.current])
        else:
            warnings.warn("Image key \"" + str(self.current) + "\" not associated with an image.")


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        # font_name = 'courier' if os.name == 'nt' else 'mono'
        # fonts = []
        # for x in pygame.font.get_fonts():
        #     if x is not None and font_name in x:
        #         fonts.append(x)
        # default_font = 'ubuntumono'
        # mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font('ubuntumono')
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = False  # whether the HUD should be displayed by default
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            # 'Map:     % 20s' % world.world.map_name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        print(carla.Transform().location.z)
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
# The game loop now will instantiate an arduino object that stores the connection
# to the arduino. A line of code was added before launching the display to set
# its position. The display got an additional NOFRAME flag. The update rate of
# the client has been reduced from 60 to 20.


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    refresh_rate = int(args.refresh_rate)
    display_size = args.display_size

    scenario_cfg = {}
    if args.scenario_config is not None:
        with open(args.scenario_config) as f:
            scenario_cfg = yaml.load(f, Loader=yaml.Loader)
    dynamics_model = scenario_cfg.get("vehicle_dynamics", {}).get("model", "single-track")
    rpm_factor = scenario_cfg.get("vehicle_dynamics", {}).get("rpm_factor", None)

    try:
        logging.info('listening to server %s:%s', args.host, args.port)
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        bike_sensor = BikeSensor()
        #display = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)  # Set to full screen (investigate further)

        if hasattr(args, "window_pos"):
            os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % tuple(args.window_pos)
        display = pygame.display.set_mode(
            display_size,
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.NOFRAME)

        hud = HUD(display_size[0], display_size[1])
        world = World(client.get_world(), hud, args)
        controller = DualControl(world, dynamics_model=dynamics_model, rpm_factor=rpm_factor)
        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(refresh_rate)
            if controller.parse_events(world, clock, bike_sensor):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()
        if bike_sensor is not None:
            bike_sensor.stop()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
# Three additional arguments have been added: display size, rolename and gamma.
# Rolename and gamma have been directly adapted from manual_control.py. Display
# size is based on the existing resolution argument, which use has been slightly
# repurposed to reflect the decoupling of display size and surface resolution.

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--resolution',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--display_size',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        help='display size (default: 1920x1080)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--refresh_rate',
        metavar='refresh_rate',
        default=20,
        help='refresh rate of clients')
    argparser.add_argument(
        '--config',
        type=str,
        help='Path to YAML config file containing display settings.')
    argparser.add_argument(
        '--display_name',
        metavar='display_name',
        help='Name of display. Used to get parameters from config file.')
    argparser.add_argument(
        '--scenario_config',
        type=str,
        default=None,
        help='Path to YAML config file containing scenario settings.')
    argparser.add_argument(
        '-x', '--spawn-point-index',
        type=int,
        default=None,
        help='Index of the spawn point to use')
    args = argparser.parse_args()

    if args.config is not None:
        # read config file
        with open(args.config) as f:
            cfg = yaml.load(f, Loader=yaml.Loader)
        cfg_default_params = cfg.get("default", {})
        cfg_display_params = cfg.get(args.display_name, {})
        cfg_params = {**cfg_default_params, **cfg_display_params}
        # set parameters
        args.resolution = cfg_params.pop("resolution", args.resolution)
        args.display_size = cfg_params.pop("display_size", args.display_size)
        for param in cfg_params:
            setattr(args, param, cfg_params[param])

    args.resolution = tuple([int(x) for x in args.resolution.split('x')])
    args.display_size = tuple([int(x) for x in args.display_size.split('x')])

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()