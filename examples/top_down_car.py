from __future__ import division

import math
from pybox2d import Body


class GroundArea(Body):
    """An area on the ground that the car can run over"""
    friction_modifier = 0.0


class Tire(Body):
    def create(self, car, max_forward_speed=100.0, max_backward_speed=-20,
               max_drive_force=150, turn_torque=15, max_lateral_impulse=3,
               dimensions=None, density=1.0):

        if dimensions is None:
            dimensions = (0.5, 1.25)

        self.current_traction = 1
        self.turn_torque = turn_torque
        self.max_forward_speed = max_forward_speed
        self.max_backward_speed = max_backward_speed
        self.max_drive_force = max_drive_force
        self.max_lateral_impulse = max_lateral_impulse
        self.ground_areas = []

        self.create_polygon_fixture(box=dimensions, density=density)

    @property
    def forward_velocity(self):
        current_normal = self.get_world_vector((0, 1))
        return current_normal * current_normal.dot(self.linear_velocity)

    @property
    def lateral_velocity(self):
        right_normal = self.get_world_vector((1, 0))
        return right_normal * right_normal.dot(self.linear_velocity)

    def update_friction(self):
        impulse = -self.lateral_velocity * self.mass
        if impulse.length > self.max_lateral_impulse:
            impulse *= self.max_lateral_impulse / impulse.length

        self.apply_linear_impulse(self.current_traction * impulse,
                                  self.world_center, True)

        aimp = (0.1 * self.current_traction * self.inertia *
                -self.angular_velocity)
        self.apply_angular_impulse(aimp, True)

        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.normalized()

        drag_force_magnitude = -2 * current_forward_speed
        self.apply_force(self.current_traction * drag_force_magnitude *
                         current_forward_normal, self.world_center, True)

    def update_drive(self, keys):
        if keys.get('up', False):
            desired_speed = self.max_forward_speed
        elif keys.get('down', False):
            desired_speed = self.max_backward_speed
        else:
            return

        # find the current speed in the forward direction
        current_forward_normal = self.get_world_vector((0, 1))
        current_speed = self.forward_velocity.dot(current_forward_normal)

        # apply necessary force
        force = 0.0
        if desired_speed > current_speed:
            force = self.max_drive_force
        elif desired_speed < current_speed:
            force = -self.max_drive_force
        else:
            return

        force = self.current_traction * force * current_forward_normal
        self.apply_force(force, self.world_center, True)

    def update_turn(self, keys):
        if 'left' in keys:
            desired_torque = self.turn_torque
        elif 'right' in keys:
            desired_torque = -self.turn_torque
        else:
            return

        self.apply_torque(desired_torque, True)

    def add_ground_area(self, ud):
        if ud not in self.ground_areas:
            self.ground_areas.append(ud)
            self.update_traction()

    def remove_ground_area(self, ud):
        if ud in self.ground_areas:
            self.ground_areas.remove(ud)
            self.update_traction()

    def update_traction(self):
        if not self.ground_areas:
            self.current_traction = 1
        else:
            self.current_traction = 0
            mods = [ga.friction_modifier for ga in self.ground_areas]

            max_mod = max(mods)
            if max_mod > self.current_traction:
                self.current_traction = max_mod


class Car(Body):
    vertices = [(1.5, 0.0),
                (3.0, 2.5),
                (2.8, 5.5),
                (1.0, 10.0),
                (-1.0, 10.0),
                (-2.8, 5.5),
                (-3.0, 2.5),
                (-1.5, 0.0),
                ]

    tire_anchors = [(-3.0, 0.75),
                    (3.0, 0.75),
                    (-3.0, 8.50),
                    (3.0, 8.50),
                    ]

    tires = None
    joints = None

    def create(self, world, vertices=None, tire_anchors=None, density=0.1,
               **tire_kws):
        if vertices is None:
            vertices = self.vertices

        self.create_polygon_fixture(vertices=vertices, density=density)
        self.tires = [world.create_dynamic_body(body_class=Tire)
                      for i in range(4)]

        for tire in self.tires:
            tire.create(car=self, **tire_kws)

        if tire_anchors is None:
            anchors = self.tire_anchors

        self.joints = []
        for tire, anchor in zip(self.tires, anchors):
            j = world.create_revolute_joint((self, tire),
                                            local_anchors=(anchor, (0, 0)),
                                            motor=False,
                                            max_motor_torque=1000,
                                            angle_limit=(0, 0),
                                            )

            tire.position = self.world_center + anchor
            self.joints.append(j)

    def update(self, keys, hz):
        for tire in self.tires:
            tire.update_friction()

        for tire in self.tires:
            tire.update_drive(keys)

        # control steering
        lock_angle = math.radians(40.)
        # from lock to lock in 0.5 sec
        turn_speed_per_sec = math.radians(160.)
        turn_per_timestep = turn_speed_per_sec / hz
        desired_angle = 0.0

        if keys.get('left', False):
            desired_angle = lock_angle
        elif keys.get('right', False):
            desired_angle = -lock_angle

        front_left_joint, front_right_joint = self.joints[2:4]
        angle_now = front_left_joint.angle
        angle_to_turn = desired_angle - angle_now

        # clamp the angle to turn to be
        #   -turn_per_timestep <= angle_to_turn <= turn_per_timestep
        angle_to_turn = max(-turn_per_timestep,
                            min(angle_to_turn, turn_per_timestep))

        new_angle = angle_now + angle_to_turn
        # Rotate the tires by locking the limits:
        front_left_joint.limits = (new_angle, new_angle)
        front_right_joint.limits = (new_angle, new_angle)


def setup(world):
    '''Top-down car test

    Press 1-5 to drop stuff, d to delete, m to switch callback modes
    '''

    world.gravity = (0, 0)
    # The boundary walls
    boundary = world.create_static_body(position=(0, 20))
    boundary.create_edge_chain(vertices=[(-30, -30), (-30, 30), (30, 30),
                                         (30, -30), (-30, -30)])

    # create a car
    car = world.create_dynamic_body(body_class=Car)
    car.create(world)

    # and stash it away for further use
    world.state['cars'] = [car]
    world.state['key_status'] = {}

    # create a couple areas that have different levels of traction
    ground_area_1 = world.create_static_body(body_class=GroundArea)
    ground_area_1.friction_modifier = 0.5

    # Set the fixture as a sensor so that the car doesn't collide
    ground_area_1.create_polygon_fixture(box=(9, 7, (-10, 15),
                                              math.radians(20)),
                                         sensor=True)

    ground_area_2 = world.create_static_body(body_class=GroundArea)
    ground_area_2.friction_modifier = 0.2

    # world.monitor_contacts(Tire, GroundArea)
    world.clear_monitoring(Tire, GroundArea)

    # Set the fixture as a sensor so that the car doesn't collide
    ground_area_2.create_polygon_fixture(
        box=(9, 5, (5, 20), math.radians(-40)),
        sensor=True)


def pre_step(world, renderer):
    for car in world.state['cars']:
        car.update(keys=world.state['key_status'],
                   hz=renderer.target_fps)


def pre_render(world, renderer):
    pass


def keyboard(world, key, pressed):
    key_status = world.state['key_status']
    key_status[key] = pressed


if __name__ == '__main__':
    from main_simple import main
    main(setup, keyboard_hook=keyboard, pre_step_hook=pre_step,
         pre_render_hook=pre_render)
