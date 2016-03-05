from math import pi
from pybox2d import (CircleShape, FixtureDef, PolygonShape, Vec2,
                     Transform)

# Original inspired by a contribution by roman_m
# Dimensions scooped from APE (http://www.cove.org/ape/index.htm)


def create_leg(world, s, wheel, wheel_anchor, offset, chassis):
    p1, p2 = Vec2(5.4 * s, -6.1), Vec2(7.2 * s, -1.2)
    p3, p4 = Vec2(4.3 * s, -1.9), Vec2(3.1 * s, 0.8)
    p5, p6 = Vec2(6.0 * s, 1.5), Vec2(2.5 * s, 3.7)

    # Use a simple system to create mirrored vertices
    if s > 0:
        poly1 = PolygonShape(vertices=(p1, p2, p3))
        poly2 = PolygonShape(vertices=((0, 0), p5 - p4, p6 - p4))
    else:
        poly1 = PolygonShape(vertices=(p1, p3, p2))
        poly2 = PolygonShape(vertices=((0, 0), p6 - p4, p5 - p4))

    body1 = world.create_dynamic_body(
        position=offset,
        angular_damping=10,
        fixtures=FixtureDef(
            shape=poly1,
            group_index=-1,
            density=1),
    )

    body2 = world.create_dynamic_body(
        position=p4 + offset,
        angular_damping=10,
        fixtures=FixtureDef(
            shape=poly2,
            group_index=-1,
            density=1),
    )

    # Using a soft distance constraint can reduce some jitter.
    # It also makes the structure seem a bit more fluid by
    # acting like a suspension system.
    # Now, join all of the bodies together with distance joints,
    # and one single revolute joint on the chassis
    world.create_distance_joint((body1, body2),
                                damping_ratio=0.5,
                                frequency_hz=10,
                                anchors=(p2 + offset, p5 + offset),
                                )

    world.create_distance_joint((body1, body2),
                                damping_ratio=0.5,
                                frequency_hz=10,
                                anchors=(p3 + offset, p4 + offset),
                                )

    world.create_distance_joint((body1, wheel),
                                damping_ratio=0.5,
                                frequency_hz=10,
                                anchors=(p3 + offset, wheel_anchor + offset),
                                )

    world.create_distance_joint((body2, wheel),
                                damping_ratio=0.5,
                                frequency_hz=10,
                                anchors=(p6 + offset, wheel_anchor + offset),
                                )

    world.create_revolute_joint(
        (body2, chassis),
        anchor=p4 + offset,
    )

    return (body1, body2)


def create_thing(world, motor_speed=2, motor_on=True, offset=None,
                 pivot=None):
    if offset is None:
        offset = Vec2(0, 8)
    if pivot is None:
        pivot = Vec2(0, 0.8)

    # The chassis
    chassis_fixture = FixtureDef(
        shape=PolygonShape(box=(2.5, 1)),
        density=1,
        friction=0.3,
        group_index=-1)

    chassis = world.create_dynamic_body(fixtures=chassis_fixture,
                                        position=pivot + offset)

    # Chassis wheel
    wheel_fixture = FixtureDef(
        shape=CircleShape(radius=1.6),
        density=1,
        friction=0.3,
        group_index=-1)

    wheel = world.create_dynamic_body(fixtures=wheel_fixture,
                                      position=pivot + offset)

    # Add a joint between the chassis wheel and the chassis itself
    motor_joint = world.create_revolute_joint(
        (wheel, chassis),
        anchor=pivot + offset,
        collide_connected=False,
        motor_speed=motor_speed,
        max_motor_torque=400,
        motor=motor_on)

    wheel_anchor = pivot + (0, -0.8)
    legs = []

    legs.append(create_leg(world, -1, wheel, wheel_anchor, offset, chassis))
    legs.append(create_leg(world, 1, wheel, wheel_anchor, offset, chassis))

    wheel.transform = Transform(position=wheel.position,
                                angle=120.0 * pi / 180)
    legs.append(create_leg(world, -1, wheel, wheel_anchor, offset, chassis))
    legs.append(create_leg(world, 1, wheel, wheel_anchor, offset, chassis))

    wheel.transform = Transform(position=wheel.position,
                                angle=-120.0 * pi / 180)
    legs.append(create_leg(world, -1, wheel, wheel_anchor, offset, chassis))
    legs.append(create_leg(world, 1, wheel, wheel_anchor, offset, chassis))
    return {'chassis': chassis,
            'wheel': wheel,
            'legs': legs,
            'motor_joint': motor_joint,
            }


def setup(world):
    '''Theo Jansen test

    Keys: left = a, brake = s, right = d, toggle motor = m
    '''
    ball_count = 40

    # The ground
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-50, 0), (50, 0)])
    ground.create_edge_fixture(vertices=[(-50, 0), (-50, 10)])
    ground.create_edge_fixture(vertices=[(50, 0), (50, 10)])

    # box = FixtureDef(shape=PolygonShape(box=(0.5, 0.5)),
    #                  density=1,
    #                  friction=0.3)
    circle = FixtureDef(shape=CircleShape(radius=0.25),
                        density=1)

    # Create the balls on the ground
    for i in range(ball_count):
        world.create_dynamic_body(
            fixtures=circle,
            position=(-40 + 2.0 * i, 0.5),
        )

    info = create_thing(world)
    world.state.update(info)


def keyboard(world, key, pressed):
    if pressed:
        joint = world.state['motor_joint']
        if key == 'a':
            joint.motor_speed = -2
        elif key == 'd':
            joint.motor_speed = 2
        elif key == 's':
            joint.motor_speed = 0
        elif key == 'm':
            joint.motor_enabled = not joint.motor_enabled
            print('Motor enabled:', joint.motor_enabled)


if __name__ == '__main__':
    from main_simple import main
    main(setup, keyboard_hook=keyboard)
