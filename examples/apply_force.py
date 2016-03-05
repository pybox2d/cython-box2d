from math import sqrt, pi
from pybox2d import (FixtureDef, PolygonShape, Transform, Rotation)


def setup(world, joint_gravity=10.0):
    world.gravity = (0.0, 0.0)

    # The boundaries
    ground = world.create_static_body(position=(0, 19))
    ground.create_edge_chain([(-18, -18), (-18, 18),
                              (18, 18), (18, -18),
                              ],
                             closed=True)

    # create the ship body
    ship = world.create_dynamic_body(position=(0, 2), angle=pi,
                                     angular_damping=5,
                                     linear_damping=0.1,)

    # stash it for later
    world.state['ship'] = ship

    # create a couple transforms to help make the shapes that make up the ship
    angle = 0.3524 * pi
    xf1 = Transform(angle=angle,
                    position=Rotation(angle=angle) * (1.0, 0.0))

    angle *= -1.0
    xf2 = Transform(angle=angle,
                    position=Rotation(angle=angle) * (-1.0, 0.0))
    ship_shapes = (PolygonShape(vertices=[xf1 * (-1, 0), xf1 * (1, 0),
                                          xf1 * (0, .5)]),
                   PolygonShape(vertices=[xf2 * (-1, 0), xf2 * (1, 0),
                                          xf2 * (0, .5)]),
                   )

    for shape in ship_shapes:
        ship.create_fixture(density=2.0, shape=shape)

    # add some boxes to fly into
    fixtures = FixtureDef(shape=PolygonShape(box=(0.5, 0.5)),
                          density=1, friction=0.3)
    for i in range(10):
        body = world.create_dynamic_body(position=(0, 5 + 1.54 * i),
                                         fixtures=fixtures)

        # For a circle:
        #   I = 0.5 * m * r * r
        #   ==> r = sqrt(2 * I / m)
        r = sqrt(2.0 * body.inertia / body.mass)

        world.create_friction_joint(
            bodies=(ground, body),
            local_anchors=((0, 0), (0, 0)),
            collide_connected=True,
            max_force=body.mass * joint_gravity,
            max_torque=body.mass * r * joint_gravity
        )


def keyboard(world, key, pressed):
    body = world.state['ship']
    if not body:
        return

    if not pressed:
        return

    if key == 'w':
        f = body.get_world_vector((0.0, -200.0))
        p = body.get_world_point((0.0, 2.0))
        body.apply_force(f, p, True)
    elif key == 'a':
        body.apply_torque(50.0, True)
    elif key == 'd':
        body.apply_torque(-50.0, True)


if __name__ == '__main__':
    from main_simple import main
    main(setup, keyboard_hook=keyboard)
