from __future__ import division
from pybox2d import (CircleShape, FixtureDef, PolygonShape)


def create_bridge(world, ground, size, offset, plank_count, friction=0.6,
                  density=1.0):
    """
    Create a bridge with plank_count planks,
    utilizing rectangular planks of size (width, height).
    The bridge should start at x_offset, and continue to
    roughly x_offset+width*plank_count.
    The y will not change.
    """
    width, height = size
    x_offset, y_offset = offset
    half_height = height / 2
    plank = FixtureDef(shape=PolygonShape(box=(width / 2, half_height)),
                       friction=friction,
                       density=density,
                       )

    bodies = []
    prev_body = ground
    for i in range(plank_count):
        body = world.create_dynamic_body(
            position=(x_offset + width * i, y_offset),
            fixtures=plank,
        )
        bodies.append(body)

        world.create_revolute_joint(
            bodies=(prev_body, body),
            anchor=(x_offset + width * (i - 0.5), y_offset)
        )

        prev_body = body

    world.create_revolute_joint(
        bodies=(prev_body, ground),
        anchor=(x_offset + width * (plank_count - 0.5), y_offset),
    )
    return bodies


def setup(world):
    '''bridge test'''
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    bodies = create_bridge(world, ground=ground,
                           size=(1.0, 0.25),
                           offset=(-14.5, 5),
                           plank_count=30,
                           friction=0.2, density=20)

    fixture = FixtureDef(
        shape=PolygonShape(vertices=[(-0.5, 0.0),
                                     (0.5, 0.0),
                                     (0.0, 1.5),
                                     ]),
        density=1.0
    )

    for i in range(2):
        world.create_dynamic_body(
            position=(-8 + 8 * i, 12),
            fixtures=fixture,
        )

    fixture = FixtureDef(shape=CircleShape(radius=0.5), density=1)
    for i in range(3):
        world.create_dynamic_body(
            position=(-6 + 6 * i, 10),
            fixtures=fixture,
        )

    world.state['bridge_bodies'] = bodies


if __name__ == '__main__':
    from main_simple import main
    main(setup)
