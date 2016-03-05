from pybox2d import (FixtureDef, PolygonShape, Transform, Rotation,
                     Vec2)


def setup(world):
    '''A pyramid'''

    # The ground
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    box_half_size = (0.5, 0.5)
    box_rows = 20

    x = Vec2(-7, 0.75)
    delta_x = (0.5625, 1.25)
    delta_y = (1.125, 0)

    fixture = FixtureDef(shape=PolygonShape(box=box_half_size),
                         density=5.0)

    for row in range(box_rows):
        y = x

        for col in range(row, box_rows):
            world.create_dynamic_body(position=y, fixtures=fixture)
            y = y + delta_y

        x = x + delta_x


if __name__ == '__main__':
    from main_simple import main
    main(setup)
