from pybox2d import (FixtureDef, PolygonShape, Transform, Rotation,
                     Vec2)


def create_pyramid(world, box_half_size=None, rows=20, start_x=None,
                   delta_x=None, delta_y=None, density=5.0):

    if box_half_size is None:
        box_half_size = (0.5, 0.5)
    if start_x is None:
        start_x = Vec2(-7, 0.75)
    if delta_x is None:
        delta_x = (0.5625, 1.25)
    if delta_y is None:
        delta_y = (1.125, 0)

    start_x = Vec2(*start_x)

    fixture = FixtureDef(shape=PolygonShape(box=box_half_size),
                         density=density)

    x = start_x

    bodies = []
    for row in range(rows):
        y = x

        for col in range(row, rows):
            bodies.append(world.create_dynamic_body(position=y,
                                                    fixtures=fixture))
            y = y + delta_y

        x = x + delta_x

    return bodies


def setup(world):
    '''A pyramid'''

    # The ground
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    create_pyramid(world, box_half_size=(0.5, 0.5),
                   rows=20, start_x=(-7, 0.75),
                   delta_x=(0.5625, 1.25),
                   delta_y=(1.125, 0),
                   density=5.0,
                   )



if __name__ == '__main__':
    from main_simple import main
    main(setup)
