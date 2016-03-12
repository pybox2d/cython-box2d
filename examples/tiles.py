from math import ceil, log

from pybox2d import Vec2
from pyramid import create_pyramid


def setup(world):
    '''This stress-tests the dynamic tree broad-phase.

    It also shows that tile based collision is _not_ smooth due to Box2D not
    knowing about adjacency.
    '''

    a = 0.5

    def ground_positions():
        N = 200
        M = 10
        position = Vec2(0, 0)
        for i in range(M):
            position.x = -N * a
            for j in range(N):
                yield position
                position.x += 2.0 * a
            position.y -= 2.0 * a

    ground = world.create_static_body(position=(0, -a))

    for position in ground_positions():
        ground.create_polygon_fixture(box=(a, a, position, 0))

    create_pyramid(world)


def pre_render_hook(world, renderer):
    status = world.status
    print(status)
    height = status.tree_height
    leaf_count = status.proxy_count
    min_node_count = 2 * leaf_count - 1
    min_height = ceil(log(float(min_node_count)) / log(2))
    print('Dynamic tree height=%d, min=%d' % (height, min_height))


if __name__ == '__main__':
    from main_simple import main
    world = main(setup, pre_render_hook=pre_render_hook)
