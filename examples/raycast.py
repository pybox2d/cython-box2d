import math
from random import random

from pybox2d import (CircleShape, PolygonShape, Vec2)


def setup(world):
    '''Raycast test

    Press 1-5 to drop stuff, d to delete, m to switch callback modes
    '''

    # The ground
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    world.gravity = (0, 0)

    # The various shapes
    w = 1.0
    b = w / (2.0 + math.sqrt(2.0))
    s = math.sqrt(2.0) * b

    # store a set of shapes to create when the user hits 1-5
    world.state['shapes'] = [
        PolygonShape(vertices=[(-0.5, 0), (0.5, 0), (0, 1.5)]),
        PolygonShape(vertices=[(-0.1, 0), (0.1, 0), (0, 1.5)]),
        PolygonShape(vertices=[(0.5 * s, 0), (0.5 * w, b), (0.5 * w, b + s),
                               (0.5 * s, w), (-0.5 * s, w), (-0.5 * w, b + s),
                               (-0.5 * w, b), (-0.5 * s, 0.0)]
                     ),
        PolygonShape(box=(0.5, 0.5)),
        CircleShape(radius=0.5),
    ]
    # keep track of the angle of the raycast
    world.state['angle'] = 0

    # keep track of the bodies the user adds
    world.state['bodies'] = []

    # modes that can be used for raycasting
    world.state['modes'] = ['closest', 'all', 'any']
    world.state['mode'] = 'closest'


def keyboard(world, key, pressed):
    if not pressed:
        return

    mode = world.state['mode']

    if key in ('1', '2', '3', '4', '5'):
        shape_index = '12345'.index(key)
        shape = world.state['shapes'][shape_index]

        pos = (10.0 * (2.0 * random() - 1.0),
               10.0 * (2.0 * random()))

        body = world.create_dynamic_body(position=pos,
                                         angle=(math.pi * (2. * random() - 1.))
                                         )
        body.create_fixture(shape=shape, friction=0.3)

        if isinstance(shape, CircleShape):
            body.angular_damping = 0.02

        world.state['bodies'].append(body)
    elif key == 'd':
        bodies = world.state['bodies']
        if bodies:
            body = bodies.pop(0)
            world.destroy_body(body)
    elif key == 'm':
        modes = world.state['modes']
        idx = (modes.index(mode) + 1) % len(modes)
        world.state['mode'] = modes[idx]
        print('Mode is', world.state['mode'])


def raycast_render(world, renderer):
    world.state['angle'] += 0.25 * math.pi / 180
    mode = world.state['mode']
    angle = world.state['angle']

    length = 11
    point1 = Vec2(0, 10)

    point2 = point1 + (length * math.cos(angle),
                       length * math.sin(angle))

    def draw_hit(hit_point, hit_normal):
        head = hit_point + hit_normal * 0.5

        p1_color = (100, 230, 100)
        s1_color = (200, 200, 200)
        s2_color = (230, 230, 100)

        renderer.draw_point(hit_point, p1_color)
        renderer.draw_line_segment(point1, hit_point, s1_color)
        renderer.draw_line_segment(hit_point, head, s2_color)

    renderer.draw_line_segment(point1, point2, (0, 255, 0, 127))
    if mode in ('closest', 'any'):
        if mode == 'closest':
            item = world.raycast_closest(point1, point2)
        else:
            item = world.raycast_any(point1, point2)

        if item is not None:
            renderer.draw_line_segment(point1, item.point, (255, 0, 0, 255))
            draw_hit(item.point, item.normal)
    else:
        for item in world.raycast_all(point1, point2):
            renderer.draw_line_segment(point1, item.point, (255, 0, 0, 255))
            draw_hit(item.point, item.normal)


if __name__ == '__main__':
    from main_simple import main
    main(setup, keyboard_hook=keyboard, pre_render_hook=raycast_render)
