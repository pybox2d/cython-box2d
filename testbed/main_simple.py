try:
    import pygame_sdl2
except ImportError:
    pass
else:
    print('Using pygame_sdl2')
    pygame_sdl2.import_as_pygame()


import pygame
from pygame.locals import (QUIT, KEYUP, KEYDOWN, K_ESCAPE)
from pybox2d import (World, CircleShape, PolygonShape)
from pybox2d import (FixtureDef, EdgeShape)

# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter

colors = {
    'static': (255, 255, 255, 255),
    'kinematic': (127, 127, 127, 255),
    'dynamic': (127, 100, 100, 255),
}

def fix_vertices(screen, vertices):
    screen_width, screen_height = screen.get_width(), screen.get_height()
    half_width = screen_width / 2.
    # convert from meters to pixels
    # flip in y
    # move over a bit in x
    return [(int(v0 * PPM + half_width),
             int(screen_height - v1 * PPM))
            for v0, v1 in vertices]


def pygame_draw_edge(screen, edge, body, fixture):
    vertices = [body.transform * v
                for v in edge.main_vertices]
    v0, v1 = fix_vertices(screen, vertices)
    pygame.draw.line(screen, colors[body.type], v0, v1)


def pygame_draw_polygon(screen, polygon, body, fixture):
    vertices = [body.transform * v
                for v in polygon.vertices]
    vertices = fix_vertices(screen, vertices)
    pygame.draw.polygon(screen, colors[body.type], vertices)


def pygame_draw_circle(screen, circle, body, fixture):
    center = fix_vertices(screen, [body.transform * circle.center])[0]
    pygame.draw.circle(screen, colors[body.type], center,
                       int(circle.radius * PPM),
                       width=1)


def setup_backend(screen_width=640, screen_height=480):
    screen = pygame.display.set_mode((screen_width, screen_height), 0, 32)

    draw_registry = {}
    draw_registry[CircleShape] = pygame_draw_circle
    draw_registry[PolygonShape] = pygame_draw_polygon
    draw_registry[EdgeShape] = pygame_draw_edge
    return screen, draw_registry


def keyboard_hook(world, key, down):
    print('key', key, 'pressed (down=', down, ')')


def main_loop(screen, world, draw_registry, target_fps=60.0,
              hooks=None):

    if hooks is None:
        hooks = {}

    if 'keyboard' not in hooks:
        hooks['keyboard'] = keyboard_hook

    keyboard_hook = hooks['keyboard']

    if 'title' in world.state:
        pygame.display.set_caption(world.state['title'])

    running = True
    clock = pygame.time.Clock()
    time_step = 1.0 / target_fps

    while running:
        # Check the event queue
        for event in pygame.event.get():
            escape_pressed = (event.type == KEYDOWN and event.key == K_ESCAPE)
            if event.type == QUIT or escape_pressed:
                # The user closed the window or pressed escape
                running = False
            elif event.type in (KEYDOWN, KEYUP):
                key_name = pygame.key.name(event.key).decode('utf-8')
                keyboard_hook(world, key_name.lower(), (event.type == KEYDOWN))

        screen.fill((0, 0, 0, 0))

        # Draw the world
        for body in world.bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                try:
                    draw_fcn = draw_registry[type(fixture.shape)]
                except KeyError:
                    # this backend can't draw the shape - draw it as a circle
                    # for now
                    draw_fcn = pygame_draw_circle

                draw_fcn(screen, shape, body, fixture)

        # Make Box2D simulate the physics of our world for one step.
        world.step(time_step, 10, 10)

        # Flip the screen and try to keep at the target FPS
        pygame.display.flip()
        clock.tick(target_fps)

    pygame.quit()


class TestbedWorld(World):
    state = {}


def main(setup_function, target_fps=60.0,
         keyboard_hook=None):
    world = TestbedWorld(gravity=(0, -10))
    setup_function(world)

    if 'title' not in world.state and setup_function.__doc__:
        world.state['title'] = setup_function.__doc__

    screen, draw_registry = setup_backend()
    hooks = {'keyboard': keyboard_hook
             }
    print(screen)
    return main_loop(screen=screen, world=world, draw_registry=draw_registry,
                     target_fps=target_fps, hooks=hooks)


def simple_setup(world):
    '''no demo selected'''
    ground_shape = EdgeShape(vertices=[(-40, 5), (40, 5)])
    world.create_static_body(
        fixtures=[FixtureDef(shape=ground_shape)],
    )

    fixture = FixtureDef(
        shape=PolygonShape(vertices=[(-0.5, 10.0),
                                     (0.5, 10.0),
                                     (0.0, 11.5),
                                     ]),
        density=1.0
    )

    for i in range(2):
        world.create_dynamic_body(
            position=(-8 + 8 * i, 12),
            fixtures=fixture,
        )


if __name__ == '__main__':
    main(simple_setup)
