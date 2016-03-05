from pygame_renderer import PygameRenderer as Renderer

import pygame
from pygame.locals import (QUIT, KEYUP, KEYDOWN, K_ESCAPE)

from pybox2d import World


def setup_backend(screen_width=640, screen_height=480, ppm=12.0):
    '''Setup the backend

    Parameters
    ----------
    screen_width : int
        Screen width in pixels
    screen_height : int
        Screen height in pixels
    ppm : float
        Box2D deals with meters, but we want to display pixels, so define a
        conversion factor of pixels per meter

    Returns
    -------
    renderer : subclass of RendererBase
        The renderer instance
    '''
    return Renderer(screen_width, screen_height, ppm=ppm)


def default_keyboard_hook(world, key, down):
    print('key', key, 'pressed (down=', down, ')')


def main_loop(renderer, world, target_fps=60.0, hooks=None):
    screen = renderer.screen

    if hooks is None:
        hooks = {}

    if 'keyboard' not in hooks:
        keyboard_hook = default_keyboard_hook
    else:
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
                renderer.draw_fixture(body, fixture)

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

    renderer = setup_backend()
    hooks = {}

    if keyboard_hook is not None:
        hooks['keyboard'] = keyboard_hook

    return main_loop(renderer=renderer, world=world, target_fps=target_fps,
                     hooks=hooks)


def simple_setup(world):
    '''no demo selected'''
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 5), (40, 5)])

    for i in range(2):
        body = world.create_dynamic_body(position=(-8 + 8 * i, 12))
        body.create_polygon_fixture(vertices=[(-0.5, 10.0),
                                              (0.5, 10.0),
                                              (0.0, 11.5),
                                              ],
                                    density=1.0)


if __name__ == '__main__':
    main(simple_setup)
