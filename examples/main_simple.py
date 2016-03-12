import time
from pygame_renderer import PygameRenderer as Renderer

import pygame
from pygame.locals import (QUIT, KEYUP, KEYDOWN, K_ESCAPE)

from pybox2d import World


def setup_backend(screen_width=640, screen_height=480, ppm=12.0,
                  target_fps=60.0):
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
    target_fps : float
        Target frames per second to reach

    Returns
    -------
    renderer : subclass of RendererBase
        The renderer instance
    '''
    return Renderer(screen_width, screen_height, ppm=ppm,
                    target_fps=target_fps)


def default_keyboard_hook(world, key, down):
    print('key', key, 'pressed (down=', down, ')')


def default_pre_step_hook(world, renderer):
    pass


def default_pre_render_hook(world, renderer):
    pass


def main_loop(renderer, world, target_fps=60.0, hooks=None):
    screen = renderer.screen

    if hooks is None:
        hooks = {}

    if 'keyboard' not in hooks:
        keyboard_hook = default_keyboard_hook
    else:
        keyboard_hook = hooks['keyboard']

    if 'pre_render' not in hooks:
        pre_render_hook = default_pre_render_hook
    else:
        pre_render_hook = hooks['pre_render']

    if 'pre_step' not in hooks:
        pre_step_hook = default_pre_step_hook
    else:
        pre_step_hook = hooks['pre_step']

    if 'title' in world.state:
        pygame.display.set_caption(world.state['title'])

    running = True
    clock = pygame.time.Clock()
    time_step = 1.0 / target_fps

    render_time = 0.0
    frame_count = 0

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

        t0 = time.time()
        # Draw the world
        for body in world.bodies:
            for fixture in body.fixtures:
                renderer.draw_fixture(body, fixture)

        for joint in world.joints:
            body1, body2 = joint.bodies
            renderer.draw_line_segment(body1.position, body2.position,
                                       (255, 255, 255, 127))

        render_time += time.time() - t0
        frame_count += 1

        if render_time > 10.0:
            print('Rendering statistics: %d frames in %f sec (%f fps)'
                  '' % (frame_count, render_time, frame_count / render_time))
            render_time = 0.0
            frame_count = 0


        pre_step_hook(world, renderer)

        # Make Box2D simulate the physics of our world for one step.
        world.step(time_step, 10, 10)

        pre_render_hook(world, renderer)
        # Flip the screen and try to keep at the target FPS
        pygame.display.flip()
        clock.tick(target_fps)

    pygame.quit()
    return world


class TestbedWorld(World):
    state = {}


def main(setup_function, target_fps=60.0, keyboard_hook=None,
         pre_render_hook=None, pre_step_hook=None):
    world = TestbedWorld(gravity=(0, -10))
    setup_function(world)

    if 'title' not in world.state and setup_function.__doc__:
        world.state['title'] = setup_function.__doc__

    renderer = setup_backend()
    hooks = {}

    if keyboard_hook is not None:
        hooks['keyboard'] = keyboard_hook
    if pre_render_hook is not None:
        hooks['pre_render'] = pre_render_hook
    if pre_step_hook is not None:
        hooks['pre_step'] = pre_step_hook

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
