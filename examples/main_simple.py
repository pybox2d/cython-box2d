try:
    import pygame_sdl2
except ImportError:
    pass
else:
    print('Using pygame_sdl2')
    pygame_sdl2.import_as_pygame()


import pygame
from pygame.locals import (QUIT, KEYUP, KEYDOWN, K_ESCAPE)
from pybox2d import (World, FixtureDef)
from pybox2d import (EdgeShape, CircleShape, PolygonShape)

from renderer import RendererBase

# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 12.0  # pixels per meter


class Renderer(RendererBase):
    colors = {'static': (255, 255, 255, 255),
              'kinematic': (127, 127, 127, 255),
              'dynamic': (127, 100, 100, 255),
              }

    def __init__(self, screen_width, screen_height):
        super().__init__()

        self.screen_width = screen_width
        self.screen_height = screen_height
        self.screen = pygame.display.set_mode((screen_width, screen_height),
                                              0, 32)

    def fix_vertices(self, vertices):
        half_width = self.screen_width / 2.
        # convert from meters to pixels
        # flip in y
        # move over a bit in x
        return [(int(v0 * PPM + half_width),
                 int(self.screen_height - v1 * PPM))
                for v0, v1 in vertices]

    def draw_edge_fixture(self, body, fixture):
        edge = fixture.shape
        vertices = [body.transform * v
                    for v in edge.main_vertices]
        v0, v1 = self.fix_vertices(vertices)
        pygame.draw.line(self.screen, self.colors[body.type], v0, v1)

    def draw_polygon_fixture(self, body, fixture):
        polygon = fixture.shape
        vertices = [body.transform * v
                    for v in polygon.vertices]
        vertices = self.fix_vertices(vertices)
        pygame.draw.polygon(self.screen, self.colors[body.type], vertices)

    def draw_circle_fixture(self, body, fixture):
        circle = fixture.shape
        center = self.fix_vertices([body.transform * circle.center])[0]
        pygame.draw.circle(self.screen, self.colors[body.type], center,
                           int(circle.radius * PPM),
                           width=1)


def setup_backend(screen_width=640, screen_height=480):
    return Renderer(screen_width, screen_height)


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
