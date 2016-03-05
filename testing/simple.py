import sys
sys.path.insert(0, 'build/lib.macosx-10.5-x86_64-3.4')

import pygame_sdl2
pygame_sdl2.import_as_pygame()

import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

from pybox2d import (World, CircleShape, FixtureDef, StaticBodyDef)

# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = World(gravity=(0, -5))

# And a static body to hold the ground shape
gdef = StaticBodyDef()
gdef.position = (10.5, 2.)

shape = CircleShape(radius=8)
fdef = FixtureDef(shape=shape, density=1, friction=0.3)

# Create a couple dynamic bodies
body = world.create_body_from_def(gdef)

ground_circle = body.create_fixture_from_def(fdef)

dbody = world.create_dynamic_body(position=(10, 10))

shape.radius = 1
dbody.create_fixture_from_def(fdef)


colors = {
    'static': (255, 255, 255, 255),
    'kinematic': (127, 127, 127, 255),
    'dynamic': (127, 100, 100, 255),
}

# Let's play with extending the shape classes to draw for us.


# def my_draw_polygon(polygon, body, fixture):
#     vertices = [(body.transform * v) * PPM for v in polygon.vertices]
#     vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
#     pygame.draw.polygon(screen, colors[body.type], vertices)
# polygonShape.draw = my_draw_polygon


def my_draw_circle(circle, body, fixture):
    position = body.transform * circle.center * PPM
    position = (position[0], SCREEN_HEIGHT - position[1])
    pygame.draw.circle(screen, colors[body.type],
                       [int(x) for x in position],
                       int(circle.radius * PPM),
                       width=1)
    # Note: Python 3.x will enforce that pygame get the integers it requests,
    #       and it will not convert from float.

draw_registry = {}
draw_registry[CircleShape] = my_draw_circle

# --- main game loop ---

running = True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    screen.fill((0, 0, 0, 0))
    # Draw the world
    for body in world.bodies:
        for fixture in body.fixtures:
            shape = fixture.shape
            draw_fcn = draw_registry[type(fixture.shape)]
            draw_fcn(shape, body, fixture)

    # Make Box2D simulate the physics of our world for one step.
    world.step(TIME_STEP, 10, 10)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
