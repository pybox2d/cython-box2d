try:
    import pygame_sdl2
except ImportError:
    pass
else:
    print('Using pygame_sdl2')
    pygame_sdl2.import_as_pygame()

import pygame
from pygame.locals import (QUIT, KEYUP, KEYDOWN, K_ESCAPE)

from renderer import RendererBase


class PygameRenderer(RendererBase):
    colors = {'static': (255, 255, 255, 255),
              'kinematic': (127, 127, 127, 255),
              'dynamic': (127, 100, 100, 255),
              }

    def __init__(self, screen_width, screen_height, ppm=12.0):
        super().__init__()

        self.screen_width = screen_width
        self.screen_height = screen_height
        self.ppm = ppm
        self.screen = pygame.display.set_mode((screen_width, screen_height),
                                              0, 32)

    def fix_vertices(self, vertices):
        half_width = self.screen_width / 2.
        # convert from meters to pixels
        # flip in y
        # move over a bit in x
        return [(int(v0 * self.ppm + half_width),
                 int(self.screen_height - v1 * self.ppm))
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
                           int(circle.radius * self.ppm),
                           width=1)


