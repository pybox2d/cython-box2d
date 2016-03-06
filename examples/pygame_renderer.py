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


def half_intensity_color(color):
    return (color[0] / 2, color[1] / 2, color[2] / 2, 127)


class PygameRenderer(RendererBase):
    colors = {'static': (255, 255, 255, 255),
              'kinematic': (127, 127, 127, 255),
              'dynamic': (127, 100, 100, 255),
              }

    def __init__(self, screen_width, screen_height, ppm=12.0,
                 target_fps=60.0):
        super().__init__()

        self.screen_width = screen_width
        self.screen_height = screen_height
        self.ppm = ppm
        self.target_fps = target_fps
        self.screen = pygame.display.set_mode((screen_width, screen_height),
                                              0, 32)

    def to_screen(self, vertices):
        half_width = self.screen_width / 2.
        # convert from meters to pixels
        # flip in y
        # move over a bit in x
        return [(int(v0 * self.ppm + half_width),
                 int(self.screen_height - v1 * self.ppm))
                for v0, v1 in vertices]

    def draw_edge_fixture(self, body, fixture):
        edge = fixture.shape
        v0, v1 = [body.transform * v
                  for v in edge.main_vertices]
        self.draw_line_segment(v0, v1, self.colors[body.type])

    def draw_polygon_fixture(self, body, fixture):
        polygon = fixture.shape
        vertices = [body.transform * v
                    for v in polygon.vertices]
        self.draw_solid_polygon(vertices, self.colors[body.type])

    def draw_circle_fixture(self, body, fixture):
        circle = fixture.shape
        transform = body.transform
        center = transform * circle.center
        axis = transform.rotation * (1, 0)
        self.draw_circle(center, int(circle.radius * self.ppm),
                         axis, self.colors[body.type])

    def draw_solid_polygon(self, vertices, color):
        """Draw a filled polygon given the screen vertices with the specified
        color.
        """
        half_color = half_intensity_color(color)
        vertices = self.to_screen(vertices)
        pygame.draw.polygon(self.screen, half_color, vertices, width=0)
        # TODO pygame_sdl2 has issues with non-filled polygons:
        # pygame.draw.polygon(self.screen, color, vertices, width=1.0)
        for v1, v2 in zip(vertices, vertices[1:]):
            pygame.draw.aaline(self.screen, color, v1, v2)
        pygame.draw.aaline(self.screen, color, vertices[-1], vertices[0])

    def draw_polygon(self, vertices, color):
        """Draw a wireframe polygon given the screen vertices with the
        specified color.
        """
        vertices = self.to_screen(vertices)
        # if len(vertices) == 2:
        #     pygame.draw.aaline(self.screen, color, vertices[0], vertices)
        # else:
        pygame.draw.polygon(self.screen, color, vertices, 1)

    def draw_circle(self, center, radius, axis, color):
        """Draw a wireframe circle given the center, radius, axis of
        orientation and color.
        """
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        half_color = half_intensity_color(color)
        center = self.to_screen([center])[0]
        pygame.draw.circle(self.screen, half_color, center, radius, 0)
        pygame.draw.circle(self.screen, color, center, radius, 1)
        pygame.draw.aaline(self.screen, (255, 0, 0), center,
                           (center[0] - radius * axis[0],
                            center[1] + radius * axis[1]))

    def draw_solid_circle(self, center, radius, axis, color):
        """Draw a solid circle given the center, radius, axis of orientation
        and color.
        """
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        center = self.to_screen([center])[0]
        half_color = half_intensity_color(color)
        pygame.draw.circle(self.screen, half_color, center, radius, 0)
        pygame.draw.circle(self.screen, color, center, radius, 1)
        pygame.draw.aaline(self.screen, (255, 0, 0), center,
                           (center[0] - radius * axis[0],
                            center[1] + radius * axis[1]))

    def draw_transform(self, transform):
        p1 = transform.position
        rotation = transform.rotation

        p2 = self.to_screen(p1 + self.ppm * rotation.x_axis)
        p3 = self.to_screen(p1 + self.ppm * rotation.y_axis)
        p1 = self.to_screen(p1)
        pygame.draw.aaline(self.screen, (255, 0, 0), p1, p2)
        pygame.draw.aaline(self.screen, (0, 255, 0), p1, p3)

    def draw_aabb(self, aabb, color):
        """
        Draw a wireframe around the AABB with the given color.
        """
        points = [(aabb.lower_bound.x, aabb.lower_bound.y),
                  (aabb.upper_bound.x, aabb.lower_bound.y),
                  (aabb.upper_bound.x, aabb.upper_bound.y),
                  (aabb.lower_bound.x, aabb.upper_bound.y)]

        points = self.to_screen(points)
        pygame.draw.aalines(self.screen, color, True, points)

    def draw_line_segment(self, p1, p2, color):
        """Draw the line segment from p1-p2 with the specified color."""
        points = self.to_screen([p1, p2])
        pygame.draw.aaline(self.screen, color, *points)

    def draw_point(self, center, color):
        radius = 1
        center = self.to_screen([center])[0]
        pygame.draw.circle(self.screen, color, center, radius, 0)
