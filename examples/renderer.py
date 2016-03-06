from pybox2d import (CircleShape, PolygonShape, EdgeShape)


class RendererBase(object):
    screen_width = 640
    screen_height = 480
    ppm = 10.0
    target_fps = 60.0

    def __init__(self):
        self.draw_registry = {}
        self.draw_registry[CircleShape] = self.draw_circle_fixture
        self.draw_registry[PolygonShape] = self.draw_polygon_fixture
        self.draw_registry[EdgeShape] = self.draw_edge_fixture

    def fix_vertices(self, vertices):
        return vertices

    def draw_edge_fixture(self, body, fixture):
        pass

    def draw_polygon_fixture(self, body, fixture):
        pass

    def draw_circle_fixture(self, body, fixture):
        pass

    def draw_fixture(self, body, fixture):
        shape = fixture.shape
        draw_function = self.draw_registry[type(shape)]
        return draw_function(body, fixture)
