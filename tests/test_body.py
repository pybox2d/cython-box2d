import pytest
import pybox2d

from .util import (assert_almost_equal, world_with_static_body,
                   world_with_body_circle)

basic_circle = pybox2d.CircleShape(radius=1.0, center=(0, 0.1))


class BodyClass(pybox2d.Body):
    name = None

    def print_position(self):
        print("{}'s position is: {}".format(self.name, self.position))


def test_body_subclass():
    world = pybox2d.World()
    body = world.create_dynamic_body(position=(0, 2.0), data='test',
                                     body_class=BodyClass)
    body.name = 'body1'
    body.print_position()

    body.create_circle_fixture(radius=1.0, center=(0, 0))

    assert isinstance(body, BodyClass)
    assert body.name == 'body1'

    body2 = world.create_dynamic_body(position=(0, 1.0), data='test',
                                      body_class=BodyClass)

    body2.create_circle_fixture(radius=0.1, center=(0, 0))
    body2.name = 'body2'
    body2.print_position()

    assert isinstance(body2, BodyClass)
    assert body2.name == 'body2'

    for i in range(10):
        print([body.world_center for body in world.bodies])
        world.step(0.01, 8, 8)

    print(body.world_center)
    print('-')

    for i, body in enumerate(world.bodies):
        print(i, body, id(body), body.world_center, body.transform)
        for j, fixture in enumerate(body.fixtures):
            print('\t', j, fixture)

        assert isinstance(body, BodyClass)
        body.data = {}
        body.data['name'] = body.name


def test_body_destroyed():
    with world_with_body_circle() as (world, body, fixture):
        assert body.valid

        world.destroy_body(body)
        with pytest.raises(RuntimeError):
            body.linear_velocity
            # pybox2d will no longer crash when the underlying c++ object is
            # destroyed

        assert not body.valid
