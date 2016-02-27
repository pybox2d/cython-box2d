from contextlib import closing

import pybox2d
from pybox2d import Vec2

from .util import (assert_almost_equal, world_with_static_body)

basic_circle = pybox2d.CircleShape(radius=1.0, center=(0, 0.1))


def test_circle():
    assert basic_circle.radius == 1.0
    assert_almost_equal(basic_circle.center, (0, 0.1))


def test_create_circle_fixture():
    with world_with_static_body() as (world, body):
        fixture = body.create_circle_fixture(radius=basic_circle.radius,
                                             center=basic_circle.center)

        assert fixture.shape.radius == basic_circle.radius
        assert_almost_equal(fixture.shape.center, basic_circle.center)


def test_create_fixture_from_def_circle():
    density = 2.0

    with world_with_static_body() as (world, body):
        fixture_defn = pybox2d.FixtureDef(shape=basic_circle, density=density)
        fixture = body.create_fixture_from_def(fixture_defn)

        assert fixture.shape.radius == basic_circle.radius
        assert_almost_equal(fixture.shape.center, basic_circle.center)
        assert fixture.density == density

    with world_with_static_body() as (world, body):
        fixture = body.create_fixture(shape=basic_circle, density=density)

        assert fixture.shape.radius == basic_circle.radius
        assert_almost_equal(fixture.shape.center, basic_circle.center)
        assert fixture.density == density

