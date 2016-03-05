import pybox2d

# ugly import syntax, otherwise linter detects test args as shadowing
# fixtures
from .fixtures import *
from .util import assert_almost_equal


def test_circle():
    basic_circle = pybox2d.CircleShape(radius=1.0, center=(0, 0.1))
    assert basic_circle.radius == 1.0
    assert_almost_equal(basic_circle.center, (0, 0.1))

    md = basic_circle.compute_mass(1.0)


def test_create_circle_fixture(static_body, circle):
    fixture = static_body.create_circle_fixture(radius=circle.radius,
                                                center=circle.center)

    assert fixture.shape.radius == circle.radius
    assert_almost_equal(fixture.shape.center, circle.center)


def test_create_fixture_from_def_circle(state, dynamic_body, circle):
    density = 2.0

    fixture_defn = pybox2d.FixtureDef(shape=circle, density=density)
    fixture = dynamic_body.create_fixture_from_def(fixture_defn)

    assert fixture.shape.radius == circle.radius
    assert_almost_equal(fixture.shape.center, circle.center)
    assert fixture.density == density


def test_create_fixture(static_body, circle, density=1.0):
    fixture = static_body.create_fixture(shape=circle, density=density)

    assert fixture.shape.radius == circle.radius
    assert_almost_equal(fixture.shape.center, circle.center)
    assert fixture.density == density


def test_destroy_fixture(static_body, circle, density=1.0):
    assert circle.valid
    fixture = static_body.create_fixture(shape=circle, density=density)
    shape = fixture.shape
    assert shape.valid
    static_body.destroy_fixture(fixture)
    assert circle.valid
    assert not shape.valid


def test_destroy_fixture(world, static_body, circle, density=1.0):
    assert circle.valid
    fixture = static_body.create_fixture(shape=circle, density=density)
    shape = fixture.shape
    assert shape.valid
    assert static_body.valid
    assert fixture.valid

    world.destroy_body(static_body)

    with pytest.raises(RuntimeError):
        # body already removed
        static_body.destroy_fixture(fixture)

    assert circle.valid
    assert not shape.valid
    assert not static_body.valid
    assert not fixture.valid
