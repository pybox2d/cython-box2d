import pybox2d

from .fixtures import *
from .util import assert_almost_equal


def test_raycast_all_1(world, subclassed_body):
    body = subclassed_body
    body_cls = subclassed_body.__class__
    assert body_cls is not pybox2d.Body

    fixture = subclassed_body.fixtures[0]
    assert fixture.body.__class__ is body_cls

    shape = fixture.shape
    print('shape', fixture.shape)
    for loop in range(100):
        it = world.raycast_all(body.position - (2.0 * shape.radius, 0.0),
                               body.position + (2.0 * shape.radius, 0.0))
        casted = list(it)
        assert len(casted) == 1
        c0 = casted[0]
        assert_almost_equal(c0.point, (-shape.radius, 0), diff=0.01)
        assert casted[0].fixture is fixture
        assert casted[0].body is body

    for loop in range(100):
        it = world.raycast_all(body.position + (2.0 * shape.radius, 0.0),
                               body.position - (2.0 * shape.radius, 0.0))
        casted = list(it)
        assert len(casted) == 1
        c0 = casted[0]
        assert_almost_equal(c0.point, (shape.radius, 0), diff=0.01)
        assert casted[0].fixture is fixture
        assert casted[0].body is body


def test_raycast_all_2(world, static_circle_fixture, dynamic_circle_fixture):
    static_circle_fixture.body.position = (-1, 0)
    dynamic_circle_fixture.body.position = (1, 0)

    b0 = static_circle_fixture.body
    b1 = dynamic_circle_fixture.body

    for loop in range(100):
        casted = list(world.raycast_all((-5, 0), (5, 0)))
        print('casted', [id(c.fixture) for c in casted])
        print('casted', [type(c.fixture) for c in casted])
        print('casted', [(c.point) for c in casted])
        print('casted', [(c.normal) for c in casted])
        print('casted', [(c.fraction) for c in casted])
        assert len(casted) == 2
        c0 = casted[0]
        for c, body in zip(casted, (b1, b0)):
            pos = body.position - (body.fixtures[0].shape.radius, 0)
            assert_almost_equal(c.point, pos,
                                diff=0.01)
            assert c.fixture is body.fixtures[0]
            assert c.body is body


def test_raycast_any(world, subclassed_body):
    body = subclassed_body
    fixture = subclassed_body.fixtures[0]
    shape = fixture.shape

    for loop in range(100):
        c0 = world.raycast_any(body.position - (2.0 * shape.radius, 0.0),
                               body.position + (2.0 * shape.radius, 0.0))
        assert_almost_equal(c0.point, (-shape.radius, 0), diff=0.01)
        assert c0.fixture is fixture
        assert c0.body is body

    for loop in range(100):
        c0 = world.raycast_any(body.position + (2.0 * shape.radius, 0.0),
                               body.position - (2.0 * shape.radius, 0.0))
        assert_almost_equal(c0.point, (shape.radius, 0), diff=0.01)
        assert c0.fixture is fixture
        assert c0.body is body


def test_raycast_closest(world, static_circle_fixture, dynamic_circle_fixture):
    static_circle_fixture.body.position = (-1, 0)
    dynamic_circle_fixture.body.position = (1, 0)

    b0 = static_circle_fixture.body
    b1 = dynamic_circle_fixture.body

    for loop in range(100):
        c0 = world.raycast_closest((-5, 0), (5, 0))

        body = b0
        pos = body.position - (body.fixtures[0].shape.radius, 0)
        assert_almost_equal(c0.point, pos,
                            diff=0.01)
        assert c0.fixture is body.fixtures[0]
        assert c0.body is body
