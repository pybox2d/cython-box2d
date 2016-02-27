import pybox2d
import contextlib


def assert_almost_equal(a, b, diff=0.001):
    try:
        len(a)
    except Exception:
        a, b = [a], [b]

    for i, (ai, bi) in enumerate(zip(a, b)):
        assert abs(ai - bi) <= diff, 'Index {}'.format(i)


@contextlib.contextmanager
def world_context(gravity=(0, -10)):
    world = pybox2d.World(gravity=gravity)
    try:
        yield world
    finally:
        assert len(world.bodies) == 0


@contextlib.contextmanager
def world_with_static_body(gravity=(0, -10)):
    world = pybox2d.World(gravity=gravity)
    body = world.create_static_body()
    try:
        yield world, body
    finally:
        pass
        # the thought is some checking will go here, hence the context manager


@contextlib.contextmanager
def world_with_body_circle(gravity=(0, -10), radius=0.2,
                           center=None):
    if center is None:
        center = (0, 0)
    world = pybox2d.World(gravity=gravity)
    body = world.create_dynamic_body(position=(0, 0))
    fixture = body.create_circle_fixture(center=center,
                                         radius=radius)
    try:
        yield world, body, fixture
    finally:
        pass
        # the thought is some checking will go here, hence the context manager
