import pytest

import pybox2d


@pytest.fixture(scope="function")
def state():
    # hold some function-level state like the world object
    # as we don't want it to get destroyed if, say, the function is only
    # testing a b2Body
    return {}


@pytest.fixture(scope='function')
def world(state):
    world = pybox2d.World(gravity=(0, -10))
    state['world'] = world
    return world


@pytest.fixture(scope='function')
def static_body(state, world, position=None):
    if position is None:
        position = (0, 0)
    body = world.create_static_body(position=position)
    state['body'] = body
    return body


@pytest.fixture(scope='function')
def dynamic_body(state, world, gravity=None, position=None):
    if position is None:
        position = (0, 0)
    body = world.create_dynamic_body(position=position)
    state['body'] = body
    return body


@pytest.fixture(scope='function')
def dynamic_body2(state, world, gravity=None, position=None):
    if position is None:
        position = (0, 0)
    body = world.create_dynamic_body(position=position)
    state['body'] = body
    return body


@pytest.fixture(scope='function')
def ground(world):
    vertices = [(-40, 0), (40, 0)]
    fixture = pybox2d.FixtureDef(shape=pybox2d.EdgeShape(vertices=vertices))
    return world.create_static_body(fixtures=fixture)


@pytest.fixture(scope='function')
def circle(radius=0.2, center=None):
    if center is None:
        center = (0, 0)
    return pybox2d.CircleShape(center=center, radius=radius)


@pytest.fixture(scope='function')
def static_circle_fixture(state, static_body, radius=0.2, center=None):
    if center is None:
        center = (0, 0)
    fixture = static_body.create_circle_fixture(center=center, radius=radius)
    state['fixture'] = fixture
    return fixture


@pytest.fixture(scope='function')
def dynamic_circle_fixture(state, dynamic_body, radius=0.2, center=None):
    if center is None:
        center = (0, 0)
    fixture = dynamic_body.create_circle_fixture(center=center, radius=radius)
    state['fixture'] = fixture
    return fixture
