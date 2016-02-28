import pytest
import pybox2d
from pybox2d import (FixtureDef, PolygonShape)

from .util import assert_almost_equal
from .fixtures import *


def create_bridge(world, ground, size, offset, plank_count, friction=0.6,
                  density=1.0):
    """
    Create a bridge with plank_count planks,
    utilizing rectangular planks of size (width, height).
    The bridge should start at x_offset, and continue to
    roughly x_offset+width*plank_count.
    The y will not change.
    """
    width, height = size
    x_offset, y_offset = offset
    half_height = height / 2
    plank = FixtureDef(shape=PolygonShape(box=(width / 2, half_height)),
                       friction=friction,
                       density=density,
                       )

    bodies = []
    joints = []

    prev_body = ground
    for i in range(plank_count):
        body = world.create_dynamic_body(
            position=(x_offset + width * i, y_offset),
            fixtures=plank,
        )
        bodies.append(body)

        joint = world.create_revolute_joint(
            bodies=(prev_body, body),
            anchor=(x_offset + width * (i - 0.5), y_offset)
        )
        joints.append(joint)

        prev_body = body

    joint = world.create_revolute_joint(
        bodies=(prev_body, ground),
        anchor=(x_offset + width * (plank_count - 0.5), y_offset),
    )

    joints.append(joint)
    return bodies, joints


def test_revolute_joint(world, ground):
    bodies, joints = create_bridge(world, ground, size=(1.0, 0.25),
                                   offset=(-14.5, 5), plank_count=30,
                                   friction=0.2, density=20)
    j0 = joints[0]
    assert isinstance(j0, pybox2d.RevoluteJoint)
    assert j0.bodies == (ground, bodies[0])


def test_distance_joint(world, ground, dynamic_body):
    world.create_distance_joint((ground, dynamic_body),
                                local_anchors=((0, 0), (0, 0)))


def test_joint_deletion(world, ground):
    bodies, joints = create_bridge(world, ground, size=(1.0, 0.25),
                                   offset=(-14.5, 5), plank_count=30,
                                   friction=0.2, density=20)
    j0 = joints[0]
    assert isinstance(j0, pybox2d.RevoluteJoint)
    assert j0.bodies == (ground, bodies[0])
    assert j0 in ground.joints

    assert all(j.valid for j in joints)

    assert j0.valid
    world.destroy_joint(j0)
    assert not j0.valid
    assert j0 not in ground.joints

    for body in bodies:
        world.destroy_body(body)

    assert all(not j.valid for j in joints)
    assert all(not body.valid for body in bodies)
