from __future__ import division
from pybox2d import (FixtureDef, PolygonShape, Vec2)


def create_chain(world, position, attach_to=None, num_planks=30,
                 joint_type='revolute', plank_size=None,
                 density=20, friction=0.2,
                 ):

    if plank_size is None:
        plank_size = (0.6, 0.125)

    plank = FixtureDef(shape=PolygonShape(box=plank_size),
                       density=density,
                       friction=friction,
                       )

    bodies = []
    joints = []

    prev_body = attach_to
    x, y = position
    num_planks = 30
    for i in range(num_planks):
        plank_pos = Vec2(x + i * plank_size[0], y)
        body = world.create_dynamic_body(position=plank_pos,
                                         fixtures=plank)

        if prev_body is not None:
            anchor = plank_pos - (plank_size[0] / 2, 0)
            if joint_type == 'revolute':
                joint = world.create_revolute_joint((prev_body, body),
                                                    anchor=anchor)
            else:
                # You can try a WeldJoint for a slightly different effect.
                joint = world.create_weld_joint((prev_body, body),
                                                anchor=anchor)
            joints.append(joint)

        bodies.append(body)
        prev_body = body

    return bodies, joints


def setup(world):
    '''Chain example'''
    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    create_chain(world, position=(0.5, 25), attach_to=ground)


if __name__ == '__main__':
    from main_simple import main
    main(setup)
