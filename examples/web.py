from pybox2d import (FixtureDef, PolygonShape, Body)


class MyBodyClass(Body):
    # test_value = 'abcd'

    def destroyed(self):
        print("{}: I'm being destroyed! ({})".format(self, self.position))

    # it's possible to extend the repr, like so:
    # def _get_repr_info(self):
    #     repr_info = super(MyBodyClass, self)._get_repr_info()
    #     repr_info.append(('test_value', self.test_value))
    #     return repr_info


def setup(world):
    '''This demonstrates a soft distance joint.

    Press: (b) to delete a body, (j) to delete a joint"
    '''

    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-40, 0), (40, 0)])

    fixture = FixtureDef(shape=PolygonShape(box=(0.5, 0.5)),
                         density=5, friction=0.2)

    bodies = [
        world.create_dynamic_body(position=pos, fixtures=fixture,
                                  body_class=MyBodyClass)
        for pos in ((-5, 5),
                    (5, 5),
                    (5, 15),
                    (-5, 15))
    ]

    # Create the joints between each of the bodies and also the ground
    #        bodies                   local anchors
    sets = [((ground,    bodies[0]), ((-10, 0), (-0.5, -0.5))),
            ((ground,    bodies[1]), ((10, 0),  (0.5, -0.5))),
            ((ground,    bodies[2]), ((10, 20), (0.5, 0.5))),
            ((ground,    bodies[3]), ((-10, 20), (-0.5, 0.5))),
            ((bodies[0], bodies[1]), ((0.5, 0), (-0.5, 0))),
            ((bodies[1], bodies[2]), ((0, 0.5), (0, -0.5))),
            ((bodies[2], bodies[3]), ((-0.5, 0), (0.5, 0))),
            ((bodies[3], bodies[0]), ((0, -0.5), (0, 0.5))),
            ]

    # We will define the positions in the local body coordinates, the length
    # will automatically be set by the __init__ of the DistanceJointDef
    joints = [world.create_distance_joint(bodies, local_anchors=local_anchors,
                                          frequency_hz=4.0,
                                          damping_ratio=0.5)
              for bodies, local_anchors in sets]

    world.state['bodies'] = bodies
    world.state['joints'] = joints


def keyboard(world, key, pressed):
    if not pressed:
        return

    if key == 'b':
        bodies = world.state['bodies']
        if bodies:
            body = bodies.pop(0)
            world.destroy_body(body)

    elif key == 'j':
        joints = world.state['joints']
        while joints:
            joint = joints.pop(0)
            if not joint.valid:
                continue

            world.destroy_joint(joint)


if __name__ == '__main__':
    from main_simple import main
    main(setup, keyboard_hook=keyboard)
