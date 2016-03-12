from pybox2d import (EdgeShape, FixtureDef, PolygonShape,
                     Body)


class ConveyorBeltBody(Body):
    def contact_pre_solve(self, contact, old_manifold):
        fixture = self.fixtures[0]
        contact_fixtures = contact.fixtures

        if contact_fixtures[0] is fixture:
            contact.tangent_speed = 5.0
        elif contact_fixtures[1] is fixture:
            contact.tangent_speed = -5.0


def setup(world):
    "Conveyor Belt"

    ground = world.create_static_body()
    ground.create_edge_fixture(vertices=[(-20, 0), (20, 0)])

    world.monitor_mode = 'full_callbacks'
    world.monitor_contacts(ConveyorBeltBody, Body)

    # Platform
    platform = world.create_static_body(position=(-5, 5), allow_sleep=False,
                                        body_class=ConveyorBeltBody)
    platform.create_polygon_fixture(friction=0.8, box=(10.0, 5.0))

    # Boxes
    for i in range(5):
        box = world.create_dynamic_body(position=(-10.0 + 2.0 * i, 10.0))
        box.create_polygon_fixture(density=20.0, box=(0.5, 0.5))

    world.state['platform'] = platform


if __name__ == '__main__':
    from main_simple import main
    main(setup)
