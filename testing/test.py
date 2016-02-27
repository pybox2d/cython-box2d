import sys
sys.path.insert(0, 'build/lib.macosx-10.5-x86_64-3.4')

import pybox2d
from pybox2d import Vec2

v = pybox2d.Vec2()
print(v)
print(v.x, v.y, v.is_valid, type(v.is_valid))

v.x = 1
v.y = 2
print('v', v)
copy = pybox2d.Vec2(v)
print('copy', copy)
shape = pybox2d.CircleShape(radius=1.0, center=(0, 0.1))
print(shape.radius)
print(shape.center)

class BodyClass(pybox2d.Body):
    def print_position(self):
        print(self.world_center)

    # TODO will not work, can't use 'cpdef' with generators
    # def _repr_info(self):
    #     yield from super()._repr_info()
    #     yield ('test', 'a')


world = pybox2d.World(default_body_class=BodyClass)
print('gravity', world.gravity)
world.gravity = (0, -10)
print('gravity', world.gravity)

shape = pybox2d.CircleShape(radius=1.0, center=(0, 0.2))
fixture_defn = pybox2d.FixtureDef(shape=shape, density=0.1)
print(fixture_defn)


# bdef = pybox2d.StaticBodyDef()
# print('body type', body.type)
# bdef = pybox2d.DynamicBodyDef()
# print('body type', body.type)
#
#
bdef = pybox2d.DynamicBodyDef(position=(0, 2.0), data='test')
print(bdef)
bdef.position = (0, 1.0)
print(bdef)

body = world.create_body_from_def(bdef)
fixture = body.create_fixture_from_def(fixture_defn)

# body = world.create_body(None)
body = world.create_body_from_def(bdef, body_class=pybox2d.Body)
fixture = body.create_fixture(shape=shape, density=0.1)
fixture = body.create_circle_fixture(radius=0.1, center=(1, 2),
                                     density=0.1)
fixture = body.create_polygon_fixture(box=(3, 3), density=0.1)
print('created polygon fixture with box settings', fixture)
fixture = body.create_polygon_fixture(vertices=[(-3.0, -3.0), (3.0, -3.0),
                                                (3.0, 3.0), (-3.0, 3.0)],
                                      density=0.1)
fixture = body.create_edge_fixture(vertices=[(-3.0, -3.0), (3.0, -3.0)],
                                   density=0.1)

print('created polygon fixture with vertices', fixture)
body = world.create_static_body(position=(0, 2.0), data='test',
                                fixtures=[fixture_defn,
                                          dict(shape=shape, density=0.2)])

print(body)

for i in range(10):
    print([body.world_center for body in world.bodies])
    world.step(0.01, 8, 8)

print(body.world_center)
print('-')

for i, body in enumerate(world.bodies):
    print(i, body, id(body), body.world_center, body.transform)
    for j, fixture in enumerate(body.fixtures):
        print('\t', j, fixture)

    body.data = {}
    body.data['a'] = i

# body.linear_velocity = None
# body.linear_velocity = 0.1
body = world.bodies[-1]
body.print_position()
#
# body.linear_velocity = Vec2(0, 1)
# body.angular_velocity = 0.345
# print(body.angular_velocity)
# fixture = body.fixtures[0]
# print('fixtures', body.fixtures, fixture.valid)
#
# body.destroy_fixture(fixture)
# print('fixtures', body.fixtures, fixture.valid)
#
# print(body, body.valid, body.angle)
# print(body, body.linear_velocity)
#
# print('pre-destroy, body.valid=', body.valid)
# world.destroy_body(body)
# try:
#     print(body, body.linear_velocity)
# except RuntimeError as ex:
#     print('pybox2d will no longer crash when the underlying c++ object is destroyed')
#
# print('post-destroy, body.valid=', body.valid)
