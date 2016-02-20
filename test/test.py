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
print(fixture_defn.shape, fixture_defn.density, fixture_defn.sensor,
      type(fixture_defn.sensor))


# bdef = pybox2d.StaticBodyDef()
# print('body type', body.type)
# bdef = pybox2d.DynamicBodyDef()
# print('body type', body.type)
#
#
bdef = pybox2d.DynamicBodyDef(position=(0, 2.0), data='test')
print(bdef.position)
bdef.position = (0, 1.0)
print(bdef.position)
print(bdef.data)

body = world.create_body_from_def(bdef)
fixture = body.create_fixture_from_def(fixture_defn)

# body = world.create_body(None)
body = world.create_body_from_def(bdef, body_class=pybox2d.Body)
fixture = body.create_fixture(shape=shape, density=0.1)

body = world.create_static_body(position=(0, 2.0), data='test',
                                fixtures=[fixture_defn,
                                          dict(shape=shape, density=0.2)])

print(body)

for i in range(10):
    print(body.world_center)
    world.step(0.01, 8, 8)

print(body.world_center)
print('-')

for i, body in enumerate(world.bodies):
    print(i, body, id(body), body.world_center, body.transform)
    for j, fixture in enumerate(body.fixtures):
        print('\t', j, fixture)

    body.data = {}
    body.data['a'] = 1

# body.linear_velocity = None
# body.linear_velocity = 0.1
body = world.bodies[-1]
body.print_position()

body.linear_velocity = Vec2(0, 1)
body.angular_velocity = 0.345
print(body.angular_velocity)
fixture = body.fixtures[0]
print('fixtures', body.fixtures, fixture.valid)

body.destroy_fixture(fixture)
print('fixtures', body.fixtures, fixture.valid)

print(body, body.valid, body.angle)
print(body, body.linear_velocity)
world.destroy_body(body)
print(body, body.linear_velocity)
print(body, body.valid, body.angle)
