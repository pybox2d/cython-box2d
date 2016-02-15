import sys
sys.path.insert(0, 'build/lib.macosx-10.5-x86_64-3.4')

import pybox2d


v = pybox2d.Vec2()
print(v)
print(v.x, v.y, v.is_valid, type(v.is_valid))

v.x = 1
v.y = 2
print('v', v)
copy = pybox2d.Vec2(v)
print('copy', copy)
shape = pybox2d.CircleShape(radius=1.0, center=(0, 1))
print(shape.radius)
print(shape.center)

world = pybox2d.World()
print('gravity', world.gravity)
world.gravity = (0, -10)
print('gravity', world.gravity)

shape = pybox2d.CircleShape(radius=1.0, center=(0, 0))
fixture_defn = pybox2d.FixtureDef(shape=shape, density=0.1)
print(fixture_defn.shape, fixture_defn.density, fixture_defn.sensor,
      type(fixture_defn.sensor))


# bdef = pybox2d.StaticBodyDef()
# print('body type', body.type)
# bdef = pybox2d.DynamicBodyDef()
# print('body type', body.type)
#
#
bdef = pybox2d.DynamicBodyDef()
print(bdef.position)
bdef.position = (0, 2.0)
print(bdef.position)

body = world.create_body(bdef)
print(body)
fixture = body.create_fixture(fixture_defn)
print(fixture)
print(fixture.density)

for i in range(10):
    print(body.world_center)
    world.step(0.01, 8, 8)

pybox2d.Fixture()
