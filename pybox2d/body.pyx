from defn.body cimport b2Body
from defn.common cimport *


cdef class Body(Base):
    cdef b2Body *thisptr
    cdef public object data
    cdef dict _fixtures
    cdef list _joints

    def __cinit__(self):
        self.data = None
        self._fixtures = {}
        self._joints = []

    def __init__(self):
        pass

    def __hash__(self):
        return pointer_as_key(self.thisptr)

    cdef invalidate(self):
        # underlying b2Body has been destroyed
        for fixture in self.fixtures:
            fixture.invalidate()

        self.thisptr = NULL

    @property
    def valid(self):
        '''If the underlying Box2D Body is still valid'''
        return (self.thisptr != NULL)

    @staticmethod
    cdef from_b2Body(b2Body *b2body):
        body = Body()
        body.thisptr = b2body
        return body

    @safe_property
    def angle(self):
        '''The current world rotation angle in radians.'''
        return self.thisptr.GetAngle()

    @safe_property
    def world_center(self):
        '''The world position of the center of mass.'''
        return to_vec2(self.thisptr.GetWorldCenter())

    @safe_property
    def local_center(self):
        '''The local position of the center of mass.'''
        return to_vec2(self.thisptr.GetLocalCenter())

    @safe_rw_property
    def linear_velocity(self, linear_velocity):
        '''The linear velocity of the center of mass.'''
        if linear_velocity is None:
            return to_vec2(self.thisptr.GetLinearVelocity())

        self.thisptr.SetLinearVelocity(to_b2vec2(linear_velocity))

    @safe_rw_property
    def angular_velocity(self, angular_velocity):
        '''The angular velocity in radians/second.'''
        if angular_velocity is None:
            return self.thisptr.GetAngularVelocity()

        self.thisptr.SetAngularVelocity(angular_velocity)

    @safe_rw_property
    def gravity_scale(self, gravity_scale):
        '''The gravity scale of the body.'''
        if gravity_scale is None:
            return self.thisptr.GetGravityScale()

        self.thisptr.SetGravityScale(gravity_scale)

    @safe_rw_property
    def bullet(self, bullet):
        '''Body treated like a bullet for continuous collision detection?'''
        if bullet is None:
            return self.thisptr.IsBullet()

        self.thisptr.SetBullet(bullet)

    @safe_rw_property
    def linear_damping(self, linear_damping):
        '''Linear damping is use to reduce the linear velocity.

        The damping parameter can be larger than 1.0 but the damping effect
        becomes sensitive to the time step when the damping parameter is large.
        '''
        if linear_damping is None:
            return self.thisptr.GetLinearDamping()

        self.thisptr.SetLinearDamping(linear_damping)

    @safe_rw_property
    def angular_damping(self, angular_damping):
        '''Angular damping is use to reduce the angular velocity.

        The damping parameter can be larger than 1.0 but the damping effect
        becomes sensitive to the time step when the damping parameter is large.
        '''
        if angular_damping is None:
            return self.thisptr.GetAngularDamping()

        self.thisptr.SetAngularDamping(angular_damping)

    @safe_method
    def create_fixture_from_def(self, FixtureDef fixture_defn not None):
        '''Create a fixture from a FixtureDef'''

        fptr = self.thisptr.CreateFixture(fixture_defn.thisptr)
        fixture = Fixture.from_b2Fixture(fptr)
        self._fixtures[pointer_as_key(fptr)] = fixture

        if fixture_defn.data is not None:
            fixture.data = fixture_defn.data

        return fixture

    @safe_method
    def create_fixture(self, **kwargs):
        '''Create a fixture without first creating a FixtureDef

        Parameters
        ----------
        kwargs : dict
            Keyword arguments are all passed to a temporary FixtureDef
            definition.
        '''
        defn = FixtureDef(**kwargs)
        return self.create_fixture_from_def(defn)

    @safe_method
    def create_polygon_fixture(self, box=None, vertices=None, **fx_kwargs):
        '''Create a fixture and attach a polygon shape to it

        There are two ways to specify the vertices:
        1. as the 'box' parameter, which specifies the half-width and
           half-height of the generated rectangle
        2. as a list of vertices

        Parameters
        ----------
        vertices : list of Vec2, optional
            Create a polygon with the vertices given.  If specified, there must
            be at least 3 vertices.
        box : (half-width, half-height), float, optional
            Creates a box with these half-dimensions
        kwargs : dict, optional
            Additional keyword arguments are set on the FixtureDef
        '''
        if 'shape' in fx_kwargs:
            raise ValueError('Shape is implicit in this method')

        shape = PolygonShape(box=box, vertices=vertices)
        return self.create_fixture(shape=shape, **fx_kwargs)

    @safe_method
    def create_edge_fixture(self, vertices, **fx_kwargs):
        '''Create a fixture and attach an edge shape to it

        There are two ways to specify the vertices:
            vertices = (vertex1, vertex2)
            where the edge shape goes from vertex1 to vertex2
        and
            vertices = (vertex0, vertex1, vertex2, vertex3)
            where the edge shape goes from vertex1 to vertex2, but vertex0 and
            vertex3 are adjacent vertices. These are used for smooth collision.
            Chances are you don't need this option.

        Parameters
        ----------
        vertices : list of Vec2
            Create an edge fixture with the vertices given. Must be either 2 or
            4 vertices.
        kwargs : dict, optional
            Additional keyword arguments are set on the FixtureDef
        '''
        if 'shape' in fx_kwargs:
            raise ValueError('Shape is implicit in this method')

        shape = EdgeShape(vertices=vertices)
        return self.create_fixture(shape=shape, **fx_kwargs)

    @safe_method
    def create_circle_fixture(self, radius, center=None, **fx_kwargs):
        '''Create a fixture and attach a circle shape to it


        Parameters
        ----------
        radius : float
            The radius of the circle
        center : Vec2
            The center position of the circle
        kwargs : dict, optional
            Additional keyword arguments are set on the FixtureDef
        '''
        if 'shape' in fx_kwargs:
            raise ValueError('Shape is implicit in this method')

        shape = CircleShape(radius=radius, center=center)
        return self.create_fixture(shape=shape, **fx_kwargs)

    @safe_method
    def destroy_fixture(self, Fixture fixture not None):
        '''Destroy (remove) a fixture from this body

        Parameters
        ----------
        fixture : Fixture
            The fixture to remove
        '''
        cdef b2Fixture *fptr = fixture.thisptr
        del self._fixtures[pointer_as_key(fptr)]

        fixture.invalidate()
        del fixture

        self.thisptr.DestroyFixture(fptr)

    @safe_method
    def _iter_fixtures(self):
        '''Iterate over Fixtures in the body

        Note: users may attempt to delete or add fixtures during iteration, so
        the exposed property returns a full list.
        '''
        cdef b2Fixture *fptr
        fptr = self.thisptr.GetFixtureList()

        while fptr:
            yield self._fixtures[pointer_as_key(fptr)]
            fptr = fptr.GetNext()

    @property  # safety check comes in _iter_fixtures
    def fixtures(self):
        '''The fixtures attached to the body'''
        return list(self._iter_fixtures())

    @safe_property
    def joints(self):
        '''The joints which attach this body to another'''
        return list(self._joints)

    @safe_rw_property
    def transform(self, Transform transform):
        '''The world transform of the body's origin.

        This is the position of the body's origin and rotation.

        Manipulating a body's transform may cause non-physical behavior.
        Note: contacts are updated on the next call to World.step.
        '''
        if transform is None:
            return Transform.from_b2Transform(self.thisptr.GetTransform())

        cdef b2Transform *xf = &transform.transform
        self.thisptr.SetTransform(xf.p, xf.q.GetAngle())

    @safe_rw_property
    def type(self, body_type):
        if body_type is None:
            return BodyType.to_string(self.thisptr.GetType())

        body_type = BodyType.to_enum(body_type)
        self.thisptr.SetType(body_type)

    @safe_property
    def mass(self):
        return self.thisptr.GetMass()

    @safe_property
    def inertia(self):
        return self.thisptr.GetInertia()

    @safe_property
    def position(self):
        return to_vec2(self.thisptr.GetPosition())

    @safe_method
    def apply_force(self, force, point, bool wake):
        self.thisptr.ApplyForce(to_b2vec2(force), to_b2vec2(point), wake)

    @safe_method
    def apply_linear_impulse(self, impulse, point, bool wake):
        self.thisptr.ApplyLinearImpulse(to_b2vec2(impulse), to_b2vec2(point),
                                        wake)

    @safe_method
    def apply_angular_impulse(self, float impulse, bool wake):
        self.thisptr.ApplyAngularImpulse(impulse, wake)

    @safe_method
    def apply_torque(self, float torque, bool wake):
        self.thisptr.ApplyTorque(torque, wake)

    @safe_rw_property
    def sleeping_allowed(self, sleeping_allowed):
        if sleeping_allowed is None:
            return self.thisptr.IsSleepingAllowed()

        self.thisptr.SetSleepingAllowed(sleeping_allowed)

    @safe_rw_property
    def awake(self, awake):
        if awake is None:
            return self.thisptr.IsAwake()

        self.thisptr.SetAwake(awake)

    @safe_rw_property
    def active(self, active):
        if active is None:
            return self.thisptr.IsActive()

        self.thisptr.SetActive(active)

    @safe_rw_property
    def fixed_rotation(self, fixed_rotation):
        if fixed_rotation is None:
            return self.thisptr.IsFixedRotation()

        self.thisptr.SetFixedRotation(fixed_rotation)

    # TODO can't use cpdef with generators?
    def _get_repr_info(self):
        if self.data is not None:
            yield ('data', self.data)

        yield ('position', self.position)
        yield ('angle', self.angle)
        yield ('world_center', self.world_center)
        yield ('local_center', self.local_center)
        yield ('linear_velocity', self.linear_velocity)
        yield ('angular_velocity', self.angular_velocity)
        yield ('transform', self.transform)
        yield ('type', self.type)
        yield ('fixtures', self.fixtures)
        yield ('gravity_scale', self.gravity_scale)
        yield ('linear_damping', self.linear_damping)
        yield ('angular_damping', self.angular_damping)
        yield ('inertia', self.inertia)
        yield ('mass', self.mass)
        # yield ('mass_data', self.mass_data)
