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
        if self.thisptr == NULL:
            raise ValueError('Underlying object was destroyed')
        return pointer_as_key(self.thisptr)

    cpdef destroyed(self):
        '''Destruction callback, overrideable in python'''
        pass

    cdef invalidate(self):
        self.destroyed()

        # underlying b2Body has been destroyed
        for fixture in self._fixtures.values():
            fixture.invalidate()

        self._fixtures.clear()
        del self._joints[:]
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

    @safe_method
    def get_world_vector(self, local_vector):
        '''Get the world coordinates of a vector given the local coordinates.

        Parameters
        ----------
        local_vector : Vec2
            a vector fixed in the body.

        Returns
        -------
        world_vector : Vec2
            the same vector expressed in world coordinates.
        '''
        return to_vec2(self.thisptr.GetWorldVector(to_b2vec2(local_vector)))

    @safe_method
    def get_world_point(self, local_point):
        '''Get the world coordinates of a point given the local coordinates.

        Parameters
        ----------
        local_point : Vec2
            a point on the body measured relative the the body's origin.

        Returns
        -------
        world_point : Vec2
            the same point expressed in world coordinates.
        '''
        return to_vec2(self.thisptr.GetWorldPoint(to_b2vec2(local_point)))

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
        key = hash(fixture)
        self._fixtures[key] = fixture

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
    def create_edge_chain(self, vertices, closed=False, **fx_kwargs):
        '''Create a chain of edge shapes

        For the two ways to specify the vertices, see `create_edge_fixture`

        Parameters
        ----------
        vertices : list of Vec2
            Must have at least two vertices
        closed : bool, optional
            Close the edge chain, where the last specified vertex is connected
            to the first.
        kwargs : dict, optional
            Additional keyword arguments are set on the FixtureDef

        Returns
        -------
        fixtures : list of Fixture
            List of created fixtures
        '''
        if 'shape' in fx_kwargs:
            raise ValueError('Shape is implicit in this method')

        if len(vertices) < 2:
            raise ValueError('Must specify at least 2 vertices')

        fixtures = []
        for v1, v2 in zip(vertices, vertices[1:]):
            fixtures.append(self.create_edge_fixture((v1, v2), **fx_kwargs))

        if closed:
            fixture = self.create_edge_fixture((vertices[-1], vertices[0]),
                                               **fx_kwargs)
            fixtures.append(fixture)

        return fixtures

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
        if not fixture.valid:
            raise RuntimeError('Fixture no longer valid')

        cdef b2Fixture *fptr = fixture.thisptr
        key = hash(fixture)
        del self._fixtures[key]

        fixture.invalidate()
        del fixture

        self.thisptr.DestroyFixture(fptr)

    @safe_method
    def _iter_fixtures(self):
        '''Iterate over Fixtures in the body

        Note: users may attempt to delete or add fixtures during iteration, so
        the exposed property returns a full list.
        '''
        cdef b2Fixture *fptr = self.thisptr.GetFixtureList()

        while fptr:
            key = pointer_as_key(fptr)
            yield self._fixtures[key]
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
        '''Get the total mass of the body, usually in kilograms (kg).'''
        return self.thisptr.GetMass()

    @safe_property
    def inertia(self):
        '''Get the rotational inertia of the body about the local origin
        Usually in kg-m^2.
        '''
        return self.thisptr.GetInertia()

    @safe_property
    def position(self):
        '''Get the world body origin position.'''
        return to_vec2(self.thisptr.GetPosition())

    @safe_method
    def apply_force(self, force, point, bool wake=True):
        '''Apply a force at a world point.

        If the force is not applied at the center of mass, it will generate a
        torque and affect the angular velocity. This wakes up the body.

        Parameters
        ----------
        force : Vec2
            the world force vector, usually in Newtons (N).
        point : Vec2
            the world position of the point of application.
        wake : bool, optional
            also wake up the body
        '''
        self.thisptr.ApplyForce(to_b2vec2(force), to_b2vec2(point), wake)

    @safe_method
    def apply_linear_impulse(self, impulse, point, bool wake=True):
        '''Apply an impulse at a point.

        This immediately modifies the velocity.  It also modifies the angular
        velocity if the point of application is not at the center of mass. This
        wakes up the body.

        Parameters
        ----------
        impulse : Vec2
            the world impulse vector, usually in N-seconds or kg-m/s
        point : Vec2
            the world position of the point of application.
        wake : bool, optional
            also wake up the body
        '''
        self.thisptr.ApplyLinearImpulse(to_b2vec2(impulse), to_b2vec2(point),
                                        wake)

    @safe_method
    def apply_angular_impulse(self, float impulse, bool wake=True):
        '''Apply an angular impulse.

        impulse : float
            impulse the angular impulse in units of kg*m*m/s
        wake : bool, optional
            also wake up the body
        '''
        self.thisptr.ApplyAngularImpulse(impulse, wake)

    @safe_method
    def apply_torque(self, float torque, bool wake=True):
        '''Apply a torque.

        This affects the angular velocity without affecting the linear velocity
        of the center of mass.

        Parameters
        ----------
        torque : float
            about the z-axis (out of the screen), usually in N-m.
        wake : bool, optional
            also wake up the body
        '''
        self.thisptr.ApplyTorque(torque, wake)

    @safe_rw_property
    def sleeping_allowed(self, sleeping_allowed):
        '''You can disable sleeping on this body. If you disable sleeping, the
        body will be woken.'''

        if sleeping_allowed is None:
            return self.thisptr.IsSleepingAllowed()

        self.thisptr.SetSleepingAllowed(sleeping_allowed)

    @safe_rw_property
    def awake(self, awake):
        '''Set the sleep state of the body.

        A sleeping body has very low CPU cost.

        If set to True, the body will be awoken.
        '''
        if awake is None:
            return self.thisptr.IsAwake()

        self.thisptr.SetAwake(awake)

    @safe_rw_property
    def active(self, active):
        '''The active state of the body.

        An inactive body is not simulated and cannot be collided with or woken
        up.  If you pass a flag of true, all fixtures will be added to the
        broad-phase.

        If you pass a flag of false, all fixtures will be removed from the
        broad-phase and all contacts will be destroyed.  Fixtures and joints
        are otherwise unaffected. You may continue to create/destroy fixtures
        and joints on inactive bodies.

        Fixtures on an inactive body are implicitly inactive and will not
        participate in collisions, ray-casts, or queries.  Joints connected to
        an inactive body are implicitly inactive.  An inactive body is still
        owned by a b2World object and remains in the body list.
        '''
        if active is None:
            return self.thisptr.IsActive()

        self.thisptr.SetActive(active)

    @safe_rw_property
    def fixed_rotation(self, fixed_rotation):
        '''Fixed rotation.

        Setting this causes the mass to be reset.
        '''
        if fixed_rotation is None:
            return self.thisptr.IsFixedRotation()

        self.thisptr.SetFixedRotation(fixed_rotation)

    @safe_rw_property
    def mass_data(self, mass_data):
        '''Get or set the mass properties of the body

        It can be used to override the mass properties of the fixtures.

        Note that this changes the center of mass position.
        Note that creating or destroying fixtures can also alter the mass.
        This function has no effect if the body isn't dynamic.

        Parameters:
        -----------
        mass_data : (mass, center, inertia) or MassData
            Where:
            The mass of the shape, usually in kilograms.
            The position of the shape's centroid relative to the shape's
            origin.
            The rotational inertia of the shape about the local origin.
        '''
        cdef b2MassData md
        if mass_data is None:
            self.thisptr.GetMassData(&md)
            return MassData(mass=md.mass,
                            center=to_vec2(md.center),
                            inertia=md.I)

        mass, center, inertia = mass_data
        md.mass = mass
        md.center = to_b2vec2(center)
        md.I = inertia
        self.thisptr.SetMassData(&md)

    @safe_method
    def reset_mass_data(self):
        '''Reset the mass properties to the sum of the mass properties of the
        fixtures.  This normally does not need to be called unless you called
        set_mass_data to override the mass and you later want to reset the
        mass.
        '''
        self.thisptr.ResetMassData()

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
        yield ('mass_data', self.mass_data)
