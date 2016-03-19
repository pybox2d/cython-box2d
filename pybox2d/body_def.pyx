cdef class BodyDef(Base):
    '''Body Definition

    Parameters
    ----------
    type_ : BodyType, optional
        ('static', 'dynamic', 'kinematic')
        Note: if a dynamic body would have zero mass, the mass is set to one.
    position : Vec2, optional
        The world position of the body. Avoid creating bodies at the origin
        since this can lead to many overlapping shapes.
    angle : float, optional
	    The world angle of the body in radians.
    angular_velocity : float, optional
        The angular velocity of the body.
    linear_velocity : float, optional
        The linear velocity of the body's origin in world co-ordinates.
    linear_damping : float, optional
        Linear damping is use to reduce the linear velocity. The damping
        parameter can be larger than 1.0 but the damping effect becomes
        sensitive to the time step when the damping parameter is large.
    angular_damping : float, optional
        Angular damping is use to reduce the angular velocity. The damping
        parameter can be larger than 1.0 but the damping effect becomes
        sensitive to the time step when the damping parameter is large.
    allow_sleep : bool, optional
        Set this flag to false if this body should never fall asleep. Note that
        this increases CPU usage.
    awake : bool, optional
        Is this body initially awake or sleeping?
    fixed_rotation : bool, optional
        Should this body be prevented from rotating? Useful for characters.
    bullet : bool, optional
        Is this a fast moving body that should be prevented from tunneling
        through other moving bodies? Note that all bodies are prevented from
        tunneling through kinematic and static bodies. This setting is only
        considered on dynamic bodies.
        You should use this flag sparingly since it increases processing time.
    active : bool, optional
	    Does this body start out active?
    gravity_scale : float, optional
        Scale the gravity applied to this body.
    data : any, optional
        Use this to store application specific body data.
    fixtures : list or FixtureDef, optional
        List of fixture to create on the body
    '''
    cdef b2BodyDef *thisptr
    cdef public object data
    cdef public list fixtures

    def __cinit__(self):
        self.thisptr = new b2BodyDef()

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, type_='static', position=None, angle=0.0,
                 angular_velocity=0.0, linear_velocity=None,
                 linear_damping=0.0, angular_damping=0.0,
                 allow_sleep=True, awake=True, fixed_rotation=False,
                 bullet=False, active=True, gravity_scale=1.0,
                 data=None, fixtures=None):

        self.type_ = type_
        if position is not None:
            self.position = position

        self.angular_velocity = angular_velocity
        if linear_velocity is not None:
            self.linear_velocity = linear_velocity

        self.linear_damping = linear_damping
        self.angular_damping = angular_damping
        self.allow_sleep = allow_sleep
        self.awake = awake
        self.fixed_rotation = fixed_rotation
        self.bullet = bullet
        self.active = active
        self.gravity_scale = gravity_scale
        self.data = data

        if isinstance(fixtures, FixtureDef):
            fixtures = [fixtures]

        self.fixtures = fixtures

    property position:
        '''The world position of the body.

        Avoid creating bodies at the origin since this can lead to many
        overlapping shapes.
        '''
        def __get__(self):
            return to_vec2(self.thisptr.position)

        def __set__(self, position):
            self.thisptr.position = to_b2vec2(position)

    property angle:
        '''The world angle of the body in radians.'''
        def __get__(self):
            return self.thisptr.angle

        def __set__(self, angle):
            self.thisptr.angle = angle

    property linear_velocity:
        '''The linear velocity of the body's origin in world co-ordinates.'''
        def __get__(self):
            return to_vec2(self.thisptr.linearVelocity)

        def __set__(self, linear_velocity):
            self.thisptr.linearVelocity = to_b2vec2(linear_velocity)

    property angular_velocity:
        '''The angular velocity of the body.'''
        def __get__(self):
            return self.thisptr.angularVelocity

        def __set__(self, angular_velocity):
            self.thisptr.angularVelocity = angular_velocity

    property linear_damping:
        '''Linear damping is use to reduce the linear velocity.

        The damping parameter can be larger than 1.0 but the damping effect
        becomes sensitive to the time step when the damping parameter is large.
        '''
        def __get__(self):
            return self.thisptr.linearDamping

        def __set__(self, linear_damping):
            self.thisptr.linearDamping = linear_damping

    property angular_damping:
        '''Angular damping is use to reduce the angular velocity.

        The damping parameter can be larger than 1.0 but the damping effect
        becomes sensitive to the time step when the damping parameter is large.
        '''
        def __get__(self):
            return self.thisptr.angularDamping

        def __set__(self, angular_damping):
            self.thisptr.angularDamping = angular_damping

    property allow_sleep:
        '''Set this flag to false if this body should never fall asleep.

        Note that this increases CPU usage.
        '''
        def __get__(self):
            return self.thisptr.allowSleep

        def __set__(self, allow_sleep):
            self.thisptr.allowSleep = allow_sleep

    property awake:
        '''Is this body initially awake or sleeping?'''
        def __get__(self):
            return self.thisptr.awake

        def __set__(self, awake):
            self.thisptr.awake = awake

    property fixed_rotation:
        '''Should this body be prevented from rotating?

        Useful for characters.
        '''
        def __get__(self):
            return self.thisptr.fixedRotation

        def __set__(self, fixed_rotation):
            self.thisptr.fixedRotation = fixed_rotation

    property bullet:
        '''Is this a fast moving body that should be prevented from tunneling?

        Note that all bodies are prevented from tunneling through kinematic and
        static bodies. This setting is only considered on dynamic bodies.

        You should use this flag sparingly since it increases processing time.
        '''
        def __get__(self):
            return self.thisptr.bullet

        def __set__(self, bullet):
            self.thisptr.bullet = bullet

    property active:
	    '''Does this body start out active?'''
        def __get__(self):
            return self.thisptr.active

        def __set__(self, active):
            self.thisptr.active = active

    property gravity_scale:
        '''Scale the gravity applied to this body.'''
        def __get__(self):
            return self.thisptr.gravityScale

        def __set__(self, gravity_scale):
            self.thisptr.gravityScale = gravity_scale

    property type_:
        '''Body type

        ('static', 'dynamic', 'kinematic')
        Note: if a dynamic body would have zero mass, the mass is set to one.
        '''
        def __get__(self):
            return BodyType.to_string(self.thisptr.type)

        def __set__(self, type_):
            self.thisptr.type = BodyType.to_enum(type_)

    cpdef _get_repr_info(self):
        repr_info = []
        if self.data is not None:
            repr_info.append(('data', self.data))

        repr_info.extend([
            ('type', self.type),
            ('position', self.position),
            ('angular_velocity', self.angular_velocity),
            ('linear_damping', self.linear_damping),
            ('angular_damping', self.angular_damping),
            ('allow_sleep', self.allow_sleep),
            ('awake', self.awake),
            ('fixed_rotation', self.fixed_rotation),
            ('bullet', self.bullet),
            ('active', self.active),
            ('gravity_scale', self.gravity_scale),
            ])

        if self.fixtures:
            repr_info.append(('fixtures', self.fixtures))
        return repr_info


cdef class StaticBodyDef(BodyDef):
    def __init__(self, **kwargs):
        super().__init__(type_='static', **kwargs)


cdef class KinematicBodyDef(BodyDef):
    def __init__(self, **kwargs):
        super().__init__(type_='kinematic', **kwargs)


cdef class DynamicBodyDef(BodyDef):
    def __init__(self, **kwargs):
        super().__init__(type_='dynamic', **kwargs)
