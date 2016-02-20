cdef class BodyDef:
    cdef b2BodyDef *thisptr
    cdef object user_data

    def __cinit__(self):
        self.thisptr = new b2BodyDef()

    def __dealloc__(self):
        del self.thisptr

    def __init__(self, type_=b2_staticBody, position=None, angle=0.0,
                 angular_velocity=0.0, linear_damping=0.0, angular_damping=0.0,
                 allow_sleep=True, awake=True, fixed_rotation=False,
                 bullet=False, active=True, gravity_scale=1.0,
                 data=None):

        self.type = type_
        if position is not None:
            self.position = position

        self.angular_velocity = angular_velocity
        self.linear_damping = linear_damping
        self.angular_damping = angular_damping
        self.allow_sleep = allow_sleep
        self.awake = awake
        self.fixed_rotation = fixed_rotation
        self.bullet = bullet
        self.active = active
        self.gravity_scale = gravity_scale
        self.user_data = data

    property position:
        def __get__(self):
            return to_vec2(self.thisptr.position)

        def __set__(self, position):
            self.thisptr.position = to_b2vec2(position)

    property angle:
        def __get__(self):
            return self.thisptr.angle

        def __set__(self, angle):
            self.thisptr.angle = angle

    property linear_velocity:
        def __get__(self):
            return to_vec2(self.thisptr.linearVelocity)

        def __set__(self, linear_velocity):
            self.thisptr.linearVelocity = to_b2vec2(linear_velocity)

    property angular_velocity:
        def __get__(self):
            return self.thisptr.angularVelocity

        def __set__(self, angular_velocity):
            self.thisptr.angularVelocity = angular_velocity

    property linear_damping:
        def __get__(self):
            return self.thisptr.linearDamping

        def __set__(self, linear_damping):
            self.thisptr.linearDamping = linear_damping

    property angular_damping:
        def __get__(self):
            return self.thisptr.angularDamping

        def __set__(self, angular_damping):
            self.thisptr.angularDamping = angular_damping

    property allow_sleep:
        def __get__(self):
            return self.thisptr.allowSleep

        def __set__(self, allow_sleep):
            self.thisptr.allowSleep = allow_sleep

    property awake:
        def __get__(self):
            return self.thisptr.awake

        def __set__(self, awake):
            self.thisptr.awake = awake

    property fixed_rotation:
        def __get__(self):
            return self.thisptr.fixedRotation

        def __set__(self, fixed_rotation):
            self.thisptr.fixedRotation = fixed_rotation

    property bullet:
        def __get__(self):
            return self.thisptr.bullet

        def __set__(self, bullet):
            self.thisptr.bullet = bullet

    property active:
        def __get__(self):
            return self.thisptr.active

        def __set__(self, active):
            self.thisptr.active = active

    property gravity_scale:
        def __get__(self):
            return self.thisptr.gravityScale

        def __set__(self, gravity_scale):
            self.thisptr.gravityScale = gravity_scale

    property type:
        def __get__(self):
            return self.thisptr.type

        def __set__(self, type_):
            self.thisptr.type = type_


cdef class StaticBodyDef(BodyDef):
    def __init__(self, **kwargs):
        super().__init__(self, type_=b2_staticBody, **kwargs)


cdef class KinematicBodyDef(BodyDef):
    def __init__(self, **kwargs):
        BodyDef.__init__(self, type_=b2_kinematicBody, **kwargs)


cdef class DynamicBodyDef(BodyDef):
    def __init__(self, **kwargs):
        BodyDef.__init__(self, type_=b2_dynamicBody, **kwargs)
