from cpython.ref cimport PyObject, Py_INCREF, Py_DECREF
from defn.world_callbacks cimport b2ContactListener
from defn.collision cimport b2ManifoldType
from collections import namedtuple



# TODO: inheritance with cdef cppclass? super().__init__() doesn't work,
#       c++-ish syntax doesn't work either
cdef cppclass ContactListener(b2ContactListener):
    PyObject *world

    __init__():
        this.world = NULL

    SetWorld(object world):
        if this.world != NULL:
            return

        Py_INCREF(world)
        this.world = <PyObject*>world

        cdef b2World *b2world = (<World>world).world
        b2world.SetContactListener(this)

    __dealloc__():
        cdef b2World *b2world = (<World>world).world
        b2world.SetContactListener(NULL)
        Py_DECREF(<object>world)


cdef cppclass BulkContactListener(ContactListener):
    __init__(object world):
        this.SetWorld(world)

	# Called when two fixtures begin to touch.
    void BeginContact(b2Contact* contact):
        (<World>world)._begin_bulk_contact(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_bulk_contact(contact)


cdef cppclass OnlyContactListener(ContactListener):
    __init__(object world):
        this.SetWorld(world)

	# Called when two fixtures begin to touch.
    void BeginContact(b2Contact* contact):
        (<World>world)._begin_contact_callback(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_contact_callback(contact)


cdef cppclass FullContactListener(ContactListener):
    __init__(object world):
        this.SetWorld(world)

	# Called when two fixtures begin to touch.
    void BeginContact(b2Contact* contact):
        (<World>world)._begin_contact_callback(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_contact_callback(contact)

    # This is called after a contact is updated. This allows you to inspect a
    # contact before it goes to the solver. If you are careful, you can modify
    # the contact manifold (e.g. disable contact).  A copy of the old manifold
    # is provided so that you can detect changes.
	# Note: this is called only for awake bodies.
	# Note: this is called even when the number of contact points is zero.
	# Note: this is not called for sensors.
	# Note: if you set the number of contact points to zero, you will not
	# get an EndContact callback. However, you may get a BeginContact callback
	# the next step.
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold):
        (<World>world)._pre_solve(contact, oldManifold)

    # This lets you inspect a contact after the solver is finished. This is
    # useful for inspecting impulses.  Note: the contact manifold does not
    # include time of impact impulses, which can be arbitrarily large if the
    # sub-step is small. Hence the impulse is provided explicitly in a separate
    # data structure.
	# Note: this is only called for contacts that are touching, solid, and awake.
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse):
        (<World>world)._post_solve(contact, impulse)



MonitorModeType = new_enum_type(
    'MonitorModeType',
    {'bulk': 'bulk',
     'callbacks': 'callbacks',
     'full_callbacks': 'full_callbacks',
     },
    doc='''Contact monitor mode type enum

Options
-------
bulk: do not get monitor callbacks, only populate Body.contacts
      after every world step
callbacks: get begin_contact and end_contact callbacks on monitored body
           classes
full_callbacks: get begin_contact, end_contact, pre_solve, and post_solve
                callbacks on monitored body classes
''')


ManifoldType = new_enum_type(
    'ManifoldType',
    # TODO: trouble with b2ManifoldType forward declaration?
    {0: 'circles',   # b2ManifoldType::e_circles
     1: 'face_a',  # b2ManifoldType::e_faceA
     2: 'face_b'  # b2ManifoldType::e_faceB
     },
    doc='''Manifold type'''
    )


cdef class ContactID(Base):
    cdef readonly int index_a
    cdef readonly int index_b
    cdef readonly int type_a
    cdef readonly int type_b
    cdef readonly int key

    def __init__(self, index_a, index_b,
                 type_a, type_b, key):
        self.index_a = index_a
        self.index_b = index_b
        self.type_a = type_a
        self.type_b = type_b
        self.key = key

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['type_a', 'type_b',
                             'index_a', 'index_b',
                             'key']]

    @staticmethod
    cdef from_b2ContactID(b2ContactID &b2id):
        return ContactID(b2id.cf.indexA, b2id.cf.indexB,
                         b2id.cf.typeA, b2id.cf.typeB,
                         b2id.key)


cdef class ManifoldPoint(Base):
    cdef public Vec2 local_point
    cdef public float32 normal_impulse
    cdef public float32 tangent_impulse
    cdef public ContactID contact_id

    @staticmethod
    cdef from_b2ManifoldPoint(b2ManifoldPoint &b2p):
        mp = ManifoldPoint()
        mp.local_point = to_vec2(b2p.localPoint)
        mp.normal_impulse = b2p.normalImpulse
        mp.tangent_impulse = b2p.tangentImpulse
        mp.contact_id = ContactID.from_b2ContactID(b2p.id)
        return mp

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['local_point', 'normal_impulse',
                             'tangent_impulse', 'contact_id']
                ]


cdef class WorldManifold(Base):
    cdef public Vec2 normal
    cdef public tuple points
    cdef public tuple separations

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['normal', 'points', 'separations']
                ]


cdef class ContactImpulse(Base):
    cdef readonly tuple normal_impulses
    cdef readonly tuple tangent_impulses

    def __init__(self, normal_impulses=None, tangent_impulses=None):
        self.normal_impulses = normal_impulses
        self.tangent_impulses = tangent_impulses

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['normal_impulses', 'tangent_impulses']
                ]

    @staticmethod
    cdef from_b2ContactImpulse(const b2ContactImpulse *b2impulse):
        normal = tuple(b2impulse.normalImpulses[i]
                       for i in range(b2impulse.count))
        tangent = tuple(b2impulse.tangentImpulses[i]
                        for i in range(b2impulse.count))
        impulse = ContactImpulse(normal_impulses=normal,
                                 tangent_impulses=tangent)
        return impulse


cdef class Manifold(Base):
    '''
    A manifold for two touching convex shapes.

    Box2D supports multiple types of contact:
    - clip point versus plane with radius
    - point versus point with radius (circles)

    The local point usage depends on the manifold type:
    - circles: the local center of circleA
    - face_a: the center of faceA
    - face_b: the center of faceB

    Similarly the local normal usage:
    - circles: not used
    - face_a: the normal on polygonA
    - face_b: the normal on polygonB

    We store contacts in this way so that position correction can account for
    movement, which is critical for continuous physics.

    All contact scenarios must be expressed in one of these types.  This
    structure is stored across time steps, so we keep it small.
    '''
    cdef b2Manifold *_manifold

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['points', 'local_normal', 'local_point',
                             'type_']
                ]

    @property
    def points(self):
        return [ManifoldPoint.from_b2ManifoldPoint(self._manifold.points[point])
                for point in range(self._manifold.pointCount)]

    @property
    def local_normal(self):
        return to_vec2(self._manifold.localNormal)

    @property
    def local_point(self):
        return to_vec2(self._manifold.localPoint)

    @property
    def type_(self):
        return str(ManifoldType(self._manifold.type))

    @staticmethod
    cdef from_b2Manifold(const b2Manifold *b2manifold):
        manifold = Manifold()
        manifold._manifold = <b2Manifold *>b2manifold
        return manifold


cdef class ContactInfo(Base):
    '''Contact information'''
    cdef public Body body_a
    cdef public Body body_b
    cdef public Fixture fixture_a
    cdef public Fixture fixture_b
    cdef public int child_index_a
    cdef public int child_index_b
    cdef public bool touching
    cdef b2Manifold *_b2manifold
    cdef Manifold _manifold
    cdef b2Contact *contact

    cdef float32 _friction
    cdef float32 _restitution
    cdef float32 _tangent_speed
    cdef bool _enabled

    cdef _setup(self, World world, b2Contact *contact,
                bool allow_modification=False):
        cdef const b2Fixture *fptr_a = contact.GetFixtureA()
        cdef const b2Body *bptr_a = fptr_a.GetBody()

        cdef const b2Fixture *fptr_b = contact.GetFixtureB()
        cdef const b2Body *bptr_b = fptr_b.GetBody()

        self.body_a = world._bodies[pointer_as_key(<void*>bptr_a)]
        self.fixture_a = self.body_a._fixtures[pointer_as_key(<void*>fptr_a)]
        self.body_b = world._bodies[pointer_as_key(<void*>bptr_b)]
        self.fixture_b = self.body_b._fixtures[pointer_as_key(<void*>fptr_b)]

        self.child_index_a = contact.GetChildIndexA()
        self.child_index_b = contact.GetChildIndexB()
        self.touching = contact.IsTouching()
        self._b2manifold = contact.GetManifold()

        self._friction = contact.GetFriction()
        self._restitution = contact.GetRestitution()
        self._enabled = contact.IsEnabled()
        self._tangent_speed = contact.GetTangentSpeed()

        if allow_modification:
            self.contact = contact
        else:
            self.contact = NULL

    @property
    def manifold(self):
        '''The manifold for two touching convex shapes (see Manifold)

        NOTE/TODO: this is not a safe method, and probably shouldn't be
        for speed reasons. If you access this beyond when you should,
        a segmentation fault (or unexpected results) will occur
        '''
        if self._manifold is None:
            self._manifold = Manifold.from_b2Manifold(self._b2manifold)
        return self._manifold

    @property
    def bodies(self):
        '''The bodies involved in the contact'''
        return (self.body_a, self.body_b)

    @property
    def fixtures(self):
        '''The fixtures involved in the contact'''
        return (self.fixture_a, self.fixture_b)

    @property
    def child_indices(self):
        '''Get the child primitive index for the fixtures'''
        return (self.child_index_a, self.child_index_b)

    cpdef _get_repr_info(self):
        return [(attr, getattr(self, attr))
                for attr in ['bodies', 'fixtures', 'friction', 'restitution',
                             'tangent_speed', 'child_indices', 'enabled',
                             'touching', 'manifold']]

    property friction:
        '''Contact friction'''
        def __get__(self):
            if self.contact != NULL:
                return self.contact.GetFriction()
            else:
                return self._friction

        def __set__(self, friction):
            if self.contact != NULL:
                self.contact.SetFriction(friction)
            else:
                raise RuntimeError('Contact is no longer valid')

    property restitution:
        '''Contact restitution'''
        def __get__(self):
            if self.contact != NULL:
                return self.contact.GetRestitution()
            else:
                return self._restitution

        def __set__(self, restitution):
            if self.contact != NULL:
                self.contact.SetRestitution(restitution)
            else:
                raise RuntimeError('Contact is no longer valid')

    property enabled:
        '''Contact enabled

        Enable/disable this contact. This can be used inside the pre-solve
        contact listener. The contact is only disabled for the current time
        step (or sub-step in continuous collisions).
        '''
        def __get__(self):
            if self.contact != NULL:
                return self.contact.IsEnabled()
            else:
                return self._enabled

        def __set__(self, enabled):
            if self.contact != NULL:
                self.contact.SetEnabled(enabled)
            else:
                raise RuntimeError('Contact is no longer valid')

    property tangent_speed:
        '''Contact tangent speed

        Set the desired tangent speed for a conveyor belt behavior. In meters
        per second.
        '''
        def __get__(self):
            if self.contact != NULL:
                return self.contact.GetTangentSpeed()
            else:
                return self._tangent_speed

        def __set__(self, tangent_speed):
            if self.contact != NULL:
                self.contact.SetTangentSpeed(tangent_speed)
            else:
                raise RuntimeError('Contact is no longer valid')

    def reset_friction(self):
        '''Reset the contact friction to the default value'''
        if self.contact != NULL:
            return self.contact.ResetFriction()
        else:
            raise RuntimeError('Contact is no longer valid')

    def reset_restitution(self):
        '''Reset the contact restitution to the default value'''
        if self.contact != NULL:
            return self.contact.ResetRestitution()
        else:
            raise RuntimeError('Contact is no longer valid')

    def get_other_body(self, Body body):
        if body is self.body_a:
            return self.body_b
        else:
            return self.body_a


cdef class ContactBeginInfo(ContactInfo):
    pass


cdef class ContactEndInfo(ContactInfo):
    pass


cdef class ContactPreSolveInfo(ContactInfo):
    pass


cdef class ContactPostSolveInfo(ContactInfo):
    pass
