from cpython.ref cimport PyObject, Py_INCREF, Py_DECREF
from defn.world_callbacks cimport b2ContactListener

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
        (<World>world)._begin_contact(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_contact(contact)


cdef cppclass OnlyContactListener(ContactListener):
    __init__(object world):
        this.SetWorld(world)

	# Called when two fixtures begin to touch.
    void BeginContact(b2Contact* contact):
        (<World>world)._begin_contact(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_contact(contact)


cdef cppclass FullContactListener(ContactListener):
    __init__(object world):
        this.SetWorld(world)

	# Called when two fixtures begin to touch.
    void BeginContact(b2Contact* contact):
        (<World>world)._begin_contact(contact)

	# Called when two fixtures cease to touch.
    void EndContact(b2Contact* contact):
        (<World>world)._end_contact(contact)

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
    {0: 'bulk',
     1: 'callbacks',
     2: 'full_callbacks',
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
'''
    )
