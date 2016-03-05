from cpython.ref cimport PyObject, Py_INCREF, Py_DECREF
import threading
from collections import namedtuple
from threading import Thread


cdef cppclass RayCastCallback(b2RayCastCallback):
    PyObject *context

    __init__(object context):
        Py_INCREF(context)
        this.context = <PyObject*>context
        # ref: http://stackoverflow.com/questions/17067544

    __dealloc__():
        Py_DECREF(<object>context)

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                          const b2Vec2& normal, float32 fraction):
        cdef RaycastIterable ctx = <RaycastIterable>this.context

        ret = ctx.update(fixture.GetBody(), fixture, point, normal, fraction)
        return ret



RaycastTuple = namedtuple('RaycastTuple', 'body fixture point normal fraction')
RaycastTuple.__doc__ = '''RaycastTuple

fixture: the fixture hit by the ray
point: the point of initial intersection
normal: the normal vector at the point of intersection
'''


class RaycastResponseWrapper(Base):
    '''You control how the ray cast proceeds by setting a float:

        -1.0: ignore this fixture and continue
         0.0: terminate the ray cast
    fraction: clip the ray to this point
         1.0: don't clip the ray and continue
    '''
    def __init__(self):
        self.continue_without_clipping()

    def set(self, value):
        self.value = value

    def _get_repr_info(self):
        return [('value', self.value)]

    def ignore_fixture(self):
        self.set(-1.0)

    def stop(self):
        self.set(0.0)

    def continue_without_clipping(self):
        self.set(1.0)


cdef class RaycastIterable:
    cdef bool used
    cdef bool casting
    cdef RayCastCallback *cb
    cdef Vec2 point1
    cdef Vec2 point2

    cdef object thread
    cdef object new_fixture_event
    cdef object response_event

    cdef public World world
    cdef public object body
    cdef public Fixture fixture
    cdef public Vec2 point
    cdef public Vec2 normal
    cdef public float32 fraction

    cdef public float32 response

    def __init__(self, World world, point1, point2):
        self.world = world
        self.point1 = Vec2(*point1)
        self.point2 = Vec2(*point2)
        self.used = False
        self.casting = False
        self.cb = new RayCastCallback(self)

    cdef update(self, b2Body *body, b2Fixture *fixture, const b2Vec2 &point,
                const b2Vec2 &normal, float32 fraction):
        self.body = self.world._bodies[pointer_as_key(body)]
        self.fixture = (<Body>self.body).get_fixture(<b2Fixture*>fixture)
        self.point = to_vec2(point)
        self.normal = to_vec2(normal)
        self.fraction = fraction

        self.new_fixture_event.set()
        self.response_event.wait()
        self.response_event.clear()
        return self.response

    def run_raycast(self):
        cdef b2World *world = self.world.world
        world.RayCast(self.cb, to_b2vec2(self.point1), to_b2vec2(self.point2))
        self.casting = False
        self.new_fixture_event.set()

    def __dealloc__(self):
        del self.cb

    def __iter__(self):
        if self.used:
            raise ValueError('RaycastIterable is single-use')

        self.used = True
        self.casting = True

        self.new_fixture_event = threading.Event()
        self.response_event = threading.Event()

        self.thread = threading.Thread(target=self.run_raycast, daemon=True)
        self.thread.start()

        try:
            while self.casting:
                self.new_fixture_event.wait()
                self.new_fixture_event.clear()
                if not self.casting:
                    break

                response_wrapper = RaycastResponseWrapper()
                yield (RaycastTuple(body=self.body, fixture=self.fixture,
                                    point=self.point, normal=self.normal,
                                    fraction=self.fraction),
                       response_wrapper)

                self.fixture = None
                self.response = response_wrapper.value

                self.response_event.set()
        finally:
            self.response = 0.0

            self.response_event.set()
            while self.casting:
                self.new_fixture_event.wait()

            self.thread.join()
            self.thread = None
            self.new_fixture_event = None
            self.response_event = None
