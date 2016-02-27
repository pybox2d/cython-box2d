# include "math.pyd"
import math


cdef class Vec2:
    cdef b2Vec2 *thisptr
    def __cinit__(self, x=0.0, y=0.0):
        if isinstance(x, Vec2):
            self.thisptr = new b2Vec2(x.x, x.y)
        else:
            self.thisptr = new b2Vec2(x, y)

    def __dealloc__(self):
        del self.thisptr

    @property
    def is_valid(self):
        return self.thisptr.IsValid()

    property x:
        def __get__(self):
            return self.thisptr.x

        def __set__(self, x):
            self.thisptr.x = x

    property y:
        def __get__(self):
            return self.thisptr.y

        def __set__(self, y):
            self.thisptr.y = y

    def __getitem__(self, idx):
        return (self.x, self.y)[idx]

    def __len__(self):
        return 2

    def __richcmp__(Vec2 self, other, operation):
        otherx, othery = other
        if operation == 2:  # eq
            return (self.x == otherx) and (self.y == othery)
        elif operation == 3:  # ne
            return (self.x != otherx) or (self.y != othery)
        else:
            raise ValueError('Can only compare equality of a Vec2')

    def __add__(Vec2 self, other):
        return Vec2(self.x + other[0], self.y + other[1])

    def __sub__(Vec2 self, other):
        return Vec2(self.x - other[0], self.y - other[1])

    def __div__(Vec2 self, other):
        return Vec2(self.x / other, self.y / other)

    def __mul__(Vec2 self, other):
        return Vec2(self.x * other, self.y * other)

    def __iter__(self):
        return iter((self.x, self.y))

    def __repr__(self):
        return '{0.__class__.__name__}({0.x}, {0.y})'.format(self)


cdef class Rotation:
    cdef float sine
    cdef float cosine

    def __init__(self, sine=0.0, cosine=1.0, angle=None):
        if angle is not None:
            self.angle = angle
        else:
            self.sine = sine
            self.cosine = cosine

    def set_identity(self):
        self.sine = 0.0
        self.cosine = 1.0

    def __iter__(self):
        return iter((self.sine, self.cosine))

    property angle:
        def __get__(self):
            return math.atan2(self.sine, self.cosine)

        def __set__(self, angle):
            self.sine = math.sin(angle)
            self.cosine = math.cos(angle)

    @property
    def x_axis(self):
        return math.atan2(self.cosine, self.sine)

    @property
    def y_axis(self):
        return math.atan2(-self.sine, self.cosine)

    def __repr__(self):
        return '{0.__class__.__name__}(angle={0.angle})'.format(self)


cdef class Transform(Base):
    cdef b2Transform transform;

    def __init__(self, position=None, angle=0.0):
        if position is not None:
            self.position = position

        self.angle = angle

    @staticmethod
    cdef from_b2Transform(b2Transform b2t):
        xf = Transform()
        xf.transform.p = b2t.p
        xf.transform.q.s = b2t.q.s
        xf.transform.q.c = b2t.q.c
        return xf

    property position:
        def __get__(self):
            return to_vec2(self.transform.p)

        def __set__(self, position):
            self.transform.p = to_b2vec2(position)

    property rotation:
        def __get__(self):
            return Rotation(sine=self.transform.q.s,
                            cosine=self.transform.q.c)

        def __set__(self, rotation):
            self.transform.q.s = rotation[0]
            self.transform.q.c = rotation[1]

    property angle:
        def __get__(self):
            return self.transform.q.GetAngle()

        def __set__(self, angle):
            self.transform.q.Set(angle)

    def __mul__(Transform xf, other):
        return to_vec2(b2Mul(xf.transform, to_b2vec2(other)))

    def _get_repr_info(self):
        yield ('position', self.position)
        yield ('angle', self.angle)
