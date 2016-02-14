include "math.pyd"

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

    def __add__(self, other):
        return Vec2(self.x + other[0], self.y + other[1])

    def __sub__(self, other):
        return Vec2(self.x - other[0], self.y - other[1])

    def __div__(self, other):
        return Vec2(self.x / other, self.y / other)

    def __mul__(self, other):
        return Vec2(self.x * other, self.y * other)

    def __iter__(self):
        return (self.x, self.y)

    def __str__(self):
        return '{0.__class__.__name__}({0.x}, {0.y})'.format(self)

    __repr__ = __str__
