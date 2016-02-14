include "math.pyd"

cdef class Vec2:
    cdef b2Vec2 *thisptr
    def __cinit__(self, float32 x=0.0, float32 y=0.0):
        self.thisptr = new b2Vec2(x, y)

    def __dealloc__(self):
        del self.thisptr

    @property
    def is_valid(self):
        return self.thisptr.IsValid()

    @property
    def x(self):
        return self.thisptr.x

    @x.setter
    def x(self, x):
        self.thisptr.x = x

    @property
    def y(self):
        return self.thisptr.y

    @y.setter
    def y(self, y):
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
