from collections import namedtuple

cdef class MassData(Base):
    '''This holds the mass data computed for a shape.

    Attributes
    ----------
    mass : float
        The mass of the shape, usually in kilograms.
    center : Vec2
        The position of the shape's centroid relative to the shape's origin.
    inertia : float
        The rotational inertia of the shape about the local origin.
    '''
    cdef public float32 mass
    cdef public Vec2 center
    cdef public float32 inertia

    def __init__(self, mass=0.0, center=None, inertia=0.0):
        self.mass = mass
        self.center = center
        self.inertia = inertia

    def __iter__(self):
        '''Iterate over the values, as if this were a tuple'''
        return (value for key, value in self._get_repr_info())

    cpdef _get_repr_info(self):
        return [('mass', self.mass),
                ('center', self.center),
                ('inertia', self.inertia)
                ]


cdef class Shape(Base):
    cdef b2Shape *shape
    cdef bool owner

    def __cinit__(self):
        self.shape = NULL
        self.owner = False

    @property
    def valid(self):
        return (self.shape != NULL)

    cdef from_existing(self, b2Shape *shape, owner=True):
        if self.owner:
            del self.shape

        self.shape = shape
        self.owner = owner

    # TODO:
    # def compute_aabb(self, aabb, transform, child_index):
    # def raycast(self, output, input, transform, child_index):
    # def test_point(self, transform, point):

    cdef invalidate(self):
        if not self.owner:
            self.shape = NULL

    @safe_method
    def compute_mass(self, density):
        '''Compute the mass properties of this shape using its dimensions and
        density.

        The inertia tensor is computed about the local origin.

        Parameters
        ----------
        density : float
            The density in kilograms per meter squared.

        Returns
        -------
        massdata : (mass, center, inertia)

        The mass of the shape, usually in kilograms.
        The position of the shape's centroid relative to the shape's origin.
        The rotational inertia of the shape about the local origin.
        '''
        cdef b2MassData md
        self.shape.ComputeMass(&md, density)
        return MassData(mass=md.mass,
                        center=to_vec2(md.center),
                        inertia=md.I)

    @safe_property
    def child_count(self):
        '''The number of child primitives.'''
        return self.shape.GetChildCount()

    @safe_property
    def radius(self):
        '''The shape radius'''
        return self.shape.m_radius

    @staticmethod
    cdef upcast(const b2Shape *shape):
        # TODO copies for now
        if shape.GetType() == ShapeType_circle:
            sh = CircleShape()
        elif shape.GetType() == ShapeType_polygon:
            sh = PolygonShape()
        elif shape.GetType() == ShapeType_edge:
            sh = EdgeShape()
        # elif shape.GetType() == ShapeType_chain:
        #     return ChainShape()
        else:
            raise RuntimeError('Unknown shape type')

        sh.from_existing(<b2Shape*>shape, owner=False)
        return sh
