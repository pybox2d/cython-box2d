from collections import namedtuple

MassData = namedtuple('MassData', 'mass center inertia')


cdef class Shape(Base):
    cdef b2Shape *shape
    cdef bool owner

    def __cinit__(self):
        self.shape = NULL
        self.owner = False

    cdef from_existing(self, b2Shape *shape, owner=True):
        if self.owner:
            del self.shape

        self.shape = shape
        self.owner = owner

    # TODO:
    # def compute_aabb(self, aabb, transform, child_index):
    # def raycast(self, output, input, transform, child_index):
    # def test_point(self, transform, point):
    
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

    @property
    def child_count(self):
        '''The number of child primitives.'''
        return self.shape.GetChildCount()

    @property
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
