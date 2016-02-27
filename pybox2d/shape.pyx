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
    # def compute_mass(self, mass_data, density):
    # def compute_aabb(self, aabb, transform, child_index):
    # def raycast(self, output, input, transform, child_index):
    # def test_point(self, transform, point):

    @property
    def child_count(self):
        return self.shape.GetChildCount()

    @property
    def radius(self):
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
