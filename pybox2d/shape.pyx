# include "shape.pyd"


cdef class Shape:
    # cppclass inheritance not working so cleanly in Cython - or what am I
    # doing wrong?
    cdef b2Shape *shape
    cdef bool owner

    def __cinit__(self):
        self.shape = NULL
        self.owner = False

    def compute_mass(self, mass_data, density):
        pass

    def compute_aabb(self, aabb, transform, child_index):
        pass

    # def raycast(self, output, input, transform, child_index):
    # def test_point(self, transform, point):

    @property
    def child_count(self):
        return self.shape.GetChildCount()

    @staticmethod
    cdef upcast(const b2Shape *shape):
        # TODO copies for now
        if shape.GetType() == ShapeType_circle:
            sh = CircleShape()
            sh.from_existing(<b2CircleShape*>shape, owner=False)
            return sh
        elif shape.GetType() == ShapeType_polygon:
            return PolygonShape()
        elif shape.GetType() == ShapeType_edge:
            return EdgeShape()
        # elif shape.GetType() == ShapeType_chain:
        #     return ChainShape()
        raise RuntimeError('Unknown shape type')


cdef class CircleShape(Shape):
    def __cinit__(self):
        self.shape = <b2Shape*>new b2CircleShape()
        self.owner = True

    def __dealloc__(self):
        if self.owner:
            del self.shape

    cdef from_existing(self, b2CircleShape *shape, owner=True):
        if self.owner:
            del self.shape

        self.shape = shape
        self.owner = owner

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        (<b2CircleShape *>self.shape).m_radius = radius
        self.center = center

    property radius:
        def __get__(self):
            return (<b2CircleShape *>self.shape).m_radius

        def __set__(self, value):
            (<b2CircleShape *>self.shape).m_radius = value

    property center:
        def __get__(self):
            return Vec2((<b2CircleShape *>self.shape).m_p.x,
                        (<b2CircleShape *>self.shape).m_p.y)

        def __set__(self, center):
            cx, cy = center
            (<b2CircleShape *>self.shape).m_p.x = cx
            (<b2CircleShape *>self.shape).m_p.y = cy

    def __str__(self):
        return ('{0.__class__.__name__}(radius={0.radius}, center={0.center})'
                '' .format(self))

    __repr__ = __str__


cdef class PolygonShape(Shape):
    cdef b2PolygonShape *polygon
    cdef object box_settings

    def __cinit__(self):
        self.polygon = new b2PolygonShape()
        self.polygon.m_count = 0
        self.shape = self.polygon
        self.owner = True

    cdef from_existing(self, b2PolygonShape *shape, owner=True):
        if self.owner:
            del self.shape

        self.shape = shape
        self.owner = owner

    def __init__(self, vertices=None, box=None):
        if vertices is not None:
            self.vertices = vertices

        if box is not None:
            self.set_as_box(*box)

    property vertices:
        def __get__(self):
            if self.polygon.GetVertexCount() > b2_maxPolygonVertices:
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(self.polygon.m_vertices[vertex])
                    for vertex in range(self.polygon.GetVertexCount())]

        def __set__(self, vertices):
            vertices = list(vertices)
            if len(vertices) > b2_maxPolygonVertices:
                raise ValueError('Exceeded maximum polygon vertices {} '
                                 '(max={})'.format(len(vertices),
                                                   b2_maxPolygonVertices))

            for i, vertex in enumerate(vertices):
                self.polygon.m_vertices[i] = to_b2vec2(vertex)

            self.polygon.m_count = len(vertices)
            self.box_settings = None

    property normals:
        def __get__(self):
            if self.polygon.GetVertexCount() > b2_maxPolygonVertices:
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(self.polygon.m_normals[vertex])
                    for vertex in range(self.polygon.GetVertexCount())]

        def __set__(self, normals):
            normals = list(normals)
            if len(normals) > b2_maxPolygonVertices:
                raise ValueError('Exceeded maximum polygon vertices {} '
                                 '(max={})'.format(len(normals),
                                                   b2_maxPolygonVertices))

            for i, vertex in enumerate(normals):
                self.polygon.m_normals[i] = to_b2vec2(vertex)

            self.polygon.m_count = len(normals)
            self.box_settings = None

    property centroid:
        def __get__(self):
            return to_vec2(self.polygon.m_centroid)

        def __set__(self, centroid):
            self.polygon.m_centroid = to_b2vec2(centroid)

    @property
    def valid(self):
        return self.polygon.Validate()

    def set_as_box(self, float hx, float hy, center=None, angle=0.0):
        if center is None:
            center = (0, 0)

        self.polygon.SetAsBox(hx, hy, to_b2vec2(center), angle)
        self.box_settings = (hx, hy, center, angle)

    property box:
        def __get__(self):
            return self.box_settings

        def __set__(self, box_settings):
            self.set_as_box(*box_settings)

    def _get_repr_info(self):
        if self.box_settings is not None:
            yield ('box', self.box_settings)
        else:
            yield ('vertices', self.vertices)

    def compute_mass(self, mass_data, density):
        if self.polygon.m_count < 3:
            raise ValueError('Must have at least 3 vertices to compute mass')

        # self.polygon.ComputeMass(mass_data, density)


cdef class EdgeShape(Shape):
    cdef b2EdgeShape *edge

    def __cinit__(self):
        self.edge = new b2EdgeShape()
        self.shape = self.edge
        self.owner = True
