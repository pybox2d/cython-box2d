# include "shape.pyd"


cdef class Shape(Base):
    # cppclass inheritance not working so cleanly in Cython - or what am I
    # doing wrong?
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

    def compute_mass(self, mass_data, density):
        pass

    def compute_aabb(self, aabb, transform, child_index):
        pass

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


cdef class CircleShape(Shape):
    def __cinit__(self):
        self.shape = <b2Shape*>new b2CircleShape()
        self.owner = True

    def __dealloc__(self):
        if self.owner:
            del self.shape

    def __init__(self, radius=0.0, center=None):
        if center is None:
            center = (0.0, 0.0)

        self.shape.m_radius = radius
        self.center = center

    property radius:
        def __get__(self):
            return self.shape.m_radius

        def __set__(self, value):
            self.shape.m_radius = value

    property center:
        def __get__(self):
            return Vec2((<b2CircleShape *>self.shape).m_p.x,
                        (<b2CircleShape *>self.shape).m_p.y)

        def __set__(self, center):
            cx, cy = center
            (<b2CircleShape *>self.shape).m_p.x = cx
            (<b2CircleShape *>self.shape).m_p.y = cy

    def _get_repr_info(self):
        yield ('radius', self.radius)
        yield ('center', self.center)


cdef class PolygonShape(Shape):
    cdef b2PolygonShape *polygon
    cdef object box_settings

    def __cinit__(self):
        cdef b2PolygonShape *polygon = new b2PolygonShape()
        polygon.m_count = 0

        self.shape = <b2Shape*>polygon
        self.owner = True

    def __init__(self, vertices=None, box=None):
        if vertices is not None and box is not None:
            raise ValueError('Cannot specify both box and vertices')

        if vertices is not None:
            self.vertices = vertices

        if box is not None:
            self.set_as_box(*box)

    property vertices:
        def __get__(self):
            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            if (polygon.GetVertexCount() > b2_maxPolygonVertices):
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(polygon.m_vertices[vertex])
                    for vertex in range(polygon.GetVertexCount())]

        def __set__(self, vertices):
            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            vertices = list(vertices)
            if len(vertices) > b2_maxPolygonVertices:
                raise ValueError('Exceeded maximum polygon vertices {} '
                                 '(max={})'.format(len(vertices),
                                                   b2_maxPolygonVertices))

            cdef b2Vec2 b2v[b2_maxPolygonVertices]

            for i, vertex in enumerate(vertices):
                b2v[i] = to_b2vec2(vertex)

            polygon.Set(b2v, len(vertices))

            # for i, vertex in enumerate(vertices):
            #     polygon.m_vertices[i] = to_b2vec2(vertex)
            # polygon.m_count = len(vertices)

            self.box_settings = None

    property normals:
        def __get__(self):
            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            if polygon.GetVertexCount() > b2_maxPolygonVertices:
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(polygon.m_normals[vertex])
                    for vertex in range(polygon.GetVertexCount())]

        def __set__(self, normals):
            normals = list(normals)
            if len(normals) > b2_maxPolygonVertices:
                raise ValueError('Exceeded maximum polygon vertices {} '
                                 '(max={})'.format(len(normals),
                                                   b2_maxPolygonVertices))

            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            for i, vertex in enumerate(normals):
                polygon.m_normals[i] = to_b2vec2(vertex)

            polygon.m_count = len(normals)
            self.box_settings = None

    property centroid:
        def __get__(self):
            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            return to_vec2(polygon.m_centroid)

        def __set__(self, centroid):
            cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
            polygon.m_centroid = to_b2vec2(centroid)

    @property
    def valid(self):
        return (<b2PolygonShape *>self.shape).Validate()

    def set_as_box(self, float hx, float hy, center=None, angle=0.0):
        if center is None:
            center = (0, 0)

        cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
        polygon.SetAsBox(hx, hy, to_b2vec2(center), angle)
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
        if (<b2PolygonShape *>self.shape).m_count < 3:
            raise ValueError('Must have at least 3 vertices to compute mass')

        # (<b2PolygonShape *>self.shape).ComputeMass(mass_data, density)


cdef class EdgeShape(Shape):

    def __cinit__(self):
        cdef b2EdgeShape *edge = new b2EdgeShape()
        # edge.m_hasVertex0 = False
        # edge.m_hasVertex3 = False

        self.shape = <b2Shape*>edge
        self.owner = True

    def __init__(self, vertices=None):
        if vertices is not None:
            self.vertices = vertices

    property vertices:
        def __get__(self):
            cdef b2EdgeShape *edge = <b2EdgeShape *>self.shape

            vertices = []
            if edge.m_hasVertex0:
                vertices.append(to_vec2(edge.m_vertex0))

            vertices.append(to_vec2(edge.m_vertex1))
            vertices.append(to_vec2(edge.m_vertex2))

            if edge.m_hasVertex3:
                vertices.append(to_vec2(edge.m_vertex3))

            return vertices

        def __set__(self, vertices):
            vertices = list(vertices)
            if len(vertices) not in (2, 4):
                raise ValueError('Unknown number of vertices (expected 2 or 4)'
                                 )

            cdef b2EdgeShape *edge = <b2EdgeShape *>self.shape
            if len(vertices) == 2:
                v1, v2 = vertices
                edge.m_vertex1 = to_b2vec2(v1)
                edge.m_vertex2 = to_b2vec2(v2)

                edge.m_hasVertex0 = False
                edge.m_hasVertex3 = False
            else:
                v0, v1, v2, v3 = vertices
                edge.m_vertex0 = to_b2vec2(v0)
                edge.m_vertex1 = to_b2vec2(v1)
                edge.m_vertex2 = to_b2vec2(v2)
                edge.m_vertex3 = to_b2vec2(v3)

                edge.m_hasVertex0 = True
                edge.m_hasVertex3 = True

    def _get_repr_info(self):
        cdef b2EdgeShape *edge = <b2EdgeShape *>self.shape
        yield ('vertices', self.vertices)
