cdef class PolygonShape(Shape):
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
