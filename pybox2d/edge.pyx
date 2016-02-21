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

    property main_vertices:
        def __get__(self):
            cdef b2EdgeShape *edge = <b2EdgeShape *>self.shape

            return [to_vec2(edge.m_vertex1), to_vec2(edge.m_vertex2)]

        def __set__(self, vertices):
            vertices = list(vertices)
            if len(vertices) != 2:
                raise ValueError('Unknown number of vertices (expected 2)')

            cdef b2EdgeShape *edge = <b2EdgeShape *>self.shape
            v1, v2 = vertices
            edge.m_vertex1 = to_b2vec2(v1)
            edge.m_vertex2 = to_b2vec2(v2)

            edge.m_hasVertex0 = False
            edge.m_hasVertex3 = False

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
