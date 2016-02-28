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
    
    @safe_rw_property
    def vertices(self, vertices):
        '''The polygon vertices'''
        cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
        if vertices is None:
            if (polygon.GetVertexCount() > b2_maxPolygonVertices):
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(polygon.m_vertices[vertex])
                    for vertex in range(polygon.GetVertexCount())]

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

    @safe_rw_property
    def normals(self, normals):
        '''The polygon normals'''
        cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
        if normals is None:
            if polygon.GetVertexCount() > b2_maxPolygonVertices:
                raise ValueError('Invalid state '
                                 '(more vertices than supported?)')

            return [to_vec2(polygon.m_normals[vertex])
                    for vertex in range(polygon.GetVertexCount())]

        normals = list(normals)
        if len(normals) > b2_maxPolygonVertices:
            raise ValueError('Exceeded maximum polygon vertices {} '
                             '(max={})'.format(len(normals),
                                               b2_maxPolygonVertices))

        for i, vertex in enumerate(normals):
            polygon.m_normals[i] = to_b2vec2(vertex)

        polygon.m_count = len(normals)
        self.box_settings = None

    @safe_rw_property
    def centroid(self, centroid):
        '''The polygon centroid'''
        cdef b2PolygonShape *polygon = <b2PolygonShape *>self.shape
        if centroid is None:
            return to_vec2(polygon.m_centroid)

        polygon.m_centroid = to_b2vec2(centroid)

    @property
    def valid_polygon(self):
        return (<b2PolygonShape *>self.shape).Validate()
    
    @safe_method
    def set_as_box(self, float hx, float hy, center=None, angle=0.0):
        '''Set the polygon to be a box shape
        
        Parameters
        ----------
        hx : float
            half horizontal width
        hy : float
            half vertical height
        center : Vec2, optional
            local center position, defaults to (0, 0)
        angle : float
            rotation angle
        '''
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
    
    @safe_method
    def compute_mass(self, density):
        if (<b2PolygonShape *>self.shape).m_count < 3:
            raise ValueError('Must have at least 3 vertices to compute mass')

        return super().compute_mass(density)
