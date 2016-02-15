cdef to_vec2(b2Vec2 vec):
    return Vec2(vec.x, vec.y)


cdef b2Vec2 to_b2vec2(iterable):
    return b2Vec2(iterable[0], iterable[1])
