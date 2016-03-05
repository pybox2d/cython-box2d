def assert_almost_equal(a, b, diff=0.001):
    try:
        len(a)
    except Exception:
        a, b = [a], [b]

    for i, (ai, bi) in enumerate(zip(a, b)):
        assert abs(ai - bi) <= diff, \
            'a[{i}] = {ai} b[{i}] = {bi} (diff={diff})' \
            ''.format(i=i, ai=ai, bi=bi, diff=abs(ai - bi))
