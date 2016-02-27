import pytest
import copy

import pybox2d
from pybox2d import Vec2


def test_vec2_init():
    v = Vec2()
    print(v)
    print(v.x, v.y, v.is_valid, type(v.is_valid))
    assert v.x == 0.0
    assert v.y == 0.0

    v.x = 1
    v.y = 2

    assert v.x == 1.0
    assert v.y == 2.0
    print('v', v)
    copied = Vec2(v)
    print('copy', copy)
    assert copied == v


def test_vec2_eq():
    v = Vec2(1, 1)
    assert (v == (1, 1))
    assert (v != (0, 1))

    with pytest.raises(ValueError):
        v > (0, 1)

    with pytest.raises(ValueError):
        v < (0, 1)

    with pytest.raises(ValueError):
        v <= (0, 1)

    with pytest.raises(ValueError):
        v >= (0, 1)


