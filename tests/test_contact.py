import pytest
import pybox2d

from .util import assert_almost_equal
from .fixtures import *


class BodyClass(pybox2d.Body):
    name = None


def test_bulk_contact(world):
    world.monitor_mode = 'bulk'
    world.monitor_contacts(BodyClass, BodyClass)

    body1 = world.create_dynamic_body(position=(0, 2.0), data='1',
                                      body_class=BodyClass)
    body1.create_circle_fixture(radius=1.0, center=(0, 0))

    body2 = world.create_dynamic_body(position=(0, 2.0), data='2',
                                      body_class=BodyClass)
    body2.create_circle_fixture(radius=1.0, center=(0, 0))

    body3 = world.create_dynamic_body(position=(0, 2.0), data='3',
                                      body_class=pybox2d.Body)
    body3.create_circle_fixture(radius=1.0, center=(0, 0))

    world.step(0.01, 10, 10)

    def contact_data(body):
        return [cbody.data
                for contact in body.monitor_contacts
                for cbody in contact.bodies]

    assert set(contact_data(body1)) == {'1', '2'}
    assert set(contact_data(body2)) == {'1', '2'}
    assert len(body3.monitor_contacts) == 0


class BodyWithContactCallbacks(pybox2d.Body):
    name = None
    my_contacts = []

    def contact_begin(self, contact):
        self.my_contacts.append(contact)

    def contact_end(self, contact):
        self.my_contacts.append(contact)


def test_contact_callbacks_1(world):
    world.monitor_mode = 'callbacks'
    world.monitor_contacts(BodyWithContactCallbacks, BodyWithContactCallbacks)

    body1 = world.create_dynamic_body(position=(0, 2.0), data='1',
                                      body_class=BodyWithContactCallbacks)
    body1.create_circle_fixture(radius=1.0, center=(0, 0))

    body2 = world.create_dynamic_body(position=(0, 2.0), data='2',
                                      body_class=BodyWithContactCallbacks)
    body2.create_circle_fixture(radius=1.0, center=(0, 0))

    body3 = world.create_dynamic_body(position=(0, 2.0), data='3',
                                      body_class=pybox2d.Body)
    body3.create_circle_fixture(radius=1.0, center=(0, 0))

    world.step(0.01, 10, 10)

    def contact_data(body):
        return [cbody.data
                for contact in body.my_contacts
                for cbody in contact.bodies]

    assert set(contact_data(body1)) == {'1', '2'}
    assert set(contact_data(body2)) == {'1', '2'}
    assert len(body3.monitor_contacts) == 0

    c0 = body1.my_contacts[0]
    print(c0)


class BodyWithContactSolveCallbacks(pybox2d.Body):
    name = None
    my_contacts = []
    pre_solve_contacts = []
    post_solve_contacts = []

    def contact_begin(self, contact):
        self.my_contacts.append(contact)

    def contact_end(self, contact):
        self.my_contacts.append(contact)

    def contact_pre_solve(self, contact, manifold):
        # note old manifold is not stored, TODO unsafe
        self.pre_solve_contacts.append(contact)

    def contact_post_solve(self, contact, impulse):
        self.post_solve_contacts.append((contact, impulse))


def test_contact_callbacks_2(world):
    world.monitor_mode = 'full_callbacks'
    world.monitor_contacts(BodyWithContactSolveCallbacks,
                           BodyWithContactSolveCallbacks)

    body1 = world.create_dynamic_body(position=(0, 2.0), data='1',
                                      body_class=BodyWithContactSolveCallbacks)
    body1.create_circle_fixture(radius=1.0, center=(0, 0))

    body2 = world.create_dynamic_body(position=(0, 2.0), data='2',
                                      body_class=BodyWithContactSolveCallbacks)
    body2.create_circle_fixture(radius=1.0, center=(0, 0))

    body3 = world.create_dynamic_body(position=(0, 2.0), data='3',
                                      body_class=pybox2d.Body)
    body3.create_circle_fixture(radius=1.0, center=(0, 0))

    world.step(0.01, 10, 10)

    def contact_data(body):
        return [cbody.data
                for contact in body.my_contacts
                for cbody in contact.bodies]

    assert set(contact_data(body1)) == {'1', '2'}
    assert set(contact_data(body2)) == {'1', '2'}
    assert len(body3.monitor_contacts) == 0

    c0 = body1.my_contacts[0]

    assert len(body1.pre_solve_contacts) > 0
    assert len(body2.pre_solve_contacts) > 0
    print('body1', body1.pre_solve_contacts)
    print('-')
    print('body2', body2.pre_solve_contacts)
    print('-')
    print('body1', body1.post_solve_contacts)
    print('-')
    print('body2', body2.post_solve_contacts)
    print('-')
