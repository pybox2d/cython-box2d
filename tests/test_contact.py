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


def test_bulk_contact(world):
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
