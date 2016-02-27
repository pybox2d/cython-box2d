cdef class Joint(Base):
    cdef b2Joint *joint
    cdef readonly Body body_a
    cdef readonly Body body_b

    def __cinit__(self):
        self.joint = NULL

    cdef invalidate(self):
        self.joint = NULL

    @property
    def valid(self):
        return (self.joint != NULL)

    @safe_property
    def bodies(self):
        '''The bodies attached to this joint'''
        return (self.body_a, self.body_b)

    @property
    def anchors(self):
        return (self.anchor_a, self.anchor_b)

    @safe_property
    def anchor_a(self):
        '''Get the anchor point on bodyA in world coordinates.'''
        return to_vec2(self.joint.GetAnchorA())

    @safe_property
    def anchor_b(self):
        '''Get the anchor point on bodyB in world coordinates.'''
        return to_vec2(self.joint.GetAnchorB())

    @safe_method
    def get_reaction_force(self, inv_dt):
        '''Get the reaction force on bodyB at the joint anchor in Newtons.'''
        return to_vec2(self.joint.GetReactionForce(inv_dt))

    @safe_method
    def get_reaction_torque(self, inv_dt):
        '''Get the reaction torque on bodyB in N*m.'''
        return self.joint.GetReactionTorque(inv_dt)

    @safe_property
    def active(self):
        '''Is either body active?'''
        return self.joint.IsActive()

    @safe_property
    def collide_connected(self):
        '''Collide connected bodies?'''
        return self.joint.GetCollideConnected()

    @safe_method
    def shift_origin(self, new_origin not None):
        '''Shift the origin for any points stored in world coordinates.'''
        self.joint.ShiftOrigin(to_b2vec2(new_origin))

    def _get_repr_info(self):
        yield ('valid', self.valid)
        yield ('body_a', self.body_a)
        yield ('anchor_a', self.anchor_a)
        yield ('body_b', self.body_b)
        yield ('anchor_b', self.anchor_b)
        yield ('active', self.active)
        yield ('collide_connected', self.collide_connected)

    @staticmethod
    cdef upcast(b2Joint *joint):
        cdef b2JointType joint_type = joint.GetType()
        if joint_type == JointType_revolute:
            inst = RevoluteJoint()
        elif joint_type == JointType_prismatic:
            inst = PrismaticJoint()
        elif joint_type == JointType_distance:
            inst = DistanceJoint()
        elif joint_type == JointType_pulley:
            inst = PulleyJoint()
        elif joint_type == JointType_mouse:
            inst = MouseJoint()
        elif joint_type == JointType_gear:
            inst = GearJoint()
        elif joint_type == JointType_wheel:
            inst = WheelJoint()
        elif joint_type == JointType_weld:
            inst = WeldJoint()
        elif joint_type == JointType_friction:
            inst = FrictionJoint()
        elif joint_type == JointType_rope:
            inst = RopeJoint()
        elif joint_type == JointType_motor:
            inst = MotorJoint()
        else:
            inst = Joint()

        (<Joint>inst).joint = joint
        return inst


cdef class RevoluteJoint(Joint):
    '''
    A revolute joint constrains two bodies to share a common point while they
    are free to rotate about the point. The relative rotation about the shared
    point is the joint angle. You can limit the relative rotation with a joint
    limit that specifies a lower and upper angle. You can use a motor to drive
    the relative rotation about the shared point. A maximum motor torque is
    provided so that infinite forces are not generated.
    '''

    @safe_property
    def reference_angle(self):
        return (<b2RevoluteJoint *>self.joint).GetReferenceAngle()

    @safe_property
    def joint_angle(self):
        return (<b2RevoluteJoint *>self.joint).GetJointAngle()

    @safe_property
    def joint_speed(self):
        return (<b2RevoluteJoint *>self.joint).GetJointSpeed()


cdef class PrismaticJoint(Joint):
    pass

cdef class DistanceJoint(Joint):
    pass

cdef class PulleyJoint(Joint):
    pass

cdef class MouseJoint(Joint):
    pass

cdef class GearJoint(Joint):
    pass

cdef class WheelJoint(Joint):
    pass

cdef class WeldJoint(Joint):
    pass

cdef class FrictionJoint(Joint):
    pass

cdef class RopeJoint(Joint):
    pass

cdef class MotorJoint(Joint):
    pass
