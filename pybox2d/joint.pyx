cdef class Joint(Base):
    cdef b2Joint *joint
    cdef Body body_a
    cdef Body body_b

    def __cinit__(self):
        self.joint = NULL

    cdef invalidate(self):
        self.joint = NULL

    @property
    def valid(self):
        return (self.joint != NULL)

    @safe_property
    def bodies(self):
        return (self.body_a, self.body_b)

    @safe_property
    def anchor_a(self):
        return to_vec2(self.joint.GetAnchorA())

    @safe_property
    def anchor_b(self):
        return to_vec2(self.joint.GetAnchorB())

    @safe_method
    def get_reaction_force(self, inv_dt):
        return to_vec2(self.joint.GetReactionForce(inv_dt))

    @safe_method
    def get_reaction_torque(self, inv_dt):
        return self.joint.GetReactionTorque(inv_dt)

    @safe_property
    def active(self):
        return self.joint.IsActive()

    @safe_property
    def collide_connected(self):
        return self.joint.GetCollideConnected()

    @safe_method
    def shift_origin(self, new_origin):
        self.joint.ShiftOrigin(to_b2vec2(new_origin))

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
