cdef class Joint(Base):
    cdef b2Joint *joint
    cdef readonly Body body_a
    cdef readonly Body body_b

    def __cinit__(self):
        self.joint = NULL

    cdef invalidate(self):
        self.joint = NULL

    def __hash__(self):
        if self.joint == NULL:
            raise RuntimeError('Underlying object was destroyed')
        return pointer_as_key(self.joint)

    @property
    def valid(self):
        return (self.joint != NULL)

    @safe_property
    def bodies(self):
        '''The bodies attached to this joint'''
        return (self.body_a, self.body_b)

    @safe_property
    def anchors(self):
        '''Get the anchor point on bodyB in world coordinates.'''
        return (to_vec2(self.joint.GetAnchorA()),
                to_vec2(self.joint.GetAnchorB()))

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

    cpdef _get_repr_info(self):
        return [('valid', self.valid),
                ('bodies', self.bodies),
                ('anchors', self.anchors),
                ('active', self.active),
                ('collide_connected', self.collide_connected),
                ]

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
        '''The reference angle, in radians'''
        return (<b2RevoluteJoint *>self.joint).GetReferenceAngle()

    @safe_property
    def angle(self):
        '''The current joint angle, in radians.'''
        return (<b2RevoluteJoint *>self.joint).GetJointAngle()

    @safe_property
    def speed(self):
        '''The current joint angle speed in radians per second.'''
        return (<b2RevoluteJoint *>self.joint).GetJointSpeed()

    @safe_rw_property
    def limit_enabled(self, enable):
        '''Is the joint limit enabled?'''
        if enable is None:
            return (<b2RevoluteJoint *>self.joint).IsLimitEnabled()

        (<b2RevoluteJoint *>self.joint).EnableLimit(enable)

    @safe_rw_property
    def limits(self, limits):
        '''The (lower joint limit, upper joint limit) in radians.'''
        if limits is None:
            lower_limit = (<b2RevoluteJoint *>self.joint).GetLowerLimit()
            upper_limit = (<b2RevoluteJoint *>self.joint).GetUpperLimit()
            return lower_limit, upper_limit

        lower_limit, upper_limit = limits
        (<b2RevoluteJoint *>self.joint).SetLimits(lower_limit, upper_limit)

    @safe_rw_property
    def lower_limit(self, limit):
        '''The lower joint limit in radians.'''
        if limit is None:
            return (<b2RevoluteJoint *>self.joint).GetLowerLimit()

        (<b2RevoluteJoint *>self.joint).SetLimits(limit, self.upper_limit)

    @safe_rw_property
    def upper_limit(self, limit):
        '''The upper joint limit in radians.'''
        if limit is None:
            return (<b2RevoluteJoint *>self.joint).GetUpperLimit()

        (<b2RevoluteJoint *>self.joint).SetLimits(self.lower_limit, limit)

    @safe_rw_property
    def motor_enabled(self, enable):
        '''Is the joint motor enabled?'''
        if enable is None:
            return (<b2RevoluteJoint *>self.joint).IsMotorEnabled()

        (<b2RevoluteJoint *>self.joint).EnableMotor(enable)

    @safe_rw_property
    def motor_speed(self, speed):
        '''The motor speed in radians per second.'''
        if speed is None:
            return (<b2RevoluteJoint *>self.joint).GetMotorSpeed()

        (<b2RevoluteJoint *>self.joint).SetMotorSpeed(speed)

    @safe_rw_property
    def max_motor_torque(self, max_motor_torque):
        '''The maximum motor torque, usually in N-m.'''
        if max_motor_torque is None:
            return (<b2RevoluteJoint *>self.joint).GetMaxMotorTorque()

        (<b2RevoluteJoint *>self.joint).SetMaxMotorTorque(max_motor_torque)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([('reference_angle', self.reference_angle),
                          ('angle', self.angle),
                          ('speed', self.speed),
                          ('limit_enabled', self.limit_enabled),
                          ('limits', self.limits),
                          ('motor_enabled', self.motor_enabled),
                          ('motor_speed', self.motor_speed),
                          ('max_motor_torque', self.max_motor_torque),
                          ])
        return repr_info


cdef class DistanceJoint(Joint):
    @safe_rw_property
    def damping_ratio(self, damping_ratio):
        '''Set/get damping ratio.'''
        if damping_ratio is None:
            return (<b2DistanceJoint *>self.joint).GetDampingRatio()

        (<b2DistanceJoint *>self.joint).SetDampingRatio(damping_ratio)

    @safe_rw_property
    def frequency(self, frequency):
        '''Set/get frequency in Hz.'''
        if frequency is None:
            return (<b2DistanceJoint *>self.joint).GetFrequency()

        (<b2DistanceJoint *>self.joint).SetFrequency(frequency)

    @safe_rw_property
    def length(self, length):
        '''Set/get the natural length. Manipulating the length can lead to non-
        physical behavior when the frequency is zero.'''
        if length is None:
            return (<b2DistanceJoint *>self.joint).GetLength()

        (<b2DistanceJoint *>self.joint).SetLength(length)

    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2DistanceJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2DistanceJoint *>self.joint).GetLocalAnchorB()))

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([('damping_ratio', self.damping_ratio),
                          ('frequency', self.frequency),
                          ('length', self.length),
                          ('local_anchors', self.local_anchors),
                          ])
        return repr_info


cdef class FrictionJoint(Joint):
    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2FrictionJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2FrictionJoint *>self.joint).GetLocalAnchorB()))

    @safe_rw_property
    def max_force(self, max_force):
        '''Get the maximum friction force in N.'''
        if max_force is None:
            return (<b2FrictionJoint *>self.joint).GetMaxForce()

        (<b2FrictionJoint *>self.joint).SetMaxForce(max_force)

    @safe_rw_property
    def max_torque(self, max_torque):
        '''Get the maximum friction torque in N*m.'''
        if max_torque is None:
            return (<b2FrictionJoint *>self.joint).GetMaxTorque()

        (<b2FrictionJoint *>self.joint).SetMaxTorque(max_torque)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([('local_anchors', self.local_anchors),
                          ('max_force', self.max_force),
                          ('max_torque', self.max_torque),
                          ])
        return repr_info


cdef class CompositeJoint(Joint):
    cdef readonly joint_a
    cdef readonly joint_b


cdef class GearJoint(CompositeJoint):
    @safe_rw_property
    def ratio(self, ratio):
        '''Set/Get the gear ratio.'''
        if ratio is None:
            return (<b2GearJoint *>self.joint).GetRatio()

        (<b2GearJoint *>self.joint).SetRatio(ratio)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.append(('ratio', self.ratio))
        return repr_info


cdef class MotorJoint(Joint):
    @safe_rw_property
    def angular_offset(self, angular_offset):
        '''Set/get the target angular offset, in radians.'''
        if angular_offset is None:
            return (<b2MotorJoint *>self.joint).GetAngularOffset()

        (<b2MotorJoint *>self.joint).SetAngularOffset(angular_offset)

    @safe_rw_property
    def correction_factor(self, correction_factor):
        '''Get the position correction factor in the range [0,1].'''
        if correction_factor is None:
            return (<b2MotorJoint *>self.joint).GetCorrectionFactor()

        (<b2MotorJoint *>self.joint).SetCorrectionFactor(correction_factor)

    @safe_rw_property
    def linear_offset(self, linear_offset):
        '''Set/get the target linear offset, in frame A, in meters.'''
        if linear_offset is None:
            return to_vec2((<b2MotorJoint *>self.joint).GetLinearOffset())

        (<b2MotorJoint *>self.joint).SetLinearOffset(to_b2vec2(linear_offset))

    @safe_rw_property
    def max_force(self, max_force):
        '''Get the maximum friction force in N.'''
        if max_force is None:
            return (<b2MotorJoint *>self.joint).GetMaxForce()

        (<b2MotorJoint *>self.joint).SetMaxForce(max_force)

    @safe_rw_property
    def max_torque(self, max_torque):
        '''Get the maximum friction torque in N*m.'''
        if max_torque is None:
            return (<b2MotorJoint *>self.joint).GetMaxTorque()

        (<b2MotorJoint *>self.joint).SetMaxTorque(max_torque)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([('angular_offset', self.angular_offset),
                          ('correction_factor', self.correction_factor),
                          ('linear_offset', self.linear_offset),
                          ('max_force', self.max_force),
                          ('max_torque', self.max_torque),
                          ])
        return repr_info


cdef class MouseJoint(Joint):
    @safe_rw_property
    def damping_ratio(self, damping_ratio):
        '''Set/get the damping ratio (dimensionless).'''
        if damping_ratio is None:
            return (<b2MouseJoint *>self.joint).GetDampingRatio()

        (<b2MouseJoint *>self.joint).SetDampingRatio(damping_ratio)

    @safe_rw_property
    def frequency(self, frequency):
        '''Set/get the frequency in Hertz.'''
        if frequency is None:
            return (<b2MouseJoint *>self.joint).GetFrequency()

        (<b2MouseJoint *>self.joint).SetFrequency(frequency)

    @safe_rw_property
    def max_force(self, max_force):
        '''Set/get the maximum force in Newtons.'''
        if max_force is None:
            return (<b2MouseJoint *>self.joint).GetMaxForce()

        (<b2MouseJoint *>self.joint).SetMaxForce(max_force)

    @safe_rw_property
    def target(self, target):
        '''Use this to update the target point.'''
        if target is None:
            return to_vec2((<b2MouseJoint *>self.joint).GetTarget())

        (<b2MouseJoint *>self.joint).SetTarget(to_b2vec2(target))

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)

        repr_info.extend([('damping_ratio', self.damping_ratio),
                          ('frequency', self.frequency),
                          ('max_force', self.max_force),
                          ('target', self.target),
                          ])
        return repr_info


cdef class PrismaticJoint(Joint):
    @safe_property
    def joint_speed(self):
        '''Get the current joint translation speed, usually in meters per
        second.'''
        return (<b2PrismaticJoint *>self.joint).GetJointSpeed()

    @safe_property
    def joint_translation(self):
        '''Get the current joint translation, usually in meters.'''
        return (<b2PrismaticJoint *>self.joint).GetJointTranslation()

    @safe_property
    def limit_enabled(self):
        '''Is the joint limit enabled?'''
        return (<b2PrismaticJoint *>self.joint).IsLimitEnabled()

    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2PrismaticJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2PrismaticJoint *>self.joint).GetLocalAnchorB()))

    @safe_property
    def local_axis_a(self):
        '''The local joint axis relative to bodyA.'''
        return to_vec2((<b2PrismaticJoint *>self.joint).GetLocalAxisA())

    @safe_rw_property
    def max_motor_force(self, max_motor_force):
        '''Set the maximum motor force, usually in N.'''
        if max_motor_force is None:
            return (<b2PrismaticJoint *>self.joint).GetMaxMotorForce()

        (<b2PrismaticJoint *>self.joint).SetMaxMotorForce(max_motor_force)

    @safe_property
    def motor_enabled(self):
        '''Is the joint motor enabled?'''
        return (<b2PrismaticJoint *>self.joint).IsMotorEnabled()

    @safe_method
    def get_motor_force(self, inv_dt):
        '''Get the current motor force given the inverse time step, usually in
        N.'''
        return (<b2PrismaticJoint *>self.joint).GetMotorForce(inv_dt)

    @safe_rw_property
    def motor_speed(self, motor_speed):
        '''Get the motor speed, usually in meters per second.'''
        if motor_speed is None:
            return (<b2PrismaticJoint *>self.joint).GetMotorSpeed()

        (<b2PrismaticJoint *>self.joint).SetMotorSpeed(motor_speed)

    @safe_property
    def reference_angle(self):
        '''Get the reference angle.'''
        return (<b2PrismaticJoint *>self.joint).GetReferenceAngle()

    @safe_rw_property
    def limits(self, limits):
        '''The (lower joint limit, upper joint limit) in meters.'''
        if limits is None:
            lower_limit = (<b2PrismaticJoint *>self.joint).GetLowerLimit()
            upper_limit = (<b2PrismaticJoint *>self.joint).GetUpperLimit()
            return lower_limit, upper_limit

        lower_limit, upper_limit = limits
        (<b2PrismaticJoint *>self.joint).SetLimits(lower_limit, upper_limit)

    @safe_rw_property
    def lower_limit(self, limit):
        '''The lower joint limit in meters.'''
        if limit is None:
            return (<b2PrismaticJoint *>self.joint).GetLowerLimit()

        (<b2PrismaticJoint *>self.joint).SetLimits(limit, self.upper_limit)

    @safe_rw_property
    def upper_limit(self, limit):
        '''The upper joint limit in meters.'''
        if limit is None:
            return (<b2PrismaticJoint *>self.joint).GetUpperLimit()

        (<b2PrismaticJoint *>self.joint).SetLimits(self.lower_limit, limit)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([
            ('joint_speed', self.joint_speed),
            ('joint_translation', self.joint_translation),
            ('limit_enabled', self.limit_enabled),
            ('local_anchors', self.local_anchors),
            ('local_axis_a', self.local_axis_a),
            ('max_motor_force', self.max_motor_force),
            ('motor_enabled', self.motor_enabled),
            ('motor_speed', self.motor_speed),
            ('reference_angle', self.reference_angle),
            ('limits', self.limits),
            ])

        return repr_info


cdef class PulleyJoint(Joint):
    @safe_property
    def current_lengths(self):
        '''Get the current length of the segment attached to bodyA, bodyB.'''
        return ((<b2PulleyJoint *>self.joint).GetCurrentLengthA(),
                (<b2PulleyJoint *>self.joint).GetCurrentLengthB())

    @safe_property
    def ground_anchors(self):
        '''Get the ground anchors.'''
        return (to_vec2((<b2PulleyJoint *>self.joint).GetGroundAnchorA()),
                to_vec2((<b2PulleyJoint *>self.joint).GetGroundAnchorB()))

    @safe_property
    def lengths(self):
        '''Get the current length of the segment attached to the bodies.'''
        return ((<b2PulleyJoint *>self.joint).GetLengthA(),
                (<b2PulleyJoint *>self.joint).GetLengthB())

    @safe_property
    def ratio(self):
        '''Get the pulley ratio.'''
        return (<b2PulleyJoint *>self.joint).GetRatio()

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)

        repr_info.extend([
            ('current_lengths', self.current_lengths),
            ('ground_anchors', self.ground_anchors),
            ('lengths', self.lengths),
            ('ratio', self.ratio),
            ])
        return repr_info


cdef class RopeJoint(Joint):
    @safe_property
    def limit_state(self):
        ''''''
        return (<b2RopeJoint *>self.joint).GetLimitState()

    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2RopeJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2RopeJoint *>self.joint).GetLocalAnchorB()))

    @safe_rw_property
    def max_length(self, max_length):
        '''Set/Get the maximum length of the rope.'''
        if max_length is None:
            return (<b2RopeJoint *>self.joint).GetMaxLength()

        (<b2RopeJoint *>self.joint).SetMaxLength(max_length)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)

        repr_info.extend([('limit_state', self.limit_state),
                          ('local_anchors', self.local_anchors),
                          ('max_length', self.max_length),
                          ])
        return repr_info


cdef class WeldJoint(Joint):
    @safe_rw_property
    def damping_ratio(self, damping_ratio):
        '''Set/get damping ratio.'''
        if damping_ratio is None:
            return (<b2WeldJoint *>self.joint).GetDampingRatio()

        (<b2WeldJoint *>self.joint).SetDampingRatio(damping_ratio)

    @safe_rw_property
    def frequency(self, frequency):
        '''Set/get frequency in Hz.'''
        if frequency is None:
            return (<b2WeldJoint *>self.joint).GetFrequency()

        (<b2WeldJoint *>self.joint).SetFrequency(frequency)

    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2WeldJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2WeldJoint *>self.joint).GetLocalAnchorB()))

    @safe_property
    def reference_angle(self):
        '''Get the reference angle.'''
        return (<b2WeldJoint *>self.joint).GetReferenceAngle()

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)

        repr_info.extend([('damping_ratio', self.damping_ratio),
                          ('frequency', self.frequency),
                          ('local_anchors', self.local_anchors),
                          ('reference_angle', self.reference_angle),
                          ])
        return repr_info


cdef class WheelJoint(Joint):
    @safe_property
    def joint_speed(self):
        '''Get the current joint translation speed, usually in meters per
        second.'''
        return (<b2WheelJoint *>self.joint).GetJointSpeed()

    @safe_property
    def joint_translation(self):
        '''Get the current joint translation, usually in meters.'''
        return (<b2WheelJoint *>self.joint).GetJointTranslation()

    @safe_property
    def local_anchors(self):
        '''The local anchor points relative to body_a, body_b's origins.'''
        return (to_vec2((<b2WheelJoint *>self.joint).GetLocalAnchorA()),
                to_vec2((<b2WheelJoint *>self.joint).GetLocalAnchorB()))

    @safe_property
    def local_axis_a(self):
        '''The local joint axis relative to bodyA.'''
        return to_vec2((<b2WheelJoint *>self.joint).GetLocalAxisA())

    @safe_rw_property
    def max_motor_torque(self, max_motor_torque):
        '''Set/Get the maximum motor force, usually in N-m.'''
        if max_motor_torque is None:
            return (<b2WheelJoint *>self.joint).GetMaxMotorTorque()

        (<b2WheelJoint *>self.joint).SetMaxMotorTorque(max_motor_torque)

    @safe_property
    def motor_enabled(self):
        '''Is the joint motor enabled?'''
        return (<b2WheelJoint *>self.joint).IsMotorEnabled()

    @safe_rw_property
    def motor_speed(self, motor_speed):
        '''Get the motor speed, usually in radians per second.'''
        if motor_speed is None:
            return (<b2WheelJoint *>self.joint).GetMotorSpeed()

        (<b2WheelJoint *>self.joint).SetMotorSpeed(motor_speed)

    @safe_method
    def get_motor_torque(self, inv_dt):
        '''Get the current motor torque given the inverse time step, usually in
        N-m.'''
        return (<b2WheelJoint *>self.joint).GetMotorTorque(inv_dt)

    @safe_rw_property
    def spring_damping_ratio(self, spring_damping_ratio):
        '''Set/Get the spring damping ratio'''
        if spring_damping_ratio is None:
            return (<b2WheelJoint *>self.joint).GetSpringDampingRatio()

        (<b2WheelJoint *>self.joint).SetSpringDampingRatio(spring_damping_ratio)

    @safe_rw_property
    def spring_frequency_hz(self, spring_frequency_hz):
        '''Set/Get the spring frequency in hertz. Setting the frequency to zero
        disables the spring.'''
        if spring_frequency_hz is None:
            return (<b2WheelJoint *>self.joint).GetSpringFrequencyHz()

        (<b2WheelJoint *>self.joint).SetSpringFrequencyHz(spring_frequency_hz)

    cpdef _get_repr_info(self):
        repr_info = Joint._get_repr_info(self)
        repr_info.extend([
            ('joint_speed', self.joint_speed),
            ('joint_translation', self.joint_translation),
            ('local_anchors', self.local_anchors),
            ('local_axis_a', self.local_axis_a),
            ('max_motor_torque', self.max_motor_torque),
            ('motor_enabled', self.motor_enabled),
            ('motor_speed', self.motor_speed),
            ('spring_damping_ratio', self.spring_damping_ratio),
            ('spring_frequency_hz', self.spring_frequency_hz),
            ])
        return repr_info
