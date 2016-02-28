from defn.math cimport *
from defn.body cimport b2Body


cdef extern from "b2Joint.h":
    cdef enum b2JointType:
        JointType_unknown "e_unknownJoint"
        JointType_revolute "e_revoluteJoint"
        JointType_prismatic "e_prismaticJoint"
        JointType_distance "e_distanceJoint"
        JointType_pulley "e_pulleyJoint"
        JointType_mouse "e_mouseJoint"
        JointType_gear "e_gearJoint"
        JointType_wheel "e_wheelJoint"
        JointType_weld "e_weldJoint"
        JointType_friction "e_frictionJoint"
        JointType_rope "e_ropeJoint"
        JointType_motor "e_motorJoint"

    cdef enum b2LimitState:
        e_inactiveLimit
        e_atLowerLimit
        e_atUpperLimit
        e_equalLimits

    cdef struct b2Jacobian:
        b2Vec2 linear
        float32 angularA
        float32 angularB

    cdef cppclass b2JointDef:
        b2JointType type
        void* userData
        b2Body* bodyA
        b2Body* bodyB
        bool collideConnected

    cdef cppclass b2Joint:
        b2JointType GetType() const
        b2Body* GetBodyA()
        b2Body* GetBodyB()
        b2Vec2 GetAnchorA() const
        b2Vec2 GetAnchorB() const
        b2Vec2 GetReactionForce(float32 inv_dt) const
        float32 GetReactionTorque(float32 inv_dt) const
        # b2Joint* GetNext()
        const b2Joint* GetNext() const
        void* GetUserData() const
        void SetUserData(void* data)
        bool IsActive() const
        bool GetCollideConnected() const
        void Dump()
        void ShiftOrigin(const b2Vec2& newOrigin)


cdef extern from "b2RevoluteJoint.h":
    cdef cppclass b2RevoluteJointDef(b2JointDef):
        void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor)

        b2Vec2 localAnchorA
        b2Vec2 localAnchorB
        float32 referenceAngle
        bool enableLimit
        float32 lowerAngle
        float32 upperAngle
        bool enableMotor
        float32 motorSpeed
        float32 maxMotorTorque

    cdef cppclass b2RevoluteJoint(b2Joint):
        b2Vec2 GetAnchorA() const
        b2Vec2 GetAnchorB() const
        const b2Vec2& GetLocalAnchorA() const
        const b2Vec2& GetLocalAnchorB() const
        float32 GetReferenceAngle() const
        float32 GetJointAngle() const
        float32 GetJointSpeed() const
        bool IsLimitEnabled() const
        void EnableLimit(bool flag)
        float32 GetLowerLimit() const
        float32 GetUpperLimit() const
        void SetLimits(float32 lower, float32 upper)
        bool IsMotorEnabled() const
        void EnableMotor(bool flag)
        void SetMotorSpeed(float32 speed)
        float32 GetMotorSpeed() const
        void SetMaxMotorTorque(float32 torque)
        float32 GetMaxMotorTorque() const
        b2Vec2 GetReactionForce(float32 inv_dt) const
        float32 GetReactionTorque(float32 inv_dt) const
        float32 GetMotorTorque(float32 inv_dt) const
        void Dump()


cdef extern from "b2DistanceJoint.h":
    cdef cppclass b2DistanceJointDef(b2JointDef):
        void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchorA,
                        const b2Vec2& anchorB)
        float32 dampingRatio
        float32 frequencyHz
        float32 length
        b2Vec2 localAnchorA
        b2Vec2 localAnchorB

    cdef cppclass b2DistanceJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetDampingRatio()
        void SetDampingRatio(float32 ratio)
        float32 GetFrequency()
        void SetFrequency(float32 hz)
        float32 GetLength()
        void SetLength(float32 length)
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)


cdef extern from "b2FrictionJoint.h":
    cdef cppclass b2FrictionJointDef(b2JointDef):
        void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor)
        b2Vec2 localAnchorA
        b2Vec2 localAnchorB
        float32 maxForce
        float32 maxTorque

    cdef cppclass b2FrictionJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        float32 GetMaxForce()
        void SetMaxForce(float32 force)
        float32 GetMaxTorque()
        void SetMaxTorque(float32 torque)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)


cdef extern from "b2GearJoint.h":
    cdef cppclass b2GearJointDef(b2JointDef):
        b2Joint* joint1
        b2Joint* joint2
        float32 ratio

    cdef cppclass b2GearJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetRatio()
        void SetRatio(float32 ratio)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)


cdef extern from "b2MotorJoint.h":
    cdef cppclass b2MotorJointDef(b2JointDef):
        void Initialize(b2Body* bodyA, b2Body* bodyB)
        float32 angularOffset
        float32 correctionFactor
        b2Vec2 linearOffset
        float32 maxForce
        float32 maxTorque

    cdef cppclass b2MotorJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetAngularOffset()
        void SetAngularOffset(float32 angularOffset)
        float32 GetCorrectionFactor()
        void SetCorrectionFactor(float32 factor)
        const b2Vec2& GetLinearOffset()
        void SetLinearOffset(const b2Vec2& linearOffset)
        float32 GetMaxForce()
        void SetMaxForce(float32 force)
        float32 GetMaxTorque()
        void SetMaxTorque(float32 torque)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)


cdef extern from "b2MouseJoint.h":
    cdef cppclass b2MouseJointDef(b2JointDef):
        float32 dampingRatio
        float32 frequencyHz
        float32 maxForce
        b2Vec2 target

    cdef cppclass b2MouseJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetDampingRatio()
        void SetDampingRatio(float32 ratio)
        float32 GetFrequency()
        void SetFrequency(float32 hz)
        float32 GetMaxForce()
        void SetMaxForce(float32 force)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)
        const b2Vec2& GetTarget()
        void SetTarget(const b2Vec2& target)


cdef extern from "b2PrismaticJoint.h":
    cdef cppclass b2PrismaticJointDef(b2JointDef):
        void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor,
                        const b2Vec2& axis)

        bool enableLimit
        bool enableMotor
        b2Vec2 localAnchorA
        b2Vec2 localAnchorB
        b2Vec2 localAxisA
        float32 lowerTranslation
        float32 maxMotorForce
        float32 motorSpeed
        float32 referenceAngle
        float32 upperTranslation

    cdef cppclass b2PrismaticJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        bool IsLimitEnabled()
        float32 GetJointSpeed()
        float32 GetJointTranslation()
        void SetLimits(float32 lower, float32 upper)
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        const b2Vec2& GetLocalAxisA()
        float32 GetLowerLimit()
        float32 GetMaxMotorForce()
        void SetMaxMotorForce(float32 force)
        float32 GetMotorForce(float32 inv_dt)
        float32 GetMotorSpeed()
        void SetMotorSpeed(float32 speed)
        bool IsMotorEnabled()
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)
        float32 GetReferenceAngle()
        float32 GetUpperLimit()


cdef extern from "b2PulleyJoint.h":
    cdef cppclass b2PulleyJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetCurrentLengthA()
        float32 GetCurrentLengthB()
        b2Vec2 GetGroundAnchorA()
        b2Vec2 GetGroundAnchorB()
        float32 GetLengthA()
        float32 GetLengthB()
        float32 GetRatio()
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)
        void ShiftOrigin(const b2Vec2& newOrigin)


cdef extern from "b2RopeJoint.h":
    cdef cppclass b2RopeJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        b2LimitState GetLimitState()
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        float32 GetMaxLength()
        void SetMaxLength(float32 length)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)


cdef extern from "b2WeldJoint.h":
    cdef cppclass b2WeldJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetDampingRatio()
        void SetDampingRatio(float32 ratio)
        float32 GetFrequency()
        void SetFrequency(float32 hz)
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)
        float32 GetReferenceAngle()


cdef extern from "b2WheelJoint.h":
    cdef cppclass b2WheelJoint(b2Joint):
        b2Vec2 GetAnchorA()
        b2Vec2 GetAnchorB()
        float32 GetJointSpeed()
        float32 GetJointTranslation()
        const b2Vec2& GetLocalAnchorA()
        const b2Vec2& GetLocalAnchorB()
        const b2Vec2& GetLocalAxisA()
        float32 GetMaxMotorTorque()
        void SetMaxMotorTorque(float32 torque)
        bool IsMotorEnabled()
        float32 GetMotorSpeed()
        void SetMotorSpeed(float32 speed)
        float32 GetMotorTorque(float32 inv_dt)
        b2Vec2 GetReactionForce(float32 inv_dt)
        float32 GetReactionTorque(float32 inv_dt)
        float32 GetSpringDampingRatio()
        void SetSpringDampingRatio(float32 ratio)
        float32 GetSpringFrequencyHz()
        void SetSpringFrequencyHz(float32 hz)
