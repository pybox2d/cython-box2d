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
