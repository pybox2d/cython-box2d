from defn.math cimport *
from defn.body cimport b2Body


cdef extern from "b2Joint.h":
    cdef enum b2JointType:
        e_unknownJoint
        e_revoluteJoint
        e_prismaticJoint
        e_distanceJoint
        e_pulleyJoint
        e_mouseJoint
        e_gearJoint
        e_wheelJoint
        e_weldJoint
        e_frictionJoint
        e_ropeJoint
        e_motorJoint

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
        b2Joint* GetNext()
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

