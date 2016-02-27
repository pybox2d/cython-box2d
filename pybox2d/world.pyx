from defn.world cimport b2World
from defn.joint cimport (b2Joint, b2JointDef, b2RevoluteJointDef)


cdef class World:
    cdef b2World *world
    cdef dict _bodies
    cdef dict _joints
    cdef object _default_body_class

    def __cinit__(self):
        self.world = new b2World(b2Vec2(0.0, 0.0))
        self._bodies = {}
        self._joints = {}

    def __dealloc__(self):
        del self.world

    def __init__(self, gravity=None, default_body_class=None):
        '''The world class manages all physics entities, dynamic simulation,
        and asynchronous queries.
        '''
        if gravity is None:
            gravity = (0, -10)

        if default_body_class is None:
            default_body_class = Body

        self.default_body_class = default_body_class
        self.gravity = gravity

    property default_body_class:
        '''Default Body class.

        create_body() by default returns a pybox2d.Body instance, but you have
        the option of subclassing this for different body types in your
        simulation.
        '''
        def __get__(self):
            return self._default_body_class

        def __set__(self, cls not None):
            if not issubclass(cls, Body):
                raise TypeError('Class must be a subclass of Body')

            self._default_body_class = cls

    property gravity:
        '''The global gravity vector.'''

        def __get__(self):
            return to_vec2(self.world.GetGravity())

        def __set__(self, value):
            self.world.SetGravity(to_b2vec2(value))

    def _iter_bodies(self):
        '''Iterate over Bodies in the world

        Note: users may attempt to delete or add bodies during iteration, so
        the exposed property returns a full list.
        '''
        cdef b2Body *bptr = self.world.GetBodyList()

        while bptr:
            yield self._bodies[pointer_as_key(bptr)]
            bptr = bptr.GetNext()

    property bodies:
        '''List of all bodies in the world'''
        def __get__(self):
            return list(self._iter_bodies())

    def _iter_joints(self):
        '''Iterate over Joints in the world

        Note: users may attempt to delete or add joints during iteration, so
        the exposed property returns a full list.
        '''
        cdef b2Joint *jptr = self.world.GetJointList()

        while jptr:
            yield self._joints[pointer_as_key(jptr)]
            jptr = jptr.GetNext()

    property joints:
        '''List of all joints in the world'''
        def __get__(self):
            return list(self._iter_joints())

    def step(self, float time_step, int vel_iters, int pos_iters):
        '''Take a time step.

        This performs collision detection, integration, and constraint
        solution.

        Parameters
        ----------
        timeStep : float
            the amount of time to simulate, this should not vary.
        velocityIterations : float
            Number of iterations for the velocity constraint solver.
        positionIterations :
            Number of iterations for the position constraint solver.
        '''
        self.world.Step(time_step, vel_iters, pos_iters)

    def create_body_from_def(self, BodyDef body_defn, *, body_class=None):
        '''Create a body from a BodyDef

        Parameters
        ----------
        body_defn : BodyDef
            The body definition
        body_class : subclass of Body, optional
            The wrapped Body will be of this type.

        Returns
        -------
        body : body_class
        '''
        if body_defn is None:
            raise ValueError('Body definition must be specified')

        if body_class is None:
            body_class = self.default_body_class

        if not issubclass(body_class, Body):
            raise TypeError('body_class must be a subclass of Body')

        bptr = self.world.CreateBody(body_defn.thisptr)
        body = body_class()
        (<Body>body).thisptr = bptr

        self._bodies[pointer_as_key(bptr)] = body

        # userdata is never passed along to the Box2D-level objects, so it
        # exists only in pybox2d
        if body_defn.data is not None:
            body.data = body_defn.data

        if body_defn.fixtures is not None:
            for fixture in body_defn.fixtures:
                if isinstance(fixture, dict):
                    fixture_dict = fixture
                    body.create_fixture(**fixture_dict)
                else:
                    body.create_fixture_from_def(fixture)

        return body

    def create_static_body(self, *, body_class=None, **kwargs):
        '''Create a static body with the given keyword arguments

        Parameters
        ----------
        body_class : subclass of Body, optional
            The wrapped Body will be of this type.
        kwargs : dict
            Keyword arguments for the StaticBodyDef

        Returns
        -------
        body : body_class
        '''
        defn = StaticBodyDef(**kwargs)
        return self.create_body_from_def(defn, body_class=body_class)

    def create_dynamic_body(self, *, body_class=None, **kwargs):
        '''Create a dynamic body with the given keyword arguments

        Parameters
        ----------
        body_class : subclass of Body, optional
            The wrapped Body will be of this type.
        kwargs : dict
            Keyword arguments for the DynamicBodyDef

        Returns
        -------
        body : body_class
        '''
        defn = DynamicBodyDef(**kwargs)
        return self.create_body_from_def(defn, body_class=body_class)

    def create_kinematic_body(self, *, body_class=None, **kwargs):
        '''Create a kinematic body with the given keyword arguments

        Parameters
        ----------
        body_class : subclass of Body, optional
            The wrapped Body will be of this type.
        kwargs : dict
            Keyword arguments for the KinematicBodyDef

        Returns
        -------
        body : body_class
        '''
        defn = KinematicBodyDef(**kwargs)
        return self.create_body_from_def(defn, body_class=body_class)

    def destroy_body(self, Body body not None):
        '''Destroy (remove) a body from the world

        All associated shapes and joints will be deleted.

        Parameters
        ----------
        body : Body
            The body to remove
        '''
        for joint in list(body._joints):
            self.destroy_joint(joint)

        cdef b2Body *bptr = body.thisptr
        del self._bodies[pointer_as_key(bptr)]

        body.invalidate()
        del body
        self.world.DestroyBody(bptr)

    def destroy_joint(self, Joint joint not None):
        '''Destroy (remove) a joint from the world

        Parameters
        ----------
        joint : Joint
            The joint to remove
        '''
        cdef b2Joint *jptr = joint.joint
        del self._joints[pointer_as_key(jptr)]

        joint.body_a._joints.remove(joint)
        joint.body_b._joints.remove(joint)

        joint.invalidate()
        self.world.DestroyJoint(jptr)

    def create_revolute_joint(self, bodies, *, anchor=None,
                              reference_angle=None, local_anchors=None,
                              collide_connected=False, angle_limit=None,
                              motor=False, motor_speed=0.0,
                              max_motor_torque=0.0):
        '''Create a revolute joint

        A revolute joint requires an anchor point where the bodies are joined.
        This uses local anchor points so that the initial configuration can
        violate the constraint slightly. You also need to specify the initial
        relative angle for joint limits. This helps when saving and loading a
        game.

        The local anchor points are measured from the body's origin rather than
        the center of mass because:

        1. you might not know where the center of mass will be.
        2. if you add/remove shapes from a body and recompute the mass,
           the joints will be broken.

        Parameters
        ----------
        bodies : (body_a, body_b), Body instances
            The bodies to join together
        anchor : Vec2, optional
            The world anchor point where the bodies will be joined
            If unspecified, reference_angle and local_anchors must be specified.
        reference_angle : float, optional
            Reference angle in radians (body_b angle minus body_a angle)
            Required if 'anchor' is unspecified.
        local_anchors : (anchor_a, anchor_b), Vec2, optional
            Local anchor points relative to (body_a, body_b).
            Required if 'anchor' is unspecified.
        collide_connected : bool, optional
            Allow collision between connected bodies (default: False)
        motor : bool, optional
            Enable the joint motor (default: False)
        motor_speed : float, optional
            Desired motor speed [radians/s]
        max_motor_torque : float, optional
            Maximum motor torque used to achieve the desired motor_speed [N-m]
        angle_limit : (lower_angle, upper_angle), float
            Joint angle limits, in radians. If None, limits will be disabled.
        '''
        body_a, body_b = bodies

        cdef b2RevoluteJointDef defn
        if not isinstance(body_a, Body) or not isinstance(body_b, Body):
            raise TypeError('Bodies must be a subclass of Body')

        cdef b2Body *ba=(<Body>body_a).thisptr
        cdef b2Body *bb=(<Body>body_b).thisptr

        if anchor is None:
            if reference_angle is None or local_anchors is None:
                raise ValueError('If specifying reference angles or local '
                                 'anchors, both are required')

            anchora, anchorb = local_anchors
            defn.bodyA = ba
            defn.bodyB = bb
            defn.localAnchorA = to_b2vec2(anchora)
            defn.localAnchorB = to_b2vec2(anchorb)
            defn.referenceAngle = reference_angle
        else:
            defn.Initialize(ba, bb, to_b2vec2(anchor))

        enable_limit = (angle_limit is not None)
        if angle_limit is None:
            angle_limit = (0, 0)

        defn.collideConnected = collide_connected
        defn.enableMotor = motor
        defn.motorSpeed = motor_speed
        defn.maxMotorTorque = max_motor_torque
        defn.enableLimit = enable_limit
        defn.lowerAngle = angle_limit[0]
        defn.upperAngle = angle_limit[1]
        return self.create_joint_from_defn(defn, body_a, body_b)

    cdef create_joint_from_defn(self, b2JointDef defn, Body body_a,
                                Body body_b):

        cdef b2Joint *jptr = self.world.CreateJoint(&defn)

        joint = Joint.upcast(jptr)
        (<Joint>joint).body_a = body_a
        (<Joint>joint).body_b = body_b

        body_a._joints.append(joint)
        body_b._joints.append(joint)

        self._joints[pointer_as_key(jptr)] = joint
        return joint
