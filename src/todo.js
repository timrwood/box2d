





	function b2TimeStep() {
		b2TimeStep.b2TimeStep.apply(this, arguments);
	};
	Box2D.Dynamics.b2TimeStep = b2TimeStep;

	function b2World() {
		b2World.b2World.apply(this, arguments);
		if (this.constructor === b2World) this.b2World.apply(this, arguments);
	};
	Box2D.Dynamics.b2World = b2World;


/* Contacts */


	function b2CircleContact() {
		b2CircleContact.b2CircleContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact;

	function b2Contact() {
		b2Contact.b2Contact.apply(this, arguments);
		if (this.constructor === b2Contact) this.b2Contact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2Contact = b2Contact;

	function b2ContactConstraint() {
		b2ContactConstraint.b2ContactConstraint.apply(this, arguments);
		if (this.constructor === b2ContactConstraint) this.b2ContactConstraint.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactConstraint = b2ContactConstraint;

	function b2ContactConstraintPoint() {
		b2ContactConstraintPoint.b2ContactConstraintPoint.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactConstraintPoint = b2ContactConstraintPoint;

	function b2ContactEdge() {
		b2ContactEdge.b2ContactEdge.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactEdge = b2ContactEdge;

	function b2ContactFactory() {
		b2ContactFactory.b2ContactFactory.apply(this, arguments);
		if (this.constructor === b2ContactFactory) this.b2ContactFactory.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactFactory = b2ContactFactory;

	function b2ContactRegister() {
		b2ContactRegister.b2ContactRegister.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactRegister = b2ContactRegister;

	function b2ContactResult() {
		b2ContactResult.b2ContactResult.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactResult = b2ContactResult;

	function b2ContactSolver() {
		b2ContactSolver.b2ContactSolver.apply(this, arguments);
		if (this.constructor === b2ContactSolver) this.b2ContactSolver.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2ContactSolver = b2ContactSolver;

	function b2EdgeAndCircleContact() {
		b2EdgeAndCircleContact.b2EdgeAndCircleContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact;

	function b2NullContact() {
		b2NullContact.b2NullContact.apply(this, arguments);
		if (this.constructor === b2NullContact) this.b2NullContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2NullContact = b2NullContact;

	function b2PolyAndCircleContact() {
		b2PolyAndCircleContact.b2PolyAndCircleContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2PolyAndCircleContact = b2PolyAndCircleContact;

	function b2PolyAndEdgeContact() {
		b2PolyAndEdgeContact.b2PolyAndEdgeContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = b2PolyAndEdgeContact;

	function b2PolygonContact() {
		b2PolygonContact.b2PolygonContact.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2PolygonContact = b2PolygonContact;

	function b2PositionSolverManifold() {
		b2PositionSolverManifold.b2PositionSolverManifold.apply(this, arguments);
		if (this.constructor === b2PositionSolverManifold) this.b2PositionSolverManifold.apply(this, arguments);
	};
	Box2D.Dynamics.Contacts.b2PositionSolverManifold = b2PositionSolverManifold;


/* Controllers */


	function b2BuoyancyController() {
		b2BuoyancyController.b2BuoyancyController.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2BuoyancyController = b2BuoyancyController;

	function b2ConstantAccelController() {
		b2ConstantAccelController.b2ConstantAccelController.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2ConstantAccelController = b2ConstantAccelController;

	function b2ConstantForceController() {
		b2ConstantForceController.b2ConstantForceController.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2ConstantForceController = b2ConstantForceController;

	function b2Controller() {
		b2Controller.b2Controller.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2Controller = b2Controller;

	function b2ControllerEdge() {
		b2ControllerEdge.b2ControllerEdge.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2ControllerEdge = b2ControllerEdge;

	function b2GravityController() {
		b2GravityController.b2GravityController.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2GravityController = b2GravityController;

	function b2TensorDampingController() {
		b2TensorDampingController.b2TensorDampingController.apply(this, arguments);
	};
	Box2D.Dynamics.Controllers.b2TensorDampingController = b2TensorDampingController;


/* Joints */


	function b2DistanceJoint() {
		b2DistanceJoint.b2DistanceJoint.apply(this, arguments);
		if (this.constructor === b2DistanceJoint) this.b2DistanceJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2DistanceJoint = b2DistanceJoint;

	function b2DistanceJointDef() {
		b2DistanceJointDef.b2DistanceJointDef.apply(this, arguments);
		if (this.constructor === b2DistanceJointDef) this.b2DistanceJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef;

	function b2FrictionJoint() {
		b2FrictionJoint.b2FrictionJoint.apply(this, arguments);
		if (this.constructor === b2FrictionJoint) this.b2FrictionJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2FrictionJoint = b2FrictionJoint;

	function b2FrictionJointDef() {
		b2FrictionJointDef.b2FrictionJointDef.apply(this, arguments);
		if (this.constructor === b2FrictionJointDef) this.b2FrictionJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2FrictionJointDef = b2FrictionJointDef;

	function b2GearJoint() {
		b2GearJoint.b2GearJoint.apply(this, arguments);
		if (this.constructor === b2GearJoint) this.b2GearJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint;

	function b2GearJointDef() {
		b2GearJointDef.b2GearJointDef.apply(this, arguments);
		if (this.constructor === b2GearJointDef) this.b2GearJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef;

	function b2Jacobian() {
		b2Jacobian.b2Jacobian.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian;

	function b2Joint() {
		b2Joint.b2Joint.apply(this, arguments);
		if (this.constructor === b2Joint) this.b2Joint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2Joint = b2Joint;

	function b2JointDef() {
		b2JointDef.b2JointDef.apply(this, arguments);
		if (this.constructor === b2JointDef) this.b2JointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2JointDef = b2JointDef;

	function b2JointEdge() {
		b2JointEdge.b2JointEdge.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2JointEdge = b2JointEdge;

	function b2LineJoint() {
		b2LineJoint.b2LineJoint.apply(this, arguments);
		if (this.constructor === b2LineJoint) this.b2LineJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2LineJoint = b2LineJoint;

	function b2LineJointDef() {
		b2LineJointDef.b2LineJointDef.apply(this, arguments);
		if (this.constructor === b2LineJointDef) this.b2LineJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2LineJointDef = b2LineJointDef;

	function b2MouseJoint() {
		b2MouseJoint.b2MouseJoint.apply(this, arguments);
		if (this.constructor === b2MouseJoint) this.b2MouseJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2MouseJoint = b2MouseJoint;

	function b2MouseJointDef() {
		b2MouseJointDef.b2MouseJointDef.apply(this, arguments);
		if (this.constructor === b2MouseJointDef) this.b2MouseJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef;

	function b2PrismaticJoint() {
		b2PrismaticJoint.b2PrismaticJoint.apply(this, arguments);
		if (this.constructor === b2PrismaticJoint) this.b2PrismaticJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2PrismaticJoint = b2PrismaticJoint;

	function b2PrismaticJointDef() {
		b2PrismaticJointDef.b2PrismaticJointDef.apply(this, arguments);
		if (this.constructor === b2PrismaticJointDef) this.b2PrismaticJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef;

	function b2PulleyJoint() {
		b2PulleyJoint.b2PulleyJoint.apply(this, arguments);
		if (this.constructor === b2PulleyJoint) this.b2PulleyJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2PulleyJoint = b2PulleyJoint;

	function b2PulleyJointDef() {
		b2PulleyJointDef.b2PulleyJointDef.apply(this, arguments);
		if (this.constructor === b2PulleyJointDef) this.b2PulleyJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef;

	function b2RevoluteJoint() {
		b2RevoluteJoint.b2RevoluteJoint.apply(this, arguments);
		if (this.constructor === b2RevoluteJoint) this.b2RevoluteJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint;

	function b2RevoluteJointDef() {
		b2RevoluteJointDef.b2RevoluteJointDef.apply(this, arguments);
		if (this.constructor === b2RevoluteJointDef) this.b2RevoluteJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef;

	function b2WeldJoint() {
		b2WeldJoint.b2WeldJoint.apply(this, arguments);
		if (this.constructor === b2WeldJoint) this.b2WeldJoint.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2WeldJoint = b2WeldJoint;

	function b2WeldJointDef() {
		b2WeldJointDef.b2WeldJointDef.apply(this, arguments);
		if (this.constructor === b2WeldJointDef) this.b2WeldJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2WeldJointDef = b2WeldJointDef;
})(); //definitions
Box2D.postDefs = [];
(function () {
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_localTangent = new b2Vec2();
		Box2D.Collision.b2Collision.s_localNormal = new b2Vec2();
		Box2D.Collision.b2Collision.s_planePoint = new b2Vec2();
		Box2D.Collision.b2Collision.s_normal = new b2Vec2();
		Box2D.Collision.b2Collision.s_tangent = new b2Vec2();
		Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2();
		Box2D.Collision.b2Collision.s_v11 = new b2Vec2();
		Box2D.Collision.b2Collision.s_v12 = new b2Vec2();
		Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2();
		Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff;
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2Distance.s_simplex = new b2Simplex();
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache();
		Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
		Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform();
		Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform();
		Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction();
		Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.Shapes.b2PolygonShape.s_mat = new b2Mat22();
	});
	Box2D.postDefs.push(function () {
		Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);
		Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
		Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2Body.s_xf1 = new b2Transform();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2ContactManager.s_evalCP = new b2ContactPoint();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2Island.s_impulse = new b2ContactImpulse();
	});
})();
(function () {



	
	
	
	
	
	
	b2TimeStep.b2TimeStep = function () {};
	b2TimeStep.prototype.Set = function (step) {
		this.dt = step.dt;
		this.inv_dt = step.inv_dt;
		this.positionIterations = step.positionIterations;
		this.velocityIterations = step.velocityIterations;
		this.warmStarting = step.warmStarting;
	}
	b2World.b2World = function () {
		this.s_stack = new Vector();
		this.m_contactManager = new b2ContactManager();
		this.m_contactSolver = new b2ContactSolver();
		this.m_island = new b2Island();
	};
	b2World.prototype.b2World = function (gravity, doSleep) {
		this.m_destructionListener = null;
		this.m_debugDraw = null;
		this.m_bodyList = null;
		this.m_contactList = null;
		this.m_jointList = null;
		this.m_controllerList = null;
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
		this.m_controllerCount = 0;
		b2World.m_warmStarting = true;
		b2World.m_continuousPhysics = true;
		this.m_allowSleep = doSleep;
		this.m_gravity = gravity;
		this.m_inv_dt0 = 0.0;
		this.m_contactManager.m_world = this;
		var bd = new b2BodyDef();
		this.m_groundBody = this.CreateBody(bd);
	}
	b2World.prototype.SetDestructionListener = function (listener) {
		this.m_destructionListener = listener;
	}
	b2World.prototype.SetContactFilter = function (filter) {
		this.m_contactManager.m_contactFilter = filter;
	}
	b2World.prototype.SetContactListener = function (listener) {
		this.m_contactManager.m_contactListener = listener;
	}
	b2World.prototype.SetDebugDraw = function (debugDraw) {
		this.m_debugDraw = debugDraw;
	}
	b2World.prototype.SetBroadPhase = function (broadPhase) {
		var oldBroadPhase = this.m_contactManager.m_broadPhase;
		this.m_contactManager.m_broadPhase = broadPhase;
		for (var b = this.m_bodyList; b; b = b.m_next) {
			for (var f = b.m_fixtureList; f; f = f.m_next) {
				f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
			}
		}
	}
	b2World.prototype.Validate = function () {
		this.m_contactManager.m_broadPhase.Validate();
	}
	b2World.prototype.GetProxyCount = function () {
		return this.m_contactManager.m_broadPhase.GetProxyCount();
	}
	b2World.prototype.CreateBody = function (def) {
		if (this.IsLocked() == true) {
			return null;
		}
		var b = new b2Body(def, this);
		b.m_prev = null;
		b.m_next = this.m_bodyList;
		if (this.m_bodyList) {
			this.m_bodyList.m_prev = b;
		}
		this.m_bodyList = b;
		++this.m_bodyCount;
		return b;
	}
	b2World.prototype.DestroyBody = function (b) {
		if (this.IsLocked() == true) {
			return;
		}
		var jn = b.m_jointList;
		while (jn) {
			var jn0 = jn;
			jn = jn.next;
			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
			}
			this.DestroyJoint(jn0.joint);
		}
		var coe = b.m_controllerList;
		while (coe) {
			var coe0 = coe;
			coe = coe.nextController;
			coe0.controller.RemoveBody(b);
		}
		var ce = b.m_contactList;
		while (ce) {
			var ce0 = ce;
			ce = ce.next;
			this.m_contactManager.Destroy(ce0.contact);
		}
		b.m_contactList = null;
		var f = b.m_fixtureList;
		while (f) {
			var f0 = f;
			f = f.m_next;
			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeFixture(f0);
			}
			f0.DestroyProxy(this.m_contactManager.m_broadPhase);
			f0.Destroy();
		}
		b.m_fixtureList = null;
		b.m_fixtureCount = 0;
		if (b.m_prev) {
			b.m_prev.m_next = b.m_next;
		}
		if (b.m_next) {
			b.m_next.m_prev = b.m_prev;
		}
		if (b == this.m_bodyList) {
			this.m_bodyList = b.m_next;
		}--this.m_bodyCount;
	}
	b2World.prototype.CreateJoint = function (def) {
		var j = b2Joint.Create(def, null);
		j.m_prev = null;
		j.m_next = this.m_jointList;
		if (this.m_jointList) {
			this.m_jointList.m_prev = j;
		}
		this.m_jointList = j;
		++this.m_jointCount;
		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;
		if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
		j.m_bodyA.m_jointList = j.m_edgeA;
		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;
		if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
		j.m_bodyB.m_jointList = j.m_edgeB;
		var bodyA = def.bodyA;
		var bodyB = def.bodyB;
		if (def.collideConnected == false) {
			var edge = bodyB.GetContactList();
			while (edge) {
				if (edge.other == bodyA) {
					edge.contact.FlagForFiltering();
				}
				edge = edge.next;
			}
		}
		return j;
	}
	b2World.prototype.DestroyJoint = function (j) {
		var collideConnected = j.m_collideConnected;
		if (j.m_prev) {
			j.m_prev.m_next = j.m_next;
		}
		if (j.m_next) {
			j.m_next.m_prev = j.m_prev;
		}
		if (j == this.m_jointList) {
			this.m_jointList = j.m_next;
		}
		var bodyA = j.m_bodyA;
		var bodyB = j.m_bodyB;
		bodyA.SetAwake(true);
		bodyB.SetAwake(true);
		if (j.m_edgeA.prev) {
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}
		if (j.m_edgeA.next) {
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}
		if (j.m_edgeA == bodyA.m_jointList) {
			bodyA.m_jointList = j.m_edgeA.next;
		}
		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;
		if (j.m_edgeB.prev) {
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}
		if (j.m_edgeB.next) {
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}
		if (j.m_edgeB == bodyB.m_jointList) {
			bodyB.m_jointList = j.m_edgeB.next;
		}
		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;
		b2Joint.Destroy(j, null);
		--this.m_jointCount;
		if (collideConnected == false) {
			var edge = bodyB.GetContactList();
			while (edge) {
				if (edge.other == bodyA) {
					edge.contact.FlagForFiltering();
				}
				edge = edge.next;
			}
		}
	}
	b2World.prototype.AddController = function (c) {
		c.m_next = this.m_controllerList;
		c.m_prev = null;
		this.m_controllerList = c;
		c.m_world = this;
		this.m_controllerCount++;
		return c;
	}
	b2World.prototype.RemoveController = function (c) {
		if (c.m_prev) c.m_prev.m_next = c.m_next;
		if (c.m_next) c.m_next.m_prev = c.m_prev;
		if (this.m_controllerList == c) this.m_controllerList = c.m_next;
		this.m_controllerCount--;
	}
	b2World.prototype.CreateController = function (controller) {
		if (controller.m_world != this) throw new Error("Controller can only be a member of one world");
		controller.m_next = this.m_controllerList;
		controller.m_prev = null;
		if (this.m_controllerList) this.m_controllerList.m_prev = controller;
		this.m_controllerList = controller;
		++this.m_controllerCount;
		controller.m_world = this;
		return controller;
	}
	b2World.prototype.DestroyController = function (controller) {
		controller.Clear();
		if (controller.m_next) controller.m_next.m_prev = controller.m_prev;
		if (controller.m_prev) controller.m_prev.m_next = controller.m_next;
		if (controller == this.m_controllerList) this.m_controllerList = controller.m_next;
		--this.m_controllerCount;
	}
	b2World.prototype.SetWarmStarting = function (flag) {
		b2World.m_warmStarting = flag;
	}
	b2World.prototype.SetContinuousPhysics = function (flag) {
		b2World.m_continuousPhysics = flag;
	}
	b2World.prototype.GetBodyCount = function () {
		return this.m_bodyCount;
	}
	b2World.prototype.GetJointCount = function () {
		return this.m_jointCount;
	}
	b2World.prototype.GetContactCount = function () {
		return this.m_contactCount;
	}
	b2World.prototype.SetGravity = function (gravity) {
		this.m_gravity = gravity;
	}
	b2World.prototype.GetGravity = function () {
		return this.m_gravity;
	}
	b2World.prototype.GetGroundBody = function () {
		return this.m_groundBody;
	}
	b2World.prototype.Step = function (dt, velocityIterations, positionIterations) {
		if (dt === undefined) dt = 0;
		if (velocityIterations === undefined) velocityIterations = 0;
		if (positionIterations === undefined) positionIterations = 0;
		if (this.m_flags & b2World.e_newFixture) {
			this.m_contactManager.FindNewContacts();
			this.m_flags &= ~b2World.e_newFixture;
		}
		this.m_flags |= b2World.e_locked;
		var step = b2World.s_timestep2;
		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0) {
			step.inv_dt = 1.0 / dt;
		}
		else {
			step.inv_dt = 0.0;
		}
		step.dtRatio = this.m_inv_dt0 * dt;
		step.warmStarting = b2World.m_warmStarting;
		this.m_contactManager.Collide();
		if (step.dt > 0.0) {
			this.Solve(step);
		}
		if (b2World.m_continuousPhysics && step.dt > 0.0) {
			this.SolveTOI(step);
		}
		if (step.dt > 0.0) {
			this.m_inv_dt0 = step.inv_dt;
		}
		this.m_flags &= ~b2World.e_locked;
	}
	b2World.prototype.ClearForces = function () {
		for (var body = this.m_bodyList; body; body = body.m_next) {
			body.m_force.SetZero();
			body.m_torque = 0.0;
		}
	}
	b2World.prototype.DrawDebugData = function () {
		if (this.m_debugDraw == null) {
			return;
		}
		this.m_debugDraw.m_sprite.graphics.clear();
		var flags = this.m_debugDraw.GetFlags();
		var i = 0;
		var b;
		var f;
		var s;
		var j;
		var bp;
		var invQ = new b2Vec2;
		var x1 = new b2Vec2;
		var x2 = new b2Vec2;
		var xf;
		var b1 = new b2AABB();
		var b2 = new b2AABB();
		var vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
		var color = new b2Color(0, 0, 0);
		if (flags & b2DebugDraw.e_shapeBit) {
			for (b = this.m_bodyList;
			b; b = b.m_next) {
				xf = b.m_xf;
				for (f = b.GetFixtureList();
				f; f = f.m_next) {
					s = f.GetShape();
					if (b.IsActive() == false) {
						color.Set(0.5, 0.5, 0.3);
						this.DrawShape(s, xf, color);
					}
					else if (b.GetType() == b2Body.b2_staticBody) {
						color.Set(0.5, 0.9, 0.5);
						this.DrawShape(s, xf, color);
					}
					else if (b.GetType() == b2Body.b2_kinematicBody) {
						color.Set(0.5, 0.5, 0.9);
						this.DrawShape(s, xf, color);
					}
					else if (b.IsAwake() == false) {
						color.Set(0.6, 0.6, 0.6);
						this.DrawShape(s, xf, color);
					}
					else {
						color.Set(0.9, 0.7, 0.7);
						this.DrawShape(s, xf, color);
					}
				}
			}
		}
		if (flags & b2DebugDraw.e_jointBit) {
			for (j = this.m_jointList;
			j; j = j.m_next) {
				this.DrawJoint(j);
			}
		}
		if (flags & b2DebugDraw.e_controllerBit) {
			for (var c = this.m_controllerList; c; c = c.m_next) {
				c.Draw(this.m_debugDraw);
			}
		}
		if (flags & b2DebugDraw.e_pairBit) {
			color.Set(0.3, 0.9, 0.9);
			for (var contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
				var fixtureA = contact.GetFixtureA();
				var fixtureB = contact.GetFixtureB();
				var cA = fixtureA.GetAABB().GetCenter();
				var cB = fixtureB.GetAABB().GetCenter();
				this.m_debugDraw.DrawSegment(cA, cB, color);
			}
		}
		if (flags & b2DebugDraw.e_aabbBit) {
			bp = this.m_contactManager.m_broadPhase;
			vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
			for (b = this.m_bodyList;
			b; b = b.GetNext()) {
				if (b.IsActive() == false) {
					continue;
				}
				for (f = b.GetFixtureList();
				f; f = f.GetNext()) {
					var aabb = bp.GetFatAABB(f.m_proxy);
					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
					this.m_debugDraw.DrawPolygon(vs, 4, color);
				}
			}
		}
		if (flags & b2DebugDraw.e_centerOfMassBit) {
			for (b = this.m_bodyList;
			b; b = b.m_next) {
				xf = b2World.s_xf;
				xf.R = b.m_xf.R;
				xf.position = b.GetWorldCenter();
				this.m_debugDraw.DrawTransform(xf);
			}
		}
	}
	b2World.prototype.QueryAABB = function (callback, aabb) {
		var __this = this;
		var broadPhase = __this.m_contactManager.m_broadPhase;

		function WorldQueryWrapper(proxy) {
			return callback(broadPhase.GetUserData(proxy));
		};
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
	b2World.prototype.QueryShape = function (callback, shape, transform) {
		var __this = this;
		if (transform === undefined) transform = null;
		if (transform == null) {
			transform = new b2Transform();
			transform.SetIdentity();
		}
		var broadPhase = __this.m_contactManager.m_broadPhase;

		function WorldQueryWrapper(proxy) {
			var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
			if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
			return true;
		};
		var aabb = new b2AABB();
		shape.ComputeAABB(aabb, transform);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
	b2World.prototype.QueryPoint = function (callback, p) {
		var __this = this;
		var broadPhase = __this.m_contactManager.m_broadPhase;

		function WorldQueryWrapper(proxy) {
			var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
			if (fixture.TestPoint(p)) return callback(fixture);
			return true;
		};
		var aabb = new b2AABB();
		aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
		aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
		broadPhase.Query(WorldQueryWrapper, aabb);
	}
	b2World.prototype.RayCast = function (callback, point1, point2) {
		var __this = this;
		var broadPhase = __this.m_contactManager.m_broadPhase;
		var output = new b2RayCastOutput;

		function RayCastWrapper(input, proxy) {
			var userData = broadPhase.GetUserData(proxy);
			var fixture = (userData instanceof b2Fixture ? userData : null);
			var hit = fixture.RayCast(output, input);
			if (hit) {
				var fraction = output.fraction;
				var point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y);
				return callback(fixture, point, output.normal, fraction);
			}
			return input.maxFraction;
		};
		var input = new b2RayCastInput(point1, point2);
		broadPhase.RayCast(RayCastWrapper, input);
	}
	b2World.prototype.RayCastOne = function (point1, point2) {
		var __this = this;
		var result;

		function RayCastOneWrapper(fixture, point, normal, fraction) {
			if (fraction === undefined) fraction = 0;
			result = fixture;
			return fraction;
		};
		__this.RayCast(RayCastOneWrapper, point1, point2);
		return result;
	}
	b2World.prototype.RayCastAll = function (point1, point2) {
		var __this = this;
		var result = new Vector();

		function RayCastAllWrapper(fixture, point, normal, fraction) {
			if (fraction === undefined) fraction = 0;
			result[result.length] = fixture;
			return 1;
		};
		__this.RayCast(RayCastAllWrapper, point1, point2);
		return result;
	}
	b2World.prototype.GetBodyList = function () {
		return this.m_bodyList;
	}
	b2World.prototype.GetJointList = function () {
		return this.m_jointList;
	}
	b2World.prototype.GetContactList = function () {
		return this.m_contactList;
	}
	b2World.prototype.IsLocked = function () {
		return (this.m_flags & b2World.e_locked) > 0;
	}
	b2World.prototype.Solve = function (step) {
		var b;
		for (var controller = this.m_controllerList; controller; controller = controller.m_next) {
			controller.Step(step);
		}
		var island = this.m_island;
		island.Initialize(this.m_bodyCount, this.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
		for (b = this.m_bodyList;
		b; b = b.m_next) {
			b.m_flags &= ~b2Body.e_islandFlag;
		}
		for (var c = this.m_contactList; c; c = c.m_next) {
			c.m_flags &= ~b2Contact.e_islandFlag;
		}
		for (var j = this.m_jointList; j; j = j.m_next) {
			j.m_islandFlag = false;
		}
		var stackSize = parseInt(this.m_bodyCount);
		var stack = this.s_stack;
		for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
			if (seed.m_flags & b2Body.e_islandFlag) {
				continue;
			}
			if (seed.IsAwake() == false || seed.IsActive() == false) {
				continue;
			}
			if (seed.GetType() == b2Body.b2_staticBody) {
				continue;
			}
			island.Clear();
			var stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;
			while (stackCount > 0) {
				b = stack[--stackCount];
				island.AddBody(b);
				if (b.IsAwake() == false) {
					b.SetAwake(true);
				}
				if (b.GetType() == b2Body.b2_staticBody) {
					continue;
				}
				var other;
				for (var ce = b.m_contactList; ce; ce = ce.next) {
					if (ce.contact.m_flags & b2Contact.e_islandFlag) {
						continue;
					}
					if (ce.contact.IsSensor() == true || ce.contact.IsEnabled() == false || ce.contact.IsTouching() == false) {
						continue;
					}
					island.AddContact(ce.contact);
					ce.contact.m_flags |= b2Contact.e_islandFlag;
					other = ce.other;
					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
				for (var jn = b.m_jointList; jn; jn = jn.next) {
					if (jn.joint.m_islandFlag == true) {
						continue;
					}
					other = jn.other;
					if (other.IsActive() == false) {
						continue;
					}
					island.AddJoint(jn.joint);
					jn.joint.m_islandFlag = true;
					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			island.Solve(step, this.m_gravity, this.m_allowSleep);
			for (var i = 0; i < island.m_bodyCount; ++i) {
				b = island.m_bodies[i];
				if (b.GetType() == b2Body.b2_staticBody) {
					b.m_flags &= ~b2Body.e_islandFlag;
				}
			}
		}
		for (i = 0;
		i < stack.length; ++i) {
			if (!stack[i]) break;
			stack[i] = null;
		}
		for (b = this.m_bodyList;
		b; b = b.m_next) {
			if (b.IsAwake() == false || b.IsActive() == false) {
				continue;
			}
			if (b.GetType() == b2Body.b2_staticBody) {
				continue;
			}
			b.SynchronizeFixtures();
		}
		this.m_contactManager.FindNewContacts();
	}
	b2World.prototype.SolveTOI = function (step) {
		var b;
		var fA;
		var fB;
		var bA;
		var bB;
		var cEdge;
		var j;
		var island = this.m_island;
		island.Initialize(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
		var queue = b2World.s_queue;
		for (b = this.m_bodyList;
		b; b = b.m_next) {
			b.m_flags &= ~b2Body.e_islandFlag;
			b.m_sweep.t0 = 0.0;
		}
		var c;
		for (c = this.m_contactList;
		c; c = c.m_next) {
			c.m_flags &= ~ (b2Contact.e_toiFlag | b2Contact.e_islandFlag);
		}
		for (j = this.m_jointList;
		j; j = j.m_next) {
			j.m_islandFlag = false;
		}
		for (;;) {
			var minContact = null;
			var minTOI = 1.0;
			for (c = this.m_contactList;
			c; c = c.m_next) {
				if (c.IsSensor() == true || c.IsEnabled() == false || c.IsContinuous() == false) {
					continue;
				}
				var toi = 1.0;
				if (c.m_flags & b2Contact.e_toiFlag) {
					toi = c.m_toi;
				}
				else {
					fA = c.m_fixtureA;
					fB = c.m_fixtureB;
					bA = fA.m_body;
					bB = fB.m_body;
					if ((bA.GetType() != b2Body.b2_dynamicBody || bA.IsAwake() == false) && (bB.GetType() != b2Body.b2_dynamicBody || bB.IsAwake() == false)) {
						continue;
					}
					var t0 = bA.m_sweep.t0;
					if (bA.m_sweep.t0 < bB.m_sweep.t0) {
						t0 = bB.m_sweep.t0;
						bA.m_sweep.Advance(t0);
					}
					else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
						t0 = bA.m_sweep.t0;
						bB.m_sweep.Advance(t0);
					}
					toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
					b2Settings.b2Assert(0.0 <= toi && toi <= 1.0);
					if (toi > 0.0 && toi < 1.0) {
						toi = (1.0 - toi) * t0 + toi;
						if (toi > 1) toi = 1;
					}
					c.m_toi = toi;
					c.m_flags |= b2Contact.e_toiFlag;
				}
				if (Number.MIN_VALUE < toi && toi < minTOI) {
					minContact = c;
					minTOI = toi;
				}
			}
			if (minContact == null || 1.0 - 100.0 * Number.MIN_VALUE < minTOI) {
				break;
			}
			fA = minContact.m_fixtureA;
			fB = minContact.m_fixtureB;
			bA = fA.m_body;
			bB = fB.m_body;
			b2World.s_backupA.Set(bA.m_sweep);
			b2World.s_backupB.Set(bB.m_sweep);
			bA.Advance(minTOI);
			bB.Advance(minTOI);
			minContact.Update(this.m_contactManager.m_contactListener);
			minContact.m_flags &= ~b2Contact.e_toiFlag;
			if (minContact.IsSensor() == true || minContact.IsEnabled() == false) {
				bA.m_sweep.Set(b2World.s_backupA);
				bB.m_sweep.Set(b2World.s_backupB);
				bA.SynchronizeTransform();
				bB.SynchronizeTransform();
				continue;
			}
			if (minContact.IsTouching() == false) {
				continue;
			}
			var seed = bA;
			if (seed.GetType() != b2Body.b2_dynamicBody) {
				seed = bB;
			}
			island.Clear();
			var queueStart = 0;
			var queueSize = 0;
			queue[queueStart + queueSize++] = seed;
			seed.m_flags |= b2Body.e_islandFlag;
			while (queueSize > 0) {
				b = queue[queueStart++];
				--queueSize;
				island.AddBody(b);
				if (b.IsAwake() == false) {
					b.SetAwake(true);
				}
				if (b.GetType() != b2Body.b2_dynamicBody) {
					continue;
				}
				for (cEdge = b.m_contactList;
				cEdge; cEdge = cEdge.next) {
					if (island.m_contactCount == island.m_contactCapacity) {
						break;
					}
					if (cEdge.contact.m_flags & b2Contact.e_islandFlag) {
						continue;
					}
					if (cEdge.contact.IsSensor() == true || cEdge.contact.IsEnabled() == false || cEdge.contact.IsTouching() == false) {
						continue;
					}
					island.AddContact(cEdge.contact);
					cEdge.contact.m_flags |= b2Contact.e_islandFlag;
					var other = cEdge.other;
					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}
					if (other.GetType() != b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}
					queue[queueStart + queueSize] = other;
					++queueSize;
					other.m_flags |= b2Body.e_islandFlag;
				}
				for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
					if (island.m_jointCount == island.m_jointCapacity) continue;
					if (jEdge.joint.m_islandFlag == true) continue;
					other = jEdge.other;
					if (other.IsActive() == false) {
						continue;
					}
					island.AddJoint(jEdge.joint);
					jEdge.joint.m_islandFlag = true;
					if (other.m_flags & b2Body.e_islandFlag) continue;
					if (other.GetType() != b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}
					queue[queueStart + queueSize] = other;
					++queueSize;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}
			var subStep = b2World.s_timestep;
			subStep.warmStarting = false;
			subStep.dt = (1.0 - minTOI) * step.dt;
			subStep.inv_dt = 1.0 / subStep.dt;
			subStep.dtRatio = 0.0;
			subStep.velocityIterations = step.velocityIterations;
			subStep.positionIterations = step.positionIterations;
			island.SolveTOI(subStep);
			var i = 0;
			for (i = 0;
			i < island.m_bodyCount; ++i) {
				b = island.m_bodies[i];
				b.m_flags &= ~b2Body.e_islandFlag;
				if (b.IsAwake() == false) {
					continue;
				}
				if (b.GetType() != b2Body.b2_dynamicBody) {
					continue;
				}
				b.SynchronizeFixtures();
				for (cEdge = b.m_contactList;
				cEdge; cEdge = cEdge.next) {
					cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
				}
			}
			for (i = 0;
			i < island.m_contactCount; ++i) {
				c = island.m_contacts[i];
				c.m_flags &= ~ (b2Contact.e_toiFlag | b2Contact.e_islandFlag);
			}
			for (i = 0;
			i < island.m_jointCount; ++i) {
				j = island.m_joints[i];
				j.m_islandFlag = false;
			}
			this.m_contactManager.FindNewContacts();
		}
	}
	b2World.prototype.DrawJoint = function (joint) {
		var b1 = joint.GetBodyA();
		var b2 = joint.GetBodyB();
		var xf1 = b1.m_xf;
		var xf2 = b2.m_xf;
		var x1 = xf1.position;
		var x2 = xf2.position;
		var p1 = joint.GetAnchorA();
		var p2 = joint.GetAnchorB();
		var color = b2World.s_jointColor;
		switch (joint.m_type) {
		case b2Joint.e_distanceJoint:
			this.m_debugDraw.DrawSegment(p1, p2, color);
			break;
		case b2Joint.e_pulleyJoint:
			{
				var pulley = ((joint instanceof b2PulleyJoint ? joint : null));
				var s1 = pulley.GetGroundAnchorA();
				var s2 = pulley.GetGroundAnchorB();
				this.m_debugDraw.DrawSegment(s1, p1, color);
				this.m_debugDraw.DrawSegment(s2, p2, color);
				this.m_debugDraw.DrawSegment(s1, s2, color);
			}
			break;
		case b2Joint.e_mouseJoint:
			this.m_debugDraw.DrawSegment(p1, p2, color);
			break;
		default:
			if (b1 != this.m_groundBody) this.m_debugDraw.DrawSegment(x1, p1, color);
			this.m_debugDraw.DrawSegment(p1, p2, color);
			if (b2 != this.m_groundBody) this.m_debugDraw.DrawSegment(x2, p2, color);
		}
	}
	b2World.prototype.DrawShape = function (shape, xf, color) {
		switch (shape.m_type) {
		case b2Shape.e_circleShape:
			{
				var circle = ((shape instanceof b2CircleShape ? shape : null));
				var center = b2Math.MulX(xf, circle.m_p);
				var radius = circle.m_radius;
				var axis = xf.R.col1;
				this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
			}
			break;
		case b2Shape.e_polygonShape:
			{
				var i = 0;
				var poly = ((shape instanceof b2PolygonShape ? shape : null));
				var vertexCount = parseInt(poly.GetVertexCount());
				var localVertices = poly.GetVertices();
				var vertices = new Vector(vertexCount);
				for (i = 0;
				i < vertexCount; ++i) {
					vertices[i] = b2Math.MulX(xf, localVertices[i]);
				}
				this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
			}
			break;
		case b2Shape.e_edgeShape:
			{
				var edge = (shape instanceof b2EdgeShape ? shape : null);
				this.m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);
			}
			break;
		}
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2World.s_timestep2 = new b2TimeStep();
		Box2D.Dynamics.b2World.s_xf = new b2Transform();
		Box2D.Dynamics.b2World.s_backupA = new b2Sweep();
		Box2D.Dynamics.b2World.s_backupB = new b2Sweep();
		Box2D.Dynamics.b2World.s_timestep = new b2TimeStep();
		Box2D.Dynamics.b2World.s_queue = new Vector();
		Box2D.Dynamics.b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
		Box2D.Dynamics.b2World.e_newFixture = 0x0001;
		Box2D.Dynamics.b2World.e_locked = 0x0002;
	});
})();
(function () {
	
	Box2D.inherit(b2CircleContact, Box2D.Dynamics.Contacts.b2Contact);
	b2CircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2CircleContact.b2CircleContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2CircleContact.Create = function (allocator) {
		return new b2CircleContact();
	}
	b2CircleContact.Destroy = function (contact, allocator) {}
	b2CircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		this.__super.Reset.call(this, fixtureA, fixtureB);
	}
	b2CircleContact.prototype.Evaluate = function () {
		var bA = this.m_fixtureA.GetBody();
		var bB = this.m_fixtureB.GetBody();
		b2Collision.CollideCircles(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2CircleShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
	}
	b2Contact.b2Contact = function () {
		this.m_nodeA = new b2ContactEdge();
		this.m_nodeB = new b2ContactEdge();
		this.m_manifold = new b2Manifold();
		this.m_oldManifold = new b2Manifold();
	};
	b2Contact.prototype.GetManifold = function () {
		return this.m_manifold;
	}
	b2Contact.prototype.GetWorldManifold = function (worldManifold) {
		var bodyA = this.m_fixtureA.GetBody();
		var bodyB = this.m_fixtureB.GetBody();
		var shapeA = this.m_fixtureA.GetShape();
		var shapeB = this.m_fixtureB.GetShape();
		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}
	b2Contact.prototype.IsTouching = function () {
		return (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
	}
	b2Contact.prototype.IsContinuous = function () {
		return (this.m_flags & b2Contact.e_continuousFlag) == b2Contact.e_continuousFlag;
	}
	b2Contact.prototype.SetSensor = function (sensor) {
		if (sensor) {
			this.m_flags |= b2Contact.e_sensorFlag;
		}
		else {
			this.m_flags &= ~b2Contact.e_sensorFlag;
		}
	}
	b2Contact.prototype.IsSensor = function () {
		return (this.m_flags & b2Contact.e_sensorFlag) == b2Contact.e_sensorFlag;
	}
	b2Contact.prototype.SetEnabled = function (flag) {
		if (flag) {
			this.m_flags |= b2Contact.e_enabledFlag;
		}
		else {
			this.m_flags &= ~b2Contact.e_enabledFlag;
		}
	}
	b2Contact.prototype.IsEnabled = function () {
		return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
	}
	b2Contact.prototype.GetNext = function () {
		return this.m_next;
	}
	b2Contact.prototype.GetFixtureA = function () {
		return this.m_fixtureA;
	}
	b2Contact.prototype.GetFixtureB = function () {
		return this.m_fixtureB;
	}
	b2Contact.prototype.FlagForFiltering = function () {
		this.m_flags |= b2Contact.e_filterFlag;
	}
	b2Contact.prototype.b2Contact = function () {}
	b2Contact.prototype.Reset = function (fixtureA, fixtureB) {
		if (fixtureA === undefined) fixtureA = null;
		if (fixtureB === undefined) fixtureB = null;
		this.m_flags = b2Contact.e_enabledFlag;
		if (!fixtureA || !fixtureB) {
			this.m_fixtureA = null;
			this.m_fixtureB = null;
			return;
		}
		if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
			this.m_flags |= b2Contact.e_sensorFlag;
		}
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();
		if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet()) {
			this.m_flags |= b2Contact.e_continuousFlag;
		}
		this.m_fixtureA = fixtureA;
		this.m_fixtureB = fixtureB;
		this.m_manifold.m_pointCount = 0;
		this.m_prev = null;
		this.m_next = null;
		this.m_nodeA.contact = null;
		this.m_nodeA.prev = null;
		this.m_nodeA.next = null;
		this.m_nodeA.other = null;
		this.m_nodeB.contact = null;
		this.m_nodeB.prev = null;
		this.m_nodeB.next = null;
		this.m_nodeB.other = null;
	}
	b2Contact.prototype.Update = function (listener) {
		var tManifold = this.m_oldManifold;
		this.m_oldManifold = this.m_manifold;
		this.m_manifold = tManifold;
		this.m_flags |= b2Contact.e_enabledFlag;
		var touching = false;
		var wasTouching = (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
		var bodyA = this.m_fixtureA.m_body;
		var bodyB = this.m_fixtureB.m_body;
		var aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
		if (this.m_flags & b2Contact.e_sensorFlag) {
			if (aabbOverlap) {
				var shapeA = this.m_fixtureA.GetShape();
				var shapeB = this.m_fixtureB.GetShape();
				var xfA = bodyA.GetTransform();
				var xfB = bodyB.GetTransform();
				touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
			}
			this.m_manifold.m_pointCount = 0;
		}
		else {
			if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet()) {
				this.m_flags |= b2Contact.e_continuousFlag;
			}
			else {
				this.m_flags &= ~b2Contact.e_continuousFlag;
			}
			if (aabbOverlap) {
				this.Evaluate();
				touching = this.m_manifold.m_pointCount > 0;
				for (var i = 0; i < this.m_manifold.m_pointCount; ++i) {
					var mp2 = this.m_manifold.m_points[i];
					mp2.m_normalImpulse = 0.0;
					mp2.m_tangentImpulse = 0.0;
					var id2 = mp2.m_id;
					for (var j = 0; j < this.m_oldManifold.m_pointCount; ++j) {
						var mp1 = this.m_oldManifold.m_points[j];
						if (mp1.m_id.key == id2.key) {
							mp2.m_normalImpulse = mp1.m_normalImpulse;
							mp2.m_tangentImpulse = mp1.m_tangentImpulse;
							break;
						}
					}
				}
			}
			else {
				this.m_manifold.m_pointCount = 0;
			}
			if (touching != wasTouching) {
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}
		if (touching) {
			this.m_flags |= b2Contact.e_touchingFlag;
		}
		else {
			this.m_flags &= ~b2Contact.e_touchingFlag;
		}
		if (wasTouching == false && touching == true) {
			listener.BeginContact(this);
		}
		if (wasTouching == true && touching == false) {
			listener.EndContact(this);
		}
		if ((this.m_flags & b2Contact.e_sensorFlag) == 0) {
			listener.PreSolve(this, this.m_oldManifold);
		}
	}
	b2Contact.prototype.Evaluate = function () {}
	b2Contact.prototype.ComputeTOI = function (sweepA, sweepB) {
		b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
		b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
		b2Contact.s_input.sweepA = sweepA;
		b2Contact.s_input.sweepB = sweepB;
		b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
		return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2Contact.e_sensorFlag = 0x0001;
		Box2D.Dynamics.Contacts.b2Contact.e_continuousFlag = 0x0002;
		Box2D.Dynamics.Contacts.b2Contact.e_islandFlag = 0x0004;
		Box2D.Dynamics.Contacts.b2Contact.e_toiFlag = 0x0008;
		Box2D.Dynamics.Contacts.b2Contact.e_touchingFlag = 0x0010;
		Box2D.Dynamics.Contacts.b2Contact.e_enabledFlag = 0x0020;
		Box2D.Dynamics.Contacts.b2Contact.e_filterFlag = 0x0040;
		Box2D.Dynamics.Contacts.b2Contact.s_input = new b2TOIInput();
	});
	b2ContactConstraint.b2ContactConstraint = function () {
		this.localPlaneNormal = new b2Vec2();
		this.localPoint = new b2Vec2();
		this.normal = new b2Vec2();
		this.normalMass = new b2Mat22();
		this.K = new b2Mat22();
	};
	b2ContactConstraint.prototype.b2ContactConstraint = function () {
		this.points = new Vector(b2Settings.b2_maxManifoldPoints);
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.points[i] = new b2ContactConstraintPoint();
		}
	}
	b2ContactConstraintPoint.b2ContactConstraintPoint = function () {
		this.localPoint = new b2Vec2();
		this.rA = new b2Vec2();
		this.rB = new b2Vec2();
	};
	b2ContactEdge.b2ContactEdge = function () {};
	b2ContactFactory.b2ContactFactory = function () {};
	b2ContactFactory.prototype.b2ContactFactory = function (allocator) {
		this.m_allocator = allocator;
		this.InitializeRegisters();
	}
	b2ContactFactory.prototype.AddType = function (createFcn, destroyFcn, type1, type2) {
		if (type1 === undefined) type1 = 0;
		if (type2 === undefined) type2 = 0;
		this.m_registers[type1][type2].createFcn = createFcn;
		this.m_registers[type1][type2].destroyFcn = destroyFcn;
		this.m_registers[type1][type2].primary = true;
		if (type1 != type2) {
			this.m_registers[type2][type1].createFcn = createFcn;
			this.m_registers[type2][type1].destroyFcn = destroyFcn;
			this.m_registers[type2][type1].primary = false;
		}
	}
	b2ContactFactory.prototype.InitializeRegisters = function () {
		this.m_registers = new Vector(b2Shape.e_shapeTypeCount);
		for (var i = 0; i < b2Shape.e_shapeTypeCount; i++) {
			this.m_registers[i] = new Vector(b2Shape.e_shapeTypeCount);
			for (var j = 0; j < b2Shape.e_shapeTypeCount; j++) {
				this.m_registers[i][j] = new b2ContactRegister();
			}
		}
		this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
	}
	b2ContactFactory.prototype.Create = function (fixtureA, fixtureB) {
		var type1 = parseInt(fixtureA.GetType());
		var type2 = parseInt(fixtureB.GetType());
		var reg = this.m_registers[type1][type2];
		var c;
		if (reg.pool) {
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}
		var createFcn = reg.createFcn;
		if (createFcn != null) {
			if (reg.primary) {
				c = createFcn(this.m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			}
			else {
				c = createFcn(this.m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		}
		else {
			return null;
		}
	}
	b2ContactFactory.prototype.Destroy = function (contact) {
		if (contact.m_manifold.m_pointCount > 0) {
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
		var type1 = parseInt(contact.m_fixtureA.GetType());
		var type2 = parseInt(contact.m_fixtureB.GetType());
		var reg = this.m_registers[type1][type2];
		if (true) {
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}
		var destroyFcn = reg.destroyFcn;
		destroyFcn(contact, this.m_allocator);
	}
	b2ContactRegister.b2ContactRegister = function () {};
	b2ContactResult.b2ContactResult = function () {
		this.position = new b2Vec2();
		this.normal = new b2Vec2();
		this.id = new b2ContactID();
	};
	b2ContactSolver.b2ContactSolver = function () {
		this.m_step = new b2TimeStep();
		this.m_constraints = new Vector();
	};
	b2ContactSolver.prototype.b2ContactSolver = function () {}
	b2ContactSolver.prototype.Initialize = function (step, contacts, contactCount, allocator) {
		if (contactCount === undefined) contactCount = 0;
		var contact;
		this.m_step.Set(step);
		this.m_allocator = allocator;
		var i = 0;
		var tVec;
		var tMat;
		this.m_constraintCount = contactCount;
		while (this.m_constraints.length < this.m_constraintCount) {
			this.m_constraints[this.m_constraints.length] = new b2ContactConstraint();
		}
		for (i = 0;
		i < contactCount; ++i) {
			contact = contacts[i];
			var fixtureA = contact.m_fixtureA;
			var fixtureB = contact.m_fixtureB;
			var shapeA = fixtureA.m_shape;
			var shapeB = fixtureB.m_shape;
			var radiusA = shapeA.m_radius;
			var radiusB = shapeB.m_radius;
			var bodyA = fixtureA.m_body;
			var bodyB = fixtureB.m_body;
			var manifold = contact.GetManifold();
			var friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
			var restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
			var vAX = bodyA.m_linearVelocity.x;
			var vAY = bodyA.m_linearVelocity.y;
			var vBX = bodyB.m_linearVelocity.x;
			var vBY = bodyB.m_linearVelocity.y;
			var wA = bodyA.m_angularVelocity;
			var wB = bodyB.m_angularVelocity;
			b2Settings.b2Assert(manifold.m_pointCount > 0);
			b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
			var normalX = b2ContactSolver.s_worldManifold.m_normal.x;
			var normalY = b2ContactSolver.s_worldManifold.m_normal.y;
			var cc = this.m_constraints[i];
			cc.bodyA = bodyA;
			cc.bodyB = bodyB;
			cc.manifold = manifold;
			cc.normal.x = normalX;
			cc.normal.y = normalY;
			cc.pointCount = manifold.m_pointCount;
			cc.friction = friction;
			cc.restitution = restitution;
			cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
			cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
			cc.localPoint.x = manifold.m_localPoint.x;
			cc.localPoint.y = manifold.m_localPoint.y;
			cc.radius = radiusA + radiusB;
			cc.type = manifold.m_type;
			for (var k = 0; k < cc.pointCount; ++k) {
				var cp = manifold.m_points[k];
				var ccp = cc.points[k];
				ccp.normalImpulse = cp.m_normalImpulse;
				ccp.tangentImpulse = cp.m_tangentImpulse;
				ccp.localPoint.SetV(cp.m_localPoint);
				var rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
				var rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
				var rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
				var rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
				var rnA = rAX * normalY - rAY * normalX;
				var rnB = rBX * normalY - rBY * normalX;
				rnA *= rnA;
				rnB *= rnB;
				var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
				ccp.normalMass = 1.0 / kNormal;
				var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
				kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
				ccp.equalizedMass = 1.0 / kEqualized;
				var tangentX = normalY;
				var tangentY = (-normalX);
				var rtA = rAX * tangentY - rAY * tangentX;
				var rtB = rBX * tangentY - rBY * tangentX;
				rtA *= rtA;
				rtB *= rtB;
				var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
				ccp.tangentMass = 1.0 / kTangent;
				ccp.velocityBias = 0.0;
				var tX = vBX + ((-wB * rBY)) - vAX - ((-wA * rAY));
				var tY = vBY + (wB * rBX) - vAY - (wA * rAX);
				var vRel = cc.normal.x * tX + cc.normal.y * tY;
				if (vRel < (-b2Settings.b2_velocityThreshold)) {
					ccp.velocityBias += (-cc.restitution * vRel);
				}
			}
			if (cc.pointCount == 2) {
				var ccp1 = cc.points[0];
				var ccp2 = cc.points[1];
				var invMassA = bodyA.m_invMass;
				var invIA = bodyA.m_invI;
				var invMassB = bodyB.m_invMass;
				var invIB = bodyB.m_invI;
				var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
				var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
				var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
				var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
				var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
				var k_maxConditionNumber = 100.0;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
					cc.K.col1.Set(k11, k12);
					cc.K.col2.Set(k12, k22);
					cc.K.GetInverse(cc.normalMass);
				}
				else {
					cc.pointCount = 1;
				}
			}
		}
	}
	b2ContactSolver.prototype.InitVelocityConstraints = function (step) {
		var tVec;
		var tVec2;
		var tMat;
		for (var i = 0; i < this.m_constraintCount; ++i) {
			var c = this.m_constraints[i];
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			var invMassA = bodyA.m_invMass;
			var invIA = bodyA.m_invI;
			var invMassB = bodyB.m_invMass;
			var invIB = bodyB.m_invI;
			var normalX = c.normal.x;
			var normalY = c.normal.y;
			var tangentX = normalY;
			var tangentY = (-normalX);
			var tX = 0;
			var j = 0;
			var tCount = 0;
			if (step.warmStarting) {
				tCount = c.pointCount;
				for (j = 0;
				j < tCount; ++j) {
					var ccp = c.points[j];
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;
					var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
					var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
					bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
					bodyA.m_linearVelocity.x -= invMassA * PX;
					bodyA.m_linearVelocity.y -= invMassA * PY;
					bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
					bodyB.m_linearVelocity.x += invMassB * PX;
					bodyB.m_linearVelocity.y += invMassB * PY;
				}
			}
			else {
				tCount = c.pointCount;
				for (j = 0;
				j < tCount; ++j) {
					var ccp2 = c.points[j];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
				}
			}
		}
	}
	b2ContactSolver.prototype.SolveVelocityConstraints = function () {
		var j = 0;
		var ccp;
		var rAX = 0;
		var rAY = 0;
		var rBX = 0;
		var rBY = 0;
		var dvX = 0;
		var dvY = 0;
		var vn = 0;
		var vt = 0;
		var lambda = 0;
		var maxFriction = 0;
		var newImpulse = 0;
		var PX = 0;
		var PY = 0;
		var dX = 0;
		var dY = 0;
		var P1X = 0;
		var P1Y = 0;
		var P2X = 0;
		var P2Y = 0;
		var tMat;
		var tVec;
		for (var i = 0; i < this.m_constraintCount; ++i) {
			var c = this.m_constraints[i];
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			var wA = bodyA.m_angularVelocity;
			var wB = bodyB.m_angularVelocity;
			var vA = bodyA.m_linearVelocity;
			var vB = bodyB.m_linearVelocity;
			var invMassA = bodyA.m_invMass;
			var invIA = bodyA.m_invI;
			var invMassB = bodyB.m_invMass;
			var invIB = bodyB.m_invI;
			var normalX = c.normal.x;
			var normalY = c.normal.y;
			var tangentX = normalY;
			var tangentY = (-normalX);
			var friction = c.friction;
			var tX = 0;
			for (j = 0;
			j < c.pointCount; j++) {
				ccp = c.points[j];
				dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
				dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
				vt = dvX * tangentX + dvY * tangentY;
				lambda = ccp.tangentMass * (-vt);
				maxFriction = friction * ccp.normalImpulse;
				newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, (-maxFriction), maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;
				PX = lambda * tangentX;
				PY = lambda * tangentY;
				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
				ccp.tangentImpulse = newImpulse;
			}
			var tCount = parseInt(c.pointCount);
			if (c.pointCount == 1) {
				ccp = c.points[0];
				dvX = vB.x + ((-wB * ccp.rB.y)) - vA.x - ((-wA * ccp.rA.y));
				dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);
				vn = dvX * normalX + dvY * normalY;
				lambda = (-ccp.normalMass * (vn - ccp.velocityBias));
				newImpulse = ccp.normalImpulse + lambda;
				newImpulse = newImpulse > 0 ? newImpulse : 0.0;
				lambda = newImpulse - ccp.normalImpulse;
				PX = lambda * normalX;
				PY = lambda * normalY;
				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
				ccp.normalImpulse = newImpulse;
			}
			else {
				var cp1 = c.points[0];
				var cp2 = c.points[1];
				var aX = cp1.normalImpulse;
				var aY = cp2.normalImpulse;
				var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
				var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
				var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
				var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
				var vn1 = dv1X * normalX + dv1Y * normalY;
				var vn2 = dv2X * normalX + dv2Y * normalY;
				var bX = vn1 - cp1.velocityBias;
				var bY = vn2 - cp2.velocityBias;
				tMat = c.K;
				bX -= tMat.col1.x * aX + tMat.col2.x * aY;
				bY -= tMat.col1.y * aX + tMat.col2.y * aY;
				var k_errorTol = 0.001;
				for (;;) {
					tMat = c.normalMass;
					var xX = (-(tMat.col1.x * bX + tMat.col2.x * bY));
					var xY = (-(tMat.col1.y * bX + tMat.col2.y * bY));
					if (xX >= 0.0 && xY >= 0.0) {
						dX = xX - aX;
						dY = xY - aY;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						break;
					}
					xX = (-cp1.normalMass * bX);
					xY = 0.0;
					vn1 = 0.0;
					vn2 = c.K.col1.y * xX + bY;
					if (xX >= 0.0 && vn2 >= 0.0) {
						dX = xX - aX;
						dY = xY - aY;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						break;
					}
					xX = 0.0;
					xY = (-cp2.normalMass * bY);
					vn1 = c.K.col2.x * xY + bX;
					vn2 = 0.0;
					if (xY >= 0.0 && vn1 >= 0.0) {
						dX = xX - aX;
						dY = xY - aY;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						break;
					}
					xX = 0.0;
					xY = 0.0;
					vn1 = bX;
					vn2 = bY;
					if (vn1 >= 0.0 && vn2 >= 0.0) {
						dX = xX - aX;
						dY = xY - aY;
						P1X = dX * normalX;
						P1Y = dX * normalY;
						P2X = dY * normalX;
						P2Y = dY * normalY;
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						break;
					}
					break;
				}
			}
			bodyA.m_angularVelocity = wA;
			bodyB.m_angularVelocity = wB;
		}
	}
	b2ContactSolver.prototype.FinalizeVelocityConstraints = function () {
		for (var i = 0; i < this.m_constraintCount; ++i) {
			var c = this.m_constraints[i];
			var m = c.manifold;
			for (var j = 0; j < c.pointCount; ++j) {
				var point1 = m.m_points[j];
				var point2 = c.points[j];
				point1.m_normalImpulse = point2.normalImpulse;
				point1.m_tangentImpulse = point2.tangentImpulse;
			}
		}
	}
	b2ContactSolver.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var minSeparation = 0.0;
		for (var i = 0; i < this.m_constraintCount; i++) {
			var c = this.m_constraints[i];
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			var invMassA = bodyA.m_mass * bodyA.m_invMass;
			var invIA = bodyA.m_mass * bodyA.m_invI;
			var invMassB = bodyB.m_mass * bodyB.m_invMass;
			var invIB = bodyB.m_mass * bodyB.m_invI;
			b2ContactSolver.s_psm.Initialize(c);
			var normal = b2ContactSolver.s_psm.m_normal;
			for (var j = 0; j < c.pointCount; j++) {
				var ccp = c.points[j];
				var point = b2ContactSolver.s_psm.m_points[j];
				var separation = b2ContactSolver.s_psm.m_separations[j];
				var rAX = point.x - bodyA.m_sweep.c.x;
				var rAY = point.y - bodyA.m_sweep.c.y;
				var rBX = point.x - bodyB.m_sweep.c.x;
				var rBY = point.y - bodyB.m_sweep.c.y;
				minSeparation = minSeparation < separation ? minSeparation : separation;
				var C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), (-b2Settings.b2_maxLinearCorrection), 0.0);
				var impulse = (-ccp.equalizedMass * C);
				var PX = impulse * normal.x;
				var PY = impulse * normal.y;bodyA.m_sweep.c.x -= invMassA * PX;
				bodyA.m_sweep.c.y -= invMassA * PY;
				bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
				bodyA.SynchronizeTransform();
				bodyB.m_sweep.c.x += invMassB * PX;
				bodyB.m_sweep.c.y += invMassB * PY;
				bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
				bodyB.SynchronizeTransform();
			}
		}
		return minSeparation > (-1.5 * b2Settings.b2_linearSlop);
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold = new b2WorldManifold();
		Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new b2PositionSolverManifold();
	});
	Box2D.inherit(b2EdgeAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
	b2EdgeAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2EdgeAndCircleContact.b2EdgeAndCircleContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2EdgeAndCircleContact.Create = function (allocator) {
		return new b2EdgeAndCircleContact();
	}
	b2EdgeAndCircleContact.Destroy = function (contact, allocator) {}
	b2EdgeAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		this.__super.Reset.call(this, fixtureA, fixtureB);
	}
	b2EdgeAndCircleContact.prototype.Evaluate = function () {
		var bA = this.m_fixtureA.GetBody();
		var bB = this.m_fixtureB.GetBody();
		this.b2CollideEdgeAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2EdgeShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
	}
	b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle = function (manifold, edge, xf1, circle, xf2) {}
	Box2D.inherit(b2NullContact, Box2D.Dynamics.Contacts.b2Contact);
	b2NullContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2NullContact.b2NullContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2NullContact.prototype.b2NullContact = function () {
		this.__super.b2Contact.call(this);
	}
	b2NullContact.prototype.Evaluate = function () {}
	Box2D.inherit(b2PolyAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
	b2PolyAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2PolyAndCircleContact.b2PolyAndCircleContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2PolyAndCircleContact.Create = function (allocator) {
		return new b2PolyAndCircleContact();
	}
	b2PolyAndCircleContact.Destroy = function (contact, allocator) {}
	b2PolyAndCircleContact.prototype.Reset = function (fixtureA, fixtureB) {
		this.__super.Reset.call(this, fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
	}
	b2PolyAndCircleContact.prototype.Evaluate = function () {
		var bA = this.m_fixtureA.m_body;
		var bB = this.m_fixtureB.m_body;
		b2Collision.CollidePolygonAndCircle(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
	}
	Box2D.inherit(b2PolyAndEdgeContact, Box2D.Dynamics.Contacts.b2Contact);
	b2PolyAndEdgeContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2PolyAndEdgeContact.b2PolyAndEdgeContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2PolyAndEdgeContact.Create = function (allocator) {
		return new b2PolyAndEdgeContact();
	}
	b2PolyAndEdgeContact.Destroy = function (contact, allocator) {}
	b2PolyAndEdgeContact.prototype.Reset = function (fixtureA, fixtureB) {
		this.__super.Reset.call(this, fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_edgeShape);
	}
	b2PolyAndEdgeContact.prototype.Evaluate = function () {
		var bA = this.m_fixtureA.GetBody();
		var bB = this.m_fixtureB.GetBody();
		this.b2CollidePolyAndEdge(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2EdgeShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
	}
	b2PolyAndEdgeContact.prototype.b2CollidePolyAndEdge = function (manifold, polygon, xf1, edge, xf2) {}
	Box2D.inherit(b2PolygonContact, Box2D.Dynamics.Contacts.b2Contact);
	b2PolygonContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
	b2PolygonContact.b2PolygonContact = function () {
		Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
	};
	b2PolygonContact.Create = function (allocator) {
		return new b2PolygonContact();
	}
	b2PolygonContact.Destroy = function (contact, allocator) {}
	b2PolygonContact.prototype.Reset = function (fixtureA, fixtureB) {
		this.__super.Reset.call(this, fixtureA, fixtureB);
	}
	b2PolygonContact.prototype.Evaluate = function () {
		var bA = this.m_fixtureA.GetBody();
		var bB = this.m_fixtureB.GetBody();
		b2Collision.CollidePolygons(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2PolygonShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
	}
	b2PositionSolverManifold.b2PositionSolverManifold = function () {};
	b2PositionSolverManifold.prototype.b2PositionSolverManifold = function () {
		this.m_normal = new b2Vec2();
		this.m_separations = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
		this.m_points = new Vector(b2Settings.b2_maxManifoldPoints);
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.m_points[i] = new b2Vec2();
		}
	}
	b2PositionSolverManifold.prototype.Initialize = function (cc) {
		b2Settings.b2Assert(cc.pointCount > 0);
		var i = 0;
		var clipPointX = 0;
		var clipPointY = 0;
		var tMat;
		var tVec;
		var planePointX = 0;
		var planePointY = 0;
		switch (cc.type) {
		case b2Manifold.e_circles:
			{
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				var pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				var pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.points[0].localPoint;
				var pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				var pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				var dX = pointBX - pointAX;
				var dY = pointBY - pointAY;
				var d2 = dX * dX + dY * dY;
				if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
					var d = Math.sqrt(d2);
					this.m_normal.x = dX / d;
					this.m_normal.y = dY / d;
				}
				else {
					this.m_normal.x = 1.0;
					this.m_normal.y = 0.0;
				}
				this.m_points[0].x = 0.5 * (pointAX + pointBX);
				this.m_points[0].y = 0.5 * (pointAY + pointBY);
				this.m_separations[0] = dX * this.m_normal.x + dY * this.m_normal.y - cc.radius;
			}
			break;
		case b2Manifold.e_faceA:
			{
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPlaneNormal;
				this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				tMat = cc.bodyB.m_xf.R;
				for (i = 0;
				i < cc.pointCount; ++i) {
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
					this.m_points[i].x = clipPointX;
					this.m_points[i].y = clipPointY;
				}
			}
			break;
		case b2Manifold.e_faceB:
			{
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPlaneNormal;
				this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				tMat = cc.bodyA.m_xf.R;
				for (i = 0;
				i < cc.pointCount; ++i) {
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
					this.m_points[i].Set(clipPointX, clipPointY);
				}
				this.m_normal.x *= (-1);
				this.m_normal.y *= (-1);
			}
			break;
		}
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA = new b2Vec2();
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB = new b2Vec2();
	});
})();
(function () {
	
	Box2D.inherit(b2BuoyancyController, Box2D.Dynamics.Controllers.b2Controller);
	b2BuoyancyController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;
	b2BuoyancyController.b2BuoyancyController = function () {
		Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
		this.normal = new b2Vec2(0, (-1));
		this.offset = 0;
		this.density = 0;
		this.velocity = new b2Vec2(0, 0);
		this.linearDrag = 2;
		this.angularDrag = 1;
		this.useDensity = false;
		this.useWorldGravity = true;
		this.gravity = null;
	};
	b2BuoyancyController.prototype.Step = function (step) {
		if (!this.m_bodyList) return;
		if (this.useWorldGravity) {
			this.gravity = this.GetWorld().GetGravity().Copy();
		}
		for (var i = this.m_bodyList; i; i = i.nextBody) {
			var body = i.body;
			if (body.IsAwake() == false) {
				continue;
			}
			var areac = new b2Vec2();
			var massc = new b2Vec2();
			var area = 0.0;
			var mass = 0.0;
			for (var fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
				var sc = new b2Vec2();
				var sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
				area += sarea;
				areac.x += sarea * sc.x;
				areac.y += sarea * sc.y;
				var shapeDensity = 0;
				if (this.useDensity) {
					shapeDensity = 1;
				}
				else {
					shapeDensity = 1;
				}
				mass += sarea * shapeDensity;
				massc.x += sarea * sc.x * shapeDensity;
				massc.y += sarea * sc.y * shapeDensity;
			}
			areac.x /= area;
			areac.y /= area;
			massc.x /= mass;
			massc.y /= mass;
			if (area < Number.MIN_VALUE) continue;
			var buoyancyForce = this.gravity.GetNegative();
			buoyancyForce.Multiply(this.density * area);
			body.ApplyForce(buoyancyForce, massc);
			var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
			dragForce.Subtract(this.velocity);
			dragForce.Multiply((-this.linearDrag * area));
			body.ApplyForce(dragForce, areac);
			body.ApplyTorque((-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag));
		}
	}
	b2BuoyancyController.prototype.Draw = function (debugDraw) {
		var r = 1000;
		var p1 = new b2Vec2();
		var p2 = new b2Vec2();
		p1.x = this.normal.x * this.offset + this.normal.y * r;
		p1.y = this.normal.y * this.offset - this.normal.x * r;
		p2.x = this.normal.x * this.offset - this.normal.y * r;
		p2.y = this.normal.y * this.offset + this.normal.x * r;
		var color = new b2Color(0, 0, 1);
		debugDraw.DrawSegment(p1, p2, color);
	}
	Box2D.inherit(b2ConstantAccelController, Box2D.Dynamics.Controllers.b2Controller);
	b2ConstantAccelController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;
	b2ConstantAccelController.b2ConstantAccelController = function () {
		Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
		this.A = new b2Vec2(0, 0);
	};
	b2ConstantAccelController.prototype.Step = function (step) {
		var smallA = new b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
		for (var i = this.m_bodyList; i; i = i.nextBody) {
			var body = i.body;
			if (!body.IsAwake()) continue;
			body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y));
		}
	}
	Box2D.inherit(b2ConstantForceController, Box2D.Dynamics.Controllers.b2Controller);
	b2ConstantForceController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;
	b2ConstantForceController.b2ConstantForceController = function () {
		Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
		this.F = new b2Vec2(0, 0);
	};
	b2ConstantForceController.prototype.Step = function (step) {
		for (var i = this.m_bodyList; i; i = i.nextBody) {
			var body = i.body;
			if (!body.IsAwake()) continue;
			body.ApplyForce(this.F, body.GetWorldCenter());
		}
	}
	b2Controller.b2Controller = function () {};
	b2Controller.prototype.Step = function (step) {}
	b2Controller.prototype.Draw = function (debugDraw) {}
	b2Controller.prototype.AddBody = function (body) {
		var edge = new b2ControllerEdge();
		edge.controller = this;
		edge.body = body;
		edge.nextBody = this.m_bodyList;
		edge.prevBody = null;
		this.m_bodyList = edge;
		if (edge.nextBody) edge.nextBody.prevBody = edge;
		this.m_bodyCount++;
		edge.nextController = body.m_controllerList;
		edge.prevController = null;
		body.m_controllerList = edge;
		if (edge.nextController) edge.nextController.prevController = edge;
		body.m_controllerCount++;
	}
	b2Controller.prototype.RemoveBody = function (body) {
		var edge = body.m_controllerList;
		while (edge && edge.controller != this)
		edge = edge.nextController;
		if (edge.prevBody) edge.prevBody.nextBody = edge.nextBody;
		if (edge.nextBody) edge.nextBody.prevBody = edge.prevBody;
		if (edge.nextController) edge.nextController.prevController = edge.prevController;
		if (edge.prevController) edge.prevController.nextController = edge.nextController;
		if (this.m_bodyList == edge) this.m_bodyList = edge.nextBody;
		if (body.m_controllerList == edge) body.m_controllerList = edge.nextController;
		body.m_controllerCount--;
		this.m_bodyCount--;
	}
	b2Controller.prototype.Clear = function () {
		while (this.m_bodyList)
		this.RemoveBody(this.m_bodyList.body);
	}
	b2Controller.prototype.GetNext = function () {
		return this.m_next;
	}
	b2Controller.prototype.GetWorld = function () {
		return this.m_world;
	}
	b2Controller.prototype.GetBodyList = function () {
		return this.m_bodyList;
	}
	b2ControllerEdge.b2ControllerEdge = function () {};
	Box2D.inherit(b2GravityController, Box2D.Dynamics.Controllers.b2Controller);
	b2GravityController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;
	b2GravityController.b2GravityController = function () {
		Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
		this.G = 1;
		this.invSqr = true;
	};
	b2GravityController.prototype.Step = function (step) {
		var i = null;
		var body1 = null;
		var p1 = null;
		var mass1 = 0;
		var j = null;
		var body2 = null;
		var p2 = null;
		var dx = 0;
		var dy = 0;
		var r2 = 0;
		var f = null;
		if (this.invSqr) {
			for (i = this.m_bodyList;
			i; i = i.nextBody) {
				body1 = i.body;
				p1 = body1.GetWorldCenter();
				mass1 = body1.GetMass();
				for (j = this.m_bodyList;
				j != i; j = j.nextBody) {
					body2 = j.body;
					p2 = body2.GetWorldCenter();
					dx = p2.x - p1.x;
					dy = p2.y - p1.y;
					r2 = dx * dx + dy * dy;
					if (r2 < Number.MIN_VALUE) continue;
					f = new b2Vec2(dx, dy);
					f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
					if (body1.IsAwake()) body1.ApplyForce(f, p1);
					f.Multiply((-1));
					if (body2.IsAwake()) body2.ApplyForce(f, p2);
				}
			}
		}
		else {
			for (i = this.m_bodyList;
			i; i = i.nextBody) {
				body1 = i.body;
				p1 = body1.GetWorldCenter();
				mass1 = body1.GetMass();
				for (j = this.m_bodyList;
				j != i; j = j.nextBody) {
					body2 = j.body;
					p2 = body2.GetWorldCenter();
					dx = p2.x - p1.x;
					dy = p2.y - p1.y;
					r2 = dx * dx + dy * dy;
					if (r2 < Number.MIN_VALUE) continue;
					f = new b2Vec2(dx, dy);
					f.Multiply(this.G / r2 * mass1 * body2.GetMass());
					if (body1.IsAwake()) body1.ApplyForce(f, p1);
					f.Multiply((-1));
					if (body2.IsAwake()) body2.ApplyForce(f, p2);
				}
			}
		}
	}
	Box2D.inherit(b2TensorDampingController, Box2D.Dynamics.Controllers.b2Controller);
	b2TensorDampingController.prototype.__super = Box2D.Dynamics.Controllers.b2Controller.prototype;
	b2TensorDampingController.b2TensorDampingController = function () {
		Box2D.Dynamics.Controllers.b2Controller.b2Controller.apply(this, arguments);
		this.T = new b2Mat22();
		this.maxTimestep = 0;
	};
	b2TensorDampingController.prototype.SetAxisAligned = function (xDamping, yDamping) {
		if (xDamping === undefined) xDamping = 0;
		if (yDamping === undefined) yDamping = 0;
		this.T.col1.x = (-xDamping);
		this.T.col1.y = 0;
		this.T.col2.x = 0;
		this.T.col2.y = (-yDamping);
		if (xDamping > 0 || yDamping > 0) {
			this.maxTimestep = 1 / Math.max(xDamping, yDamping);
		}
		else {
			this.maxTimestep = 0;
		}
	}
	b2TensorDampingController.prototype.Step = function (step) {
		var timestep = step.dt;
		if (timestep <= Number.MIN_VALUE) return;
		if (timestep > this.maxTimestep && this.maxTimestep > 0) timestep = this.maxTimestep;
		for (var i = this.m_bodyList; i; i = i.nextBody) {
			var body = i.body;
			if (!body.IsAwake()) {
				continue;
			}
			var damping = body.GetWorldVector(b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
			body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep));
		}
	}
})();
(function () {

	Box2D.inherit(b2DistanceJoint, Box2D.Dynamics.Joints.b2Joint);
	b2DistanceJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2DistanceJoint.b2DistanceJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_u = new b2Vec2();
	};
	b2DistanceJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2DistanceJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2DistanceJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
	}
	b2DistanceJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return 0.0;
	}
	b2DistanceJoint.prototype.GetLength = function () {
		return this.m_length;
	}
	b2DistanceJoint.prototype.SetLength = function (length) {
		if (length === undefined) length = 0;
		this.m_length = length;
	}
	b2DistanceJoint.prototype.GetFrequency = function () {
		return this.m_frequencyHz;
	}
	b2DistanceJoint.prototype.SetFrequency = function (hz) {
		if (hz === undefined) hz = 0;
		this.m_frequencyHz = hz;
	}
	b2DistanceJoint.prototype.GetDampingRatio = function () {
		return this.m_dampingRatio;
	}
	b2DistanceJoint.prototype.SetDampingRatio = function (ratio) {
		if (ratio === undefined) ratio = 0;
		this.m_dampingRatio = ratio;
	}
	b2DistanceJoint.prototype.b2DistanceJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		var tMat;
		var tX = 0;
		var tY = 0;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_length = def.length;
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_impulse = 0.0;
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
	}
	b2DistanceJoint.prototype.InitVelocityConstraints = function (step) {
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
		var length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
		if (length > b2Settings.b2_linearSlop) {
			this.m_u.Multiply(1.0 / length);
		}
		else {
			this.m_u.SetZero();
		}
		var cr1u = (r1X * this.m_u.y - r1Y * this.m_u.x);
		var cr2u = (r2X * this.m_u.y - r2Y * this.m_u.x);
		var invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;
		this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
		if (this.m_frequencyHz > 0.0) {
			var C = length - this.m_length;
			var omega = 2.0 * Math.PI * this.m_frequencyHz;
			var d = 2.0 * this.m_mass * this.m_dampingRatio * omega;
			var k = this.m_mass * omega * omega;
			this.m_gamma = step.dt * (d + step.dt * k);
			this.m_gamma = this.m_gamma != 0.0 ? 1 / this.m_gamma : 0.0;
			this.m_bias = C * step.dt * k * this.m_gamma;
			this.m_mass = invMass + this.m_gamma;
			this.m_mass = this.m_mass != 0.0 ? 1.0 / this.m_mass : 0.0;
		}
		if (step.warmStarting) {
			this.m_impulse *= step.dtRatio;
			var PX = this.m_impulse * this.m_u.x;
			var PY = this.m_impulse * this.m_u.y;
			bA.m_linearVelocity.x -= bA.m_invMass * PX;
			bA.m_linearVelocity.y -= bA.m_invMass * PY;
			bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
			bB.m_linearVelocity.x += bB.m_invMass * PX;
			bB.m_linearVelocity.y += bB.m_invMass * PY;
			bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
		}
		else {
			this.m_impulse = 0.0;
		}
	}
	b2DistanceJoint.prototype.SolveVelocityConstraints = function (step) {
		var tMat;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y));
		var v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
		var v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y));
		var v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
		var Cdot = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y));
		var impulse = (-this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse));
		this.m_impulse += impulse;
		var PX = impulse * this.m_u.x;
		var PY = impulse * this.m_u.y;
		bA.m_linearVelocity.x -= bA.m_invMass * PX;
		bA.m_linearVelocity.y -= bA.m_invMass * PY;
		bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
		bB.m_linearVelocity.x += bB.m_invMass * PX;
		bB.m_linearVelocity.y += bB.m_invMass * PY;
		bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
	}
	b2DistanceJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var tMat;
		if (this.m_frequencyHz > 0.0) {
			return true;
		}
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		var dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
		var length = Math.sqrt(dX * dX + dY * dY);
		dX /= length;
		dY /= length;
		var C = length - this.m_length;
		C = b2Math.Clamp(C, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection);
		var impulse = (-this.m_mass * C);
		this.m_u.Set(dX, dY);
		var PX = impulse * this.m_u.x;
		var PY = impulse * this.m_u.y;
		bA.m_sweep.c.x -= bA.m_invMass * PX;
		bA.m_sweep.c.y -= bA.m_invMass * PY;
		bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
		bB.m_sweep.c.x += bB.m_invMass * PX;
		bB.m_sweep.c.y += bB.m_invMass * PY;
		bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		return b2Math.Abs(C) < b2Settings.b2_linearSlop;
	}
	Box2D.inherit(b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2DistanceJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2DistanceJointDef.b2DistanceJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2DistanceJointDef.prototype.b2DistanceJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_distanceJoint;
		this.length = 1.0;
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}
	b2DistanceJointDef.prototype.Initialize = function (bA, bB, anchorA, anchorB) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
		var dX = anchorB.x - anchorA.x;
		var dY = anchorB.y - anchorA.y;
		this.length = Math.sqrt(dX * dX + dY * dY);
		this.frequencyHz = 0.0;
		this.dampingRatio = 0.0;
	}
	Box2D.inherit(b2FrictionJoint, Box2D.Dynamics.Joints.b2Joint);
	b2FrictionJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2FrictionJoint.b2FrictionJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_localAnchorA = new b2Vec2();
		this.m_localAnchorB = new b2Vec2();
		this.m_linearMass = new b2Mat22();
		this.m_linearImpulse = new b2Vec2();
	};
	b2FrictionJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
	b2FrictionJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
	b2FrictionJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	}
	b2FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return inv_dt * this.m_angularImpulse;
	}
	b2FrictionJoint.prototype.SetMaxForce = function (force) {
		if (force === undefined) force = 0;
		this.m_maxForce = force;
	}
	b2FrictionJoint.prototype.GetMaxForce = function () {
		return this.m_maxForce;
	}
	b2FrictionJoint.prototype.SetMaxTorque = function (torque) {
		if (torque === undefined) torque = 0;
		this.m_maxTorque = torque;
	}
	b2FrictionJoint.prototype.GetMaxTorque = function () {
		return this.m_maxTorque;
	}
	b2FrictionJoint.prototype.b2FrictionJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		this.m_linearMass.SetZero();
		this.m_angularMass = 0.0;
		this.m_linearImpulse.SetZero();
		this.m_angularImpulse = 0.0;
		this.m_maxForce = def.maxForce;
		this.m_maxTorque = def.maxTorque;
	}
	b2FrictionJoint.prototype.InitVelocityConstraints = function (step) {
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		var mA = bA.m_invMass;
		var mB = bB.m_invMass;
		var iA = bA.m_invI;
		var iB = bB.m_invI;
		var K = new b2Mat22();
		K.col1.x = mA + mB;
		K.col2.x = 0.0;
		K.col1.y = 0.0;
		K.col2.y = mA + mB;
		K.col1.x += iA * rAY * rAY;
		K.col2.x += (-iA * rAX * rAY);
		K.col1.y += (-iA * rAX * rAY);
		K.col2.y += iA * rAX * rAX;
		K.col1.x += iB * rBY * rBY;
		K.col2.x += (-iB * rBX * rBY);
		K.col1.y += (-iB * rBX * rBY);
		K.col2.y += iB * rBX * rBX;
		K.GetInverse(this.m_linearMass);
		this.m_angularMass = iA + iB;
		if (this.m_angularMass > 0.0) {
			this.m_angularMass = 1.0 / this.m_angularMass;
		}
		if (step.warmStarting) {
			this.m_linearImpulse.x *= step.dtRatio;
			this.m_linearImpulse.y *= step.dtRatio;
			this.m_angularImpulse *= step.dtRatio;
			var P = this.m_linearImpulse;
			bA.m_linearVelocity.x -= mA * P.x;
			bA.m_linearVelocity.y -= mA * P.y;
			bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);
			bB.m_linearVelocity.x += mB * P.x;
			bB.m_linearVelocity.y += mB * P.y;
			bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
		}
		else {
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0.0;
		}
	}
	b2FrictionJoint.prototype.SolveVelocityConstraints = function (step) {
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var vA = bA.m_linearVelocity;
		var wA = bA.m_angularVelocity;
		var vB = bB.m_linearVelocity;
		var wB = bB.m_angularVelocity;
		var mA = bA.m_invMass;
		var mB = bB.m_invMass;
		var iA = bA.m_invI;
		var iB = bB.m_invI;
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		var maxImpulse = 0; {
			var Cdot = wB - wA;
			var impulse = (-this.m_angularMass * Cdot);
			var oldImpulse = this.m_angularImpulse;
			maxImpulse = step.dt * this.m_maxTorque;
			this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, (-maxImpulse), maxImpulse);
			impulse = this.m_angularImpulse - oldImpulse;
			wA -= iA * impulse;
			wB += iB * impulse;
		} {
			var CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
			var CdotY = vB.y + wB * rBX - vA.y - wA * rAX;
			var impulseV = b2Math.MulMV(this.m_linearMass, new b2Vec2((-CdotX), (-CdotY)));
			var oldImpulseV = this.m_linearImpulse.Copy();
			this.m_linearImpulse.Add(impulseV);
			maxImpulse = step.dt * this.m_maxForce;
			if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
				this.m_linearImpulse.Normalize();
				this.m_linearImpulse.Multiply(maxImpulse);
			}
			impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);
			vA.x -= mA * impulseV.x;
			vA.y -= mA * impulseV.y;
			wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);
			vB.x += mB * impulseV.x;
			vB.y += mB * impulseV.y;
			wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
		}
		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	}
	b2FrictionJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		return true;
	}
	Box2D.inherit(b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2FrictionJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2FrictionJointDef.b2FrictionJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2FrictionJointDef.prototype.b2FrictionJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_frictionJoint;
		this.maxForce = 0.0;
		this.maxTorque = 0.0;
	}
	b2FrictionJointDef.prototype.Initialize = function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
	}
	Box2D.inherit(b2GearJoint, Box2D.Dynamics.Joints.b2Joint);
	b2GearJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2GearJoint.b2GearJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_groundAnchor1 = new b2Vec2();
		this.m_groundAnchor2 = new b2Vec2();
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_J = new b2Jacobian();
	};
	b2GearJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2GearJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2GearJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
	}
	b2GearJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		var tMat = this.m_bodyB.m_xf.R;
		var rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
		var rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
		var tX = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		var PX = this.m_impulse * this.m_J.linearB.x;
		var PY = this.m_impulse * this.m_J.linearB.y;
		return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
	}
	b2GearJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
	b2GearJoint.prototype.SetRatio = function (ratio) {
		if (ratio === undefined) ratio = 0;
		this.m_ratio = ratio;
	}
	b2GearJoint.prototype.b2GearJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		var type1 = parseInt(def.joint1.m_type);
		var type2 = parseInt(def.joint2.m_type);
		this.m_revolute1 = null;
		this.m_prismatic1 = null;
		this.m_revolute2 = null;
		this.m_prismatic2 = null;
		var coordinate1 = 0;
		var coordinate2 = 0;
		this.m_ground1 = def.joint1.GetBodyA();
		this.m_bodyA = def.joint1.GetBodyB();
		if (type1 == b2Joint.e_revoluteJoint) {
			this.m_revolute1 = (def.joint1 instanceof b2RevoluteJoint ? def.joint1 : null);
			this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1);
			this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2);
			coordinate1 = this.m_revolute1.GetJointAngle();
		}
		else {
			this.m_prismatic1 = (def.joint1 instanceof b2PrismaticJoint ? def.joint1 : null);
			this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1);
			this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2);
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		this.m_ground2 = def.joint2.GetBodyA();
		this.m_bodyB = def.joint2.GetBodyB();
		if (type2 == b2Joint.e_revoluteJoint) {
			this.m_revolute2 = (def.joint2 instanceof b2RevoluteJoint ? def.joint2 : null);
			this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1);
			this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2);
			coordinate2 = this.m_revolute2.GetJointAngle();
		}
		else {
			this.m_prismatic2 = (def.joint2 instanceof b2PrismaticJoint ? def.joint2 : null);
			this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1);
			this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2);
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}
		this.m_ratio = def.ratio;
		this.m_constant = coordinate1 + this.m_ratio * coordinate2;
		this.m_impulse = 0.0;
	}
	b2GearJoint.prototype.InitVelocityConstraints = function (step) {
		var g1 = this.m_ground1;
		var g2 = this.m_ground2;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var ugX = 0;
		var ugY = 0;
		var rX = 0;
		var rY = 0;
		var tMat;
		var tVec;
		var crug = 0;
		var tX = 0;
		var K = 0.0;
		this.m_J.SetZero();
		if (this.m_revolute1) {
			this.m_J.angularA = (-1.0);
			K += bA.m_invI;
		}
		else {
			tMat = g1.m_xf.R;
			tVec = this.m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			tMat = bA.m_xf.R;
			rX = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			rY = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			crug = rX * ugY - rY * ugX;
			this.m_J.linearA.Set((-ugX), (-ugY));
			this.m_J.angularA = (-crug);
			K += bA.m_invMass + bA.m_invI * crug * crug;
		}
		if (this.m_revolute2) {
			this.m_J.angularB = (-this.m_ratio);
			K += this.m_ratio * this.m_ratio * bB.m_invI;
		}
		else {
			tMat = g2.m_xf.R;
			tVec = this.m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			tMat = bB.m_xf.R;
			rX = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			rY = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			crug = rX * ugY - rY * ugX;
			this.m_J.linearB.Set((-this.m_ratio * ugX), (-this.m_ratio * ugY));
			this.m_J.angularB = (-this.m_ratio * crug);
			K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
		}
		this.m_mass = K > 0.0 ? 1.0 / K : 0.0;
		if (step.warmStarting) {
			bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
			bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;
			bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
			bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
			bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;
			bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
		}
		else {
			this.m_impulse = 0.0;
		}
	}
	b2GearJoint.prototype.SolveVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var Cdot = this.m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity);
		var impulse = (-this.m_mass * Cdot);
		this.m_impulse += impulse;
		bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;
		bA.m_angularVelocity += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;
		bB.m_angularVelocity += bB.m_invI * impulse * this.m_J.angularB;
	}
	b2GearJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var linearError = 0.0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var coordinate1 = 0;
		var coordinate2 = 0;
		if (this.m_revolute1) {
			coordinate1 = this.m_revolute1.GetJointAngle();
		}
		else {
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		if (this.m_revolute2) {
			coordinate2 = this.m_revolute2.GetJointAngle();
		}
		else {
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}
		var C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);
		var impulse = (-this.m_mass * C);
		bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;
		bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;
		bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		return linearError < b2Settings.b2_linearSlop;
	}
	Box2D.inherit(b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2GearJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2GearJointDef.b2GearJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
	};
	b2GearJointDef.prototype.b2GearJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_gearJoint;
		this.joint1 = null;
		this.joint2 = null;
		this.ratio = 1.0;
	}
	b2Jacobian.b2Jacobian = function () {
		this.linearA = new b2Vec2();
		this.linearB = new b2Vec2();
	};
	b2Jacobian.prototype.SetZero = function () {
		this.linearA.SetZero();
		this.angularA = 0.0;
		this.linearB.SetZero();
		this.angularB = 0.0;
	}
	b2Jacobian.prototype.Set = function (x1, a1, x2, a2) {
		if (a1 === undefined) a1 = 0;
		if (a2 === undefined) a2 = 0;
		this.linearA.SetV(x1);
		this.angularA = a1;
		this.linearB.SetV(x2);
		this.angularB = a2;
	}
	b2Jacobian.prototype.Compute = function (x1, a1, x2, a2) {
		if (a1 === undefined) a1 = 0;
		if (a2 === undefined) a2 = 0;
		return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
	}
	b2Joint.b2Joint = function () {
		this.m_edgeA = new b2JointEdge();
		this.m_edgeB = new b2JointEdge();
		this.m_localCenterA = new b2Vec2();
		this.m_localCenterB = new b2Vec2();
	};
	b2Joint.prototype.GetType = function () {
		return this.m_type;
	}
	b2Joint.prototype.GetAnchorA = function () {
		return null;
	}
	b2Joint.prototype.GetAnchorB = function () {
		return null;
	}
	b2Joint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return null;
	}
	b2Joint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return 0.0;
	}
	b2Joint.prototype.GetBodyA = function () {
		return this.m_bodyA;
	}
	b2Joint.prototype.GetBodyB = function () {
		return this.m_bodyB;
	}
	b2Joint.prototype.GetNext = function () {
		return this.m_next;
	}
	b2Joint.prototype.GetUserData = function () {
		return this.m_userData;
	}
	b2Joint.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
	b2Joint.prototype.IsActive = function () {
		return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
	}
	b2Joint.Create = function (def, allocator) {
		var joint = null;
		switch (def.type) {
		case b2Joint.e_distanceJoint:
			{
				joint = new b2DistanceJoint((def instanceof b2DistanceJointDef ? def : null));
			}
			break;
		case b2Joint.e_mouseJoint:
			{
				joint = new b2MouseJoint((def instanceof b2MouseJointDef ? def : null));
			}
			break;
		case b2Joint.e_prismaticJoint:
			{
				joint = new b2PrismaticJoint((def instanceof b2PrismaticJointDef ? def : null));
			}
			break;
		case b2Joint.e_revoluteJoint:
			{
				joint = new b2RevoluteJoint((def instanceof b2RevoluteJointDef ? def : null));
			}
			break;
		case b2Joint.e_pulleyJoint:
			{
				joint = new b2PulleyJoint((def instanceof b2PulleyJointDef ? def : null));
			}
			break;
		case b2Joint.e_gearJoint:
			{
				joint = new b2GearJoint((def instanceof b2GearJointDef ? def : null));
			}
			break;
		case b2Joint.e_lineJoint:
			{
				joint = new b2LineJoint((def instanceof b2LineJointDef ? def : null));
			}
			break;
		case b2Joint.e_weldJoint:
			{
				joint = new b2WeldJoint((def instanceof b2WeldJointDef ? def : null));
			}
			break;
		case b2Joint.e_frictionJoint:
			{
				joint = new b2FrictionJoint((def instanceof b2FrictionJointDef ? def : null));
			}
			break;
		default:
			break;
		}
		return joint;
	}
	b2Joint.Destroy = function (joint, allocator) {}
	b2Joint.prototype.b2Joint = function (def) {
		b2Settings.b2Assert(def.bodyA != def.bodyB);
		this.m_type = def.type;
		this.m_prev = null;
		this.m_next = null;
		this.m_bodyA = def.bodyA;
		this.m_bodyB = def.bodyB;
		this.m_collideConnected = def.collideConnected;
		this.m_islandFlag = false;
		this.m_userData = def.userData;
	}
	b2Joint.prototype.InitVelocityConstraints = function (step) {}
	b2Joint.prototype.SolveVelocityConstraints = function (step) {}
	b2Joint.prototype.FinalizeVelocityConstraints = function () {}
	b2Joint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		return false;
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0;
		Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1;
		Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2;
		Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3;
		Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4;
		Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5;
		Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6;
		Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7;
		Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8;
		Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9;
		Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0;
		Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1;
		Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2;
		Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3;
	});
	b2JointDef.b2JointDef = function () {};
	b2JointDef.prototype.b2JointDef = function () {
		this.type = b2Joint.e_unknownJoint;
		this.userData = null;
		this.bodyA = null;
		this.bodyB = null;
		this.collideConnected = false;
	}
	b2JointEdge.b2JointEdge = function () {};
	Box2D.inherit(b2LineJoint, Box2D.Dynamics.Joints.b2Joint);
	b2LineJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2LineJoint.b2LineJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_localXAxis1 = new b2Vec2();
		this.m_localYAxis1 = new b2Vec2();
		this.m_axis = new b2Vec2();
		this.m_perp = new b2Vec2();
		this.m_K = new b2Mat22();
		this.m_impulse = new b2Vec2();
	};
	b2LineJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2LineJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2LineJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y));
	}
	b2LineJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return inv_dt * this.m_impulse.y;
	}
	b2LineJoint.prototype.GetJointTranslation = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var p1 = bA.GetWorldPoint(this.m_localAnchor1);
		var p2 = bB.GetWorldPoint(this.m_localAnchor2);
		var dX = p2.x - p1.x;
		var dY = p2.y - p1.y;
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		var translation = axis.x * dX + axis.y * dY;
		return translation;
	}
	b2LineJoint.prototype.GetJointSpeed = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var p1X = bA.m_sweep.c.x + r1X;
		var p1Y = bA.m_sweep.c.y + r1Y;
		var p2X = bB.m_sweep.c.x + r2X;
		var p2Y = bB.m_sweep.c.y + r2Y;
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		var v1 = bA.m_linearVelocity;
		var v2 = bB.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var w2 = bB.m_angularVelocity;
		var speed = (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
		return speed;
	}
	b2LineJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
	b2LineJoint.prototype.EnableLimit = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableLimit = flag;
	}
	b2LineJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerTranslation;
	}
	b2LineJoint.prototype.GetUpperLimit = function () {
		return this.m_upperTranslation;
	}
	b2LineJoint.prototype.SetLimits = function (lower, upper) {
		if (lower === undefined) lower = 0;
		if (upper === undefined) upper = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_lowerTranslation = lower;
		this.m_upperTranslation = upper;
	}
	b2LineJoint.prototype.IsMotorEnabled = function () {
		return this.m_enableMotor;
	}
	b2LineJoint.prototype.EnableMotor = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	}
	b2LineJoint.prototype.SetMotorSpeed = function (speed) {
		if (speed === undefined) speed = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}
	b2LineJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
	b2LineJoint.prototype.SetMaxMotorForce = function (force) {
		if (force === undefined) force = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorForce = force;
	}
	b2LineJoint.prototype.GetMaxMotorForce = function () {
		return this.m_maxMotorForce;
	}
	b2LineJoint.prototype.GetMotorForce = function () {
		return this.m_motorImpulse;
	}
	b2LineJoint.prototype.b2LineJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		var tMat;
		var tX = 0;
		var tY = 0;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_localXAxis1.SetV(def.localAxisA);
		this.m_localYAxis1.x = (-this.m_localXAxis1.y);
		this.m_localYAxis1.y = this.m_localXAxis1.x;
		this.m_impulse.SetZero();
		this.m_motorMass = 0.0;
		this.m_motorImpulse = 0.0;
		this.m_lowerTranslation = def.lowerTranslation;
		this.m_upperTranslation = def.upperTranslation;
		this.m_maxMotorForce = def.maxMotorForce;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = b2Joint.e_inactiveLimit;
		this.m_axis.SetZero();
		this.m_perp.SetZero();
	}
	b2LineJoint.prototype.InitVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var tX = 0;
		this.m_localCenterA.SetV(bA.GetLocalCenter());
		this.m_localCenterB.SetV(bB.GetLocalCenter());
		var xf1 = bA.GetTransform();
		var xf2 = bB.GetTransform();
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
		var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
		var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		var dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
		this.m_invMassA = bA.m_invMass;
		this.m_invMassB = bB.m_invMass;
		this.m_invIA = bA.m_invI;
		this.m_invIB = bB.m_invI; {
			this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
			this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
			this.m_motorMass = this.m_motorMass > Number.MIN_VALUE ? 1.0 / this.m_motorMass : 0.0;
		} {
			this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
			this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
			this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
			var m1 = this.m_invMassA;
			var m2 = this.m_invMassB;
			var i1 = this.m_invIA;
			var i2 = this.m_invIB;
			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
			this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
		}
		if (this.m_enableLimit) {
			var jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointTransition <= this.m_lowerTranslation) {
				if (this.m_limitState != b2Joint.e_atLowerLimit) {
					this.m_limitState = b2Joint.e_atLowerLimit;
					this.m_impulse.y = 0.0;
				}
			}
			else if (jointTransition >= this.m_upperTranslation) {
				if (this.m_limitState != b2Joint.e_atUpperLimit) {
					this.m_limitState = b2Joint.e_atUpperLimit;
					this.m_impulse.y = 0.0;
				}
			}
			else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.y = 0.0;
			}
		}
		else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}
		if (this.m_enableMotor == false) {
			this.m_motorImpulse = 0.0;
		}
		if (step.warmStarting) {
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;
			var PX = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x;
			var PY = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y;
			var L1 = this.m_impulse.x * this.m_s1 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a1;
			var L2 = this.m_impulse.x * this.m_s2 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a2;
			bA.m_linearVelocity.x -= this.m_invMassA * PX;
			bA.m_linearVelocity.y -= this.m_invMassA * PY;
			bA.m_angularVelocity -= this.m_invIA * L1;
			bB.m_linearVelocity.x += this.m_invMassB * PX;
			bB.m_linearVelocity.y += this.m_invMassB * PY;
			bB.m_angularVelocity += this.m_invIB * L2;
		}
		else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
		}
	}
	b2LineJoint.prototype.SolveVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var v1 = bA.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var v2 = bB.m_linearVelocity;
		var w2 = bB.m_angularVelocity;
		var PX = 0;
		var PY = 0;
		var L1 = 0;
		var L2 = 0;
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
			var Cdot = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
			var impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = step.dt * this.m_maxMotorForce;
			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;
			PX = impulse * this.m_axis.x;
			PY = impulse * this.m_axis.y;
			L1 = impulse * this.m_a1;
			L2 = impulse * this.m_a2;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		var Cdot1 = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			var Cdot2 = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
			var f1 = this.m_impulse.Copy();
			var df = this.m_K.Solve(new b2Vec2(), (-Cdot1), (-Cdot2));
			this.m_impulse.Add(df);
			if (this.m_limitState == b2Joint.e_atLowerLimit) {
				this.m_impulse.y = b2Math.Max(this.m_impulse.y, 0.0);
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				this.m_impulse.y = b2Math.Min(this.m_impulse.y, 0.0);
			}
			var b = (-Cdot1) - (this.m_impulse.y - f1.y) * this.m_K.col2.x;
			var f2r = 0;
			if (this.m_K.col1.x != 0.0) {
				f2r = b / this.m_K.col1.x + f1.x;
			}
			else {
				f2r = f1.x;
			}
			this.m_impulse.x = f2r;
			df.x = this.m_impulse.x - f1.x;
			df.y = this.m_impulse.y - f1.y;
			PX = df.x * this.m_perp.x + df.y * this.m_axis.x;
			PY = df.x * this.m_perp.y + df.y * this.m_axis.y;
			L1 = df.x * this.m_s1 + df.y * this.m_a1;
			L2 = df.x * this.m_s2 + df.y * this.m_a2;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		else {
			var df2 = 0;
			if (this.m_K.col1.x != 0.0) {
				df2 = ((-Cdot1)) / this.m_K.col1.x;
			}
			else {
				df2 = 0.0;
			}
			this.m_impulse.x += df2;
			PX = df2 * this.m_perp.x;
			PY = df2 * this.m_perp.y;
			L1 = df2 * this.m_s1;
			L2 = df2 * this.m_s2;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		bA.m_linearVelocity.SetV(v1);
		bA.m_angularVelocity = w1;
		bB.m_linearVelocity.SetV(v2);
		bB.m_angularVelocity = w2;
	}
	b2LineJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var limitC = 0;
		var oldLimitImpulse = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var c1 = bA.m_sweep.c;
		var a1 = bA.m_sweep.a;
		var c2 = bB.m_sweep.c;
		var a2 = bB.m_sweep.a;
		var tMat;
		var tX = 0;
		var m1 = 0;
		var m2 = 0;
		var i1 = 0;
		var i2 = 0;
		var linearError = 0.0;
		var angularError = 0.0;
		var active = false;
		var C2 = 0.0;
		var R1 = b2Mat22.FromAngle(a1);
		var R2 = b2Mat22.FromAngle(a2);
		tMat = R1;
		var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
		var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = R2;
		var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
		var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var dX = c2.x + r2X - c1.x - r1X;
		var dY = c2.y + r2Y - c1.y - r1Y;
		if (this.m_enableLimit) {
			this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
			var translation = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection);
				linearError = b2Math.Abs(translation);
				active = true;
			}
			else if (translation <= this.m_lowerTranslation) {
				C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
				linearError = this.m_lowerTranslation - translation;
				active = true;
			}
			else if (translation >= this.m_upperTranslation) {
				C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
				linearError = translation - this.m_upperTranslation;
				active = true;
			}
		}
		this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
		this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
		this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
		var impulse = new b2Vec2();
		var C1 = this.m_perp.x * dX + this.m_perp.y * dY;
		linearError = b2Math.Max(linearError, b2Math.Abs(C1));
		angularError = 0.0;
		if (active) {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;
			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
			this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
			this.m_K.Solve(impulse, (-C1), (-C2));
		}
		else {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;
			var k11 = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			var impulse1 = 0;
			if (k11 != 0.0) {
				impulse1 = ((-C1)) / k11;
			}
			else {
				impulse1 = 0.0;
			}
			impulse.x = impulse1;
			impulse.y = 0.0;
		}
		var PX = impulse.x * this.m_perp.x + impulse.y * this.m_axis.x;
		var PY = impulse.x * this.m_perp.y + impulse.y * this.m_axis.y;
		var L1 = impulse.x * this.m_s1 + impulse.y * this.m_a1;
		var L2 = impulse.x * this.m_s2 + impulse.y * this.m_a2;
		c1.x -= this.m_invMassA * PX;
		c1.y -= this.m_invMassA * PY;
		a1 -= this.m_invIA * L1;
		c2.x += this.m_invMassB * PX;
		c2.y += this.m_invMassB * PY;
		a2 += this.m_invIB * L2;
		bA.m_sweep.a = a1;
		bB.m_sweep.a = a2;
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
	Box2D.inherit(b2LineJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2LineJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2LineJointDef.b2LineJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
		this.localAxisA = new b2Vec2();
	};
	b2LineJointDef.prototype.b2LineJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_lineJoint;
		this.localAxisA.Set(1.0, 0.0);
		this.enableLimit = false;
		this.lowerTranslation = 0.0;
		this.upperTranslation = 0.0;
		this.enableMotor = false;
		this.maxMotorForce = 0.0;
		this.motorSpeed = 0.0;
	}
	b2LineJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.localAxisA = this.bodyA.GetLocalVector(axis);
	}
	Box2D.inherit(b2MouseJoint, Box2D.Dynamics.Joints.b2Joint);
	b2MouseJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2MouseJoint.b2MouseJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.K = new b2Mat22();
		this.K1 = new b2Mat22();
		this.K2 = new b2Mat22();
		this.m_localAnchor = new b2Vec2();
		this.m_target = new b2Vec2();
		this.m_impulse = new b2Vec2();
		this.m_mass = new b2Mat22();
		this.m_C = new b2Vec2();
	};
	b2MouseJoint.prototype.GetAnchorA = function () {
		return this.m_target;
	}
	b2MouseJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
	}
	b2MouseJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
	b2MouseJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return 0.0;
	}
	b2MouseJoint.prototype.GetTarget = function () {
		return this.m_target;
	}
	b2MouseJoint.prototype.SetTarget = function (target) {
		if (this.m_bodyB.IsAwake() == false) {
			this.m_bodyB.SetAwake(true);
		}
		this.m_target = target;
	}
	b2MouseJoint.prototype.GetMaxForce = function () {
		return this.m_maxForce;
	}
	b2MouseJoint.prototype.SetMaxForce = function (maxForce) {
		if (maxForce === undefined) maxForce = 0;
		this.m_maxForce = maxForce;
	}
	b2MouseJoint.prototype.GetFrequency = function () {
		return this.m_frequencyHz;
	}
	b2MouseJoint.prototype.SetFrequency = function (hz) {
		if (hz === undefined) hz = 0;
		this.m_frequencyHz = hz;
	}
	b2MouseJoint.prototype.GetDampingRatio = function () {
		return this.m_dampingRatio;
	}
	b2MouseJoint.prototype.SetDampingRatio = function (ratio) {
		if (ratio === undefined) ratio = 0;
		this.m_dampingRatio = ratio;
	}
	b2MouseJoint.prototype.b2MouseJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		this.m_target.SetV(def.target);
		var tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
		var tY = this.m_target.y - this.m_bodyB.m_xf.position.y;
		var tMat = this.m_bodyB.m_xf.R;
		this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
		this.m_maxForce = def.maxForce;
		this.m_impulse.SetZero();
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_beta = 0.0;
		this.m_gamma = 0.0;
	}
	b2MouseJoint.prototype.InitVelocityConstraints = function (step) {
		var b = this.m_bodyB;
		var mass = b.GetMass();
		var omega = 2.0 * Math.PI * this.m_frequencyHz;
		var d = 2.0 * mass * this.m_dampingRatio * omega;
		var k = mass * omega * omega;
		this.m_gamma = step.dt * (d + step.dt * k);
		this.m_gamma = this.m_gamma != 0 ? 1 / this.m_gamma : 0.0;
		this.m_beta = step.dt * k * this.m_gamma;
		var tMat;tMat = b.m_xf.R;
		var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * rX + tMat.col2.x * rY);rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		var invMass = b.m_invMass;
		var invI = b.m_invI;this.K1.col1.x = invMass;
		this.K1.col2.x = 0.0;
		this.K1.col1.y = 0.0;
		this.K1.col2.y = invMass;
		this.K2.col1.x = invI * rY * rY;
		this.K2.col2.x = (-invI * rX * rY);
		this.K2.col1.y = (-invI * rX * rY);
		this.K2.col2.y = invI * rX * rX;
		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.col1.x += this.m_gamma;
		this.K.col2.y += this.m_gamma;
		this.K.GetInverse(this.m_mass);
		this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
		this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;
		b.m_angularVelocity *= 0.98;
		this.m_impulse.x *= step.dtRatio;
		this.m_impulse.y *= step.dtRatio;
		b.m_linearVelocity.x += invMass * this.m_impulse.x;
		b.m_linearVelocity.y += invMass * this.m_impulse.y;
		b.m_angularVelocity += invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
	}
	b2MouseJoint.prototype.SolveVelocityConstraints = function (step) {
		var b = this.m_bodyB;
		var tMat;
		var tX = 0;
		var tY = 0;
		tMat = b.m_xf.R;
		var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		var rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;
		var CdotX = b.m_linearVelocity.x + ((-b.m_angularVelocity * rY));
		var CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
		tMat = this.m_mass;
		tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
		tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
		var impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY));
		var impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY));
		var oldImpulseX = this.m_impulse.x;
		var oldImpulseY = this.m_impulse.y;
		this.m_impulse.x += impulseX;
		this.m_impulse.y += impulseY;
		var maxImpulse = step.dt * this.m_maxForce;
		if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
			this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
		}
		impulseX = this.m_impulse.x - oldImpulseX;
		impulseY = this.m_impulse.y - oldImpulseY;
		b.m_linearVelocity.x += b.m_invMass * impulseX;
		b.m_linearVelocity.y += b.m_invMass * impulseY;
		b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
	}
	b2MouseJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		return true;
	}
	Box2D.inherit(b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2MouseJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2MouseJointDef.b2MouseJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.target = new b2Vec2();
	};
	b2MouseJointDef.prototype.b2MouseJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_mouseJoint;
		this.maxForce = 0.0;
		this.frequencyHz = 5.0;
		this.dampingRatio = 0.7;
	}
	Box2D.inherit(b2PrismaticJoint, Box2D.Dynamics.Joints.b2Joint);
	b2PrismaticJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2PrismaticJoint.b2PrismaticJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_localXAxis1 = new b2Vec2();
		this.m_localYAxis1 = new b2Vec2();
		this.m_axis = new b2Vec2();
		this.m_perp = new b2Vec2();
		this.m_K = new b2Mat33();
		this.m_impulse = new b2Vec3();
	};
	b2PrismaticJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2PrismaticJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2PrismaticJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
	}
	b2PrismaticJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return inv_dt * this.m_impulse.y;
	}
	b2PrismaticJoint.prototype.GetJointTranslation = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var p1 = bA.GetWorldPoint(this.m_localAnchor1);
		var p2 = bB.GetWorldPoint(this.m_localAnchor2);
		var dX = p2.x - p1.x;
		var dY = p2.y - p1.y;
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		var translation = axis.x * dX + axis.y * dY;
		return translation;
	}
	b2PrismaticJoint.prototype.GetJointSpeed = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var p1X = bA.m_sweep.c.x + r1X;
		var p1Y = bA.m_sweep.c.y + r1Y;
		var p2X = bB.m_sweep.c.x + r2X;
		var p2Y = bB.m_sweep.c.y + r2Y;
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		var v1 = bA.m_linearVelocity;
		var v2 = bB.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var w2 = bB.m_angularVelocity;
		var speed = (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
		return speed;
	}
	b2PrismaticJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
	b2PrismaticJoint.prototype.EnableLimit = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableLimit = flag;
	}
	b2PrismaticJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerTranslation;
	}
	b2PrismaticJoint.prototype.GetUpperLimit = function () {
		return this.m_upperTranslation;
	}
	b2PrismaticJoint.prototype.SetLimits = function (lower, upper) {
		if (lower === undefined) lower = 0;
		if (upper === undefined) upper = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_lowerTranslation = lower;
		this.m_upperTranslation = upper;
	}
	b2PrismaticJoint.prototype.IsMotorEnabled = function () {
		return this.m_enableMotor;
	}
	b2PrismaticJoint.prototype.EnableMotor = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	}
	b2PrismaticJoint.prototype.SetMotorSpeed = function (speed) {
		if (speed === undefined) speed = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}
	b2PrismaticJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
	b2PrismaticJoint.prototype.SetMaxMotorForce = function (force) {
		if (force === undefined) force = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorForce = force;
	}
	b2PrismaticJoint.prototype.GetMotorForce = function () {
		return this.m_motorImpulse;
	}
	b2PrismaticJoint.prototype.b2PrismaticJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		var tMat;
		var tX = 0;
		var tY = 0;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_localXAxis1.SetV(def.localAxisA);
		this.m_localYAxis1.x = (-this.m_localXAxis1.y);
		this.m_localYAxis1.y = this.m_localXAxis1.x;
		this.m_refAngle = def.referenceAngle;
		this.m_impulse.SetZero();
		this.m_motorMass = 0.0;
		this.m_motorImpulse = 0.0;
		this.m_lowerTranslation = def.lowerTranslation;
		this.m_upperTranslation = def.upperTranslation;
		this.m_maxMotorForce = def.maxMotorForce;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = b2Joint.e_inactiveLimit;
		this.m_axis.SetZero();
		this.m_perp.SetZero();
	}
	b2PrismaticJoint.prototype.InitVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var tX = 0;
		this.m_localCenterA.SetV(bA.GetLocalCenter());
		this.m_localCenterB.SetV(bB.GetLocalCenter());
		var xf1 = bA.GetTransform();
		var xf2 = bB.GetTransform();
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
		var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
		var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		var dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
		this.m_invMassA = bA.m_invMass;
		this.m_invMassB = bB.m_invMass;
		this.m_invIA = bA.m_invI;
		this.m_invIB = bB.m_invI; {
			this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
			this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
			if (this.m_motorMass > Number.MIN_VALUE) this.m_motorMass = 1.0 / this.m_motorMass;
		} {
			this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
			this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
			this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
			var m1 = this.m_invMassA;
			var m2 = this.m_invMassB;
			var i1 = this.m_invIA;
			var i2 = this.m_invIB;
			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
			this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
			this.m_K.col2.y = i1 + i2;
			this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
			this.m_K.col3.x = this.m_K.col1.z;
			this.m_K.col3.y = this.m_K.col2.z;
			this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
		}
		if (this.m_enableLimit) {
			var jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointTransition <= this.m_lowerTranslation) {
				if (this.m_limitState != b2Joint.e_atLowerLimit) {
					this.m_limitState = b2Joint.e_atLowerLimit;
					this.m_impulse.z = 0.0;
				}
			}
			else if (jointTransition >= this.m_upperTranslation) {
				if (this.m_limitState != b2Joint.e_atUpperLimit) {
					this.m_limitState = b2Joint.e_atUpperLimit;
					this.m_impulse.z = 0.0;
				}
			}
			else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0.0;
			}
		}
		else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}
		if (this.m_enableMotor == false) {
			this.m_motorImpulse = 0.0;
		}
		if (step.warmStarting) {
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;
			var PX = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x;
			var PY = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y;
			var L1 = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
			var L2 = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
			bA.m_linearVelocity.x -= this.m_invMassA * PX;
			bA.m_linearVelocity.y -= this.m_invMassA * PY;
			bA.m_angularVelocity -= this.m_invIA * L1;
			bB.m_linearVelocity.x += this.m_invMassB * PX;
			bB.m_linearVelocity.y += this.m_invMassB * PY;
			bB.m_angularVelocity += this.m_invIB * L2;
		}
		else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
		}
	}
	b2PrismaticJoint.prototype.SolveVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var v1 = bA.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var v2 = bB.m_linearVelocity;
		var w2 = bB.m_angularVelocity;
		var PX = 0;
		var PY = 0;
		var L1 = 0;
		var L2 = 0;
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
			var Cdot = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
			var impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = step.dt * this.m_maxMotorForce;
			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;
			PX = impulse * this.m_axis.x;
			PY = impulse * this.m_axis.y;
			L1 = impulse * this.m_a1;
			L2 = impulse * this.m_a2;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		var Cdot1X = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
		var Cdot1Y = w2 - w1;
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			var Cdot2 = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1;
			var f1 = this.m_impulse.Copy();
			var df = this.m_K.Solve33(new b2Vec3(), (-Cdot1X), (-Cdot1Y), (-Cdot2));
			this.m_impulse.Add(df);
			if (this.m_limitState == b2Joint.e_atLowerLimit) {
				this.m_impulse.z = b2Math.Max(this.m_impulse.z, 0.0);
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				this.m_impulse.z = b2Math.Min(this.m_impulse.z, 0.0);
			}
			var bX = (-Cdot1X) - (this.m_impulse.z - f1.z) * this.m_K.col3.x;
			var bY = (-Cdot1Y) - (this.m_impulse.z - f1.z) * this.m_K.col3.y;
			var f2r = this.m_K.Solve22(new b2Vec2(), bX, bY);
			f2r.x += f1.x;
			f2r.y += f1.y;
			this.m_impulse.x = f2r.x;
			this.m_impulse.y = f2r.y;
			df.x = this.m_impulse.x - f1.x;
			df.y = this.m_impulse.y - f1.y;
			df.z = this.m_impulse.z - f1.z;
			PX = df.x * this.m_perp.x + df.z * this.m_axis.x;
			PY = df.x * this.m_perp.y + df.z * this.m_axis.y;
			L1 = df.x * this.m_s1 + df.y + df.z * this.m_a1;
			L2 = df.x * this.m_s2 + df.y + df.z * this.m_a2;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		else {
			var df2 = this.m_K.Solve22(new b2Vec2(), (-Cdot1X), (-Cdot1Y));
			this.m_impulse.x += df2.x;
			this.m_impulse.y += df2.y;
			PX = df2.x * this.m_perp.x;
			PY = df2.x * this.m_perp.y;
			L1 = df2.x * this.m_s1 + df2.y;
			L2 = df2.x * this.m_s2 + df2.y;
			v1.x -= this.m_invMassA * PX;
			v1.y -= this.m_invMassA * PY;
			w1 -= this.m_invIA * L1;
			v2.x += this.m_invMassB * PX;
			v2.y += this.m_invMassB * PY;
			w2 += this.m_invIB * L2;
		}
		bA.m_linearVelocity.SetV(v1);
		bA.m_angularVelocity = w1;
		bB.m_linearVelocity.SetV(v2);
		bB.m_angularVelocity = w2;
	}
	b2PrismaticJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var limitC = 0;
		var oldLimitImpulse = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var c1 = bA.m_sweep.c;
		var a1 = bA.m_sweep.a;
		var c2 = bB.m_sweep.c;
		var a2 = bB.m_sweep.a;
		var tMat;
		var tX = 0;
		var m1 = 0;
		var m2 = 0;
		var i1 = 0;
		var i2 = 0;
		var linearError = 0.0;
		var angularError = 0.0;
		var active = false;
		var C2 = 0.0;
		var R1 = b2Mat22.FromAngle(a1);
		var R2 = b2Mat22.FromAngle(a2);
		tMat = R1;
		var r1X = this.m_localAnchor1.x - this.m_localCenterA.x;
		var r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = R2;
		var r2X = this.m_localAnchor2.x - this.m_localCenterB.x;
		var r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var dX = c2.x + r2X - c1.x - r1X;
		var dY = c2.y + r2Y - c1.y - r1Y;
		if (this.m_enableLimit) {
			this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
			this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
			this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
			var translation = this.m_axis.x * dX + this.m_axis.y * dY;
			if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
				C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection);
				linearError = b2Math.Abs(translation);
				active = true;
			}
			else if (translation <= this.m_lowerTranslation) {
				C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
				linearError = this.m_lowerTranslation - translation;
				active = true;
			}
			else if (translation >= this.m_upperTranslation) {
				C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
				linearError = translation - this.m_upperTranslation;
				active = true;
			}
		}
		this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
		this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
		this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
		var impulse = new b2Vec3();
		var C1X = this.m_perp.x * dX + this.m_perp.y * dY;
		var C1Y = a2 - a1 - this.m_refAngle;
		linearError = b2Math.Max(linearError, b2Math.Abs(C1X));
		angularError = b2Math.Abs(C1Y);
		if (active) {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;
			this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			this.m_K.col1.y = i1 * this.m_s1 + i2 * this.m_s2;
			this.m_K.col1.z = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
			this.m_K.col2.x = this.m_K.col1.y;
			this.m_K.col2.y = i1 + i2;
			this.m_K.col2.z = i1 * this.m_a1 + i2 * this.m_a2;
			this.m_K.col3.x = this.m_K.col1.z;
			this.m_K.col3.y = this.m_K.col2.z;
			this.m_K.col3.z = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
			this.m_K.Solve33(impulse, (-C1X), (-C1Y), (-C2));
		}
		else {
			m1 = this.m_invMassA;
			m2 = this.m_invMassB;
			i1 = this.m_invIA;
			i2 = this.m_invIB;
			var k11 = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
			var k12 = i1 * this.m_s1 + i2 * this.m_s2;
			var k22 = i1 + i2;
			this.m_K.col1.Set(k11, k12, 0.0);
			this.m_K.col2.Set(k12, k22, 0.0);
			var impulse1 = this.m_K.Solve22(new b2Vec2(), (-C1X), (-C1Y));
			impulse.x = impulse1.x;
			impulse.y = impulse1.y;
			impulse.z = 0.0;
		}
		var PX = impulse.x * this.m_perp.x + impulse.z * this.m_axis.x;
		var PY = impulse.x * this.m_perp.y + impulse.z * this.m_axis.y;
		var L1 = impulse.x * this.m_s1 + impulse.y + impulse.z * this.m_a1;
		var L2 = impulse.x * this.m_s2 + impulse.y + impulse.z * this.m_a2;
		c1.x -= this.m_invMassA * PX;
		c1.y -= this.m_invMassA * PY;
		a1 -= this.m_invIA * L1;
		c2.x += this.m_invMassB * PX;
		c2.y += this.m_invMassB * PY;
		a2 += this.m_invIB * L2;
		bA.m_sweep.a = a1;
		bB.m_sweep.a = a2;
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
	Box2D.inherit(b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2PrismaticJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2PrismaticJointDef.b2PrismaticJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
		this.localAxisA = new b2Vec2();
	};
	b2PrismaticJointDef.prototype.b2PrismaticJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_prismaticJoint;
		this.localAxisA.Set(1.0, 0.0);
		this.referenceAngle = 0.0;
		this.enableLimit = false;
		this.lowerTranslation = 0.0;
		this.upperTranslation = 0.0;
		this.enableMotor = false;
		this.maxMotorForce = 0.0;
		this.motorSpeed = 0.0;
	}
	b2PrismaticJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.localAxisA = this.bodyA.GetLocalVector(axis);
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	}
	Box2D.inherit(b2PulleyJoint, Box2D.Dynamics.Joints.b2Joint);
	b2PulleyJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2PulleyJoint.b2PulleyJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_groundAnchor1 = new b2Vec2();
		this.m_groundAnchor2 = new b2Vec2();
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_u1 = new b2Vec2();
		this.m_u2 = new b2Vec2();
	};
	b2PulleyJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2PulleyJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2PulleyJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
	}
	b2PulleyJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return 0.0;
	}
	b2PulleyJoint.prototype.GetGroundAnchorA = function () {
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	}
	b2PulleyJoint.prototype.GetGroundAnchorB = function () {
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	}
	b2PulleyJoint.prototype.GetLength1 = function () {
		var p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		var dX = p.x - sX;
		var dY = p.y - sY;
		return Math.sqrt(dX * dX + dY * dY);
	}
	b2PulleyJoint.prototype.GetLength2 = function () {
		var p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		var dX = p.x - sX;
		var dY = p.y - sY;
		return Math.sqrt(dX * dX + dY * dY);
	}
	b2PulleyJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
	b2PulleyJoint.prototype.b2PulleyJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		var tMat;
		var tX = 0;
		var tY = 0;
		this.m_ground = this.m_bodyA.m_world.m_groundBody;
		this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
		this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_ratio = def.ratio;
		this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
		this.m_maxLength1 = b2Math.Min(def.maxLengthA, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
		this.m_maxLength2 = b2Math.Min(def.maxLengthB, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);
		this.m_impulse = 0.0;
		this.m_limitImpulse1 = 0.0;
		this.m_limitImpulse2 = 0.0;
	}
	b2PulleyJoint.prototype.InitVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var p1X = bA.m_sweep.c.x + r1X;
		var p1Y = bA.m_sweep.c.y + r1Y;
		var p2X = bB.m_sweep.c.x + r2X;
		var p2Y = bB.m_sweep.c.y + r2Y;
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		this.m_u1.Set(p1X - s1X, p1Y - s1Y);
		this.m_u2.Set(p2X - s2X, p2Y - s2Y);
		var length1 = this.m_u1.Length();
		var length2 = this.m_u2.Length();
		if (length1 > b2Settings.b2_linearSlop) {
			this.m_u1.Multiply(1.0 / length1);
		}
		else {
			this.m_u1.SetZero();
		}
		if (length2 > b2Settings.b2_linearSlop) {
			this.m_u2.Multiply(1.0 / length2);
		}
		else {
			this.m_u2.SetZero();
		}
		var C = this.m_constant - length1 - this.m_ratio * length2;
		if (C > 0.0) {
			this.m_state = b2Joint.e_inactiveLimit;
			this.m_impulse = 0.0;
		}
		else {
			this.m_state = b2Joint.e_atUpperLimit;
		}
		if (length1 < this.m_maxLength1) {
			this.m_limitState1 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse1 = 0.0;
		}
		else {
			this.m_limitState1 = b2Joint.e_atUpperLimit;
		}
		if (length2 < this.m_maxLength2) {
			this.m_limitState2 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse2 = 0.0;
		}
		else {
			this.m_limitState2 = b2Joint.e_atUpperLimit;
		}
		var cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
		var cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
		this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
		this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
		this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
		this.m_limitMass1 = 1.0 / this.m_limitMass1;
		this.m_limitMass2 = 1.0 / this.m_limitMass2;
		this.m_pulleyMass = 1.0 / this.m_pulleyMass;
		if (step.warmStarting) {
			this.m_impulse *= step.dtRatio;
			this.m_limitImpulse1 *= step.dtRatio;
			this.m_limitImpulse2 *= step.dtRatio;
			var P1X = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.x;
			var P1Y = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.y;
			var P2X = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.x;
			var P2Y = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.y;
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		else {
			this.m_impulse = 0.0;
			this.m_limitImpulse1 = 0.0;
			this.m_limitImpulse2 = 0.0;
		}
	}
	b2PulleyJoint.prototype.SolveVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var v1X = 0;
		var v1Y = 0;
		var v2X = 0;
		var v2Y = 0;
		var P1X = 0;
		var P1Y = 0;
		var P2X = 0;
		var P2Y = 0;
		var Cdot = 0;
		var impulse = 0;
		var oldImpulse = 0;
		if (this.m_state == b2Joint.e_atUpperLimit) {
			v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y));
			v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
			v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y));
			v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
			Cdot = (-(this.m_u1.x * v1X + this.m_u1.y * v1Y)) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
			impulse = this.m_pulleyMass * ((-Cdot));
			oldImpulse = this.m_impulse;
			this.m_impulse = b2Math.Max(0.0, this.m_impulse + impulse);
			impulse = this.m_impulse - oldImpulse;
			P1X = (-impulse * this.m_u1.x);
			P1Y = (-impulse * this.m_u1.y);
			P2X = (-this.m_ratio * impulse * this.m_u2.x);
			P2Y = (-this.m_ratio * impulse * this.m_u2.y);
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		if (this.m_limitState1 == b2Joint.e_atUpperLimit) {
			v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y));
			v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
			Cdot = (-(this.m_u1.x * v1X + this.m_u1.y * v1Y));
			impulse = (-this.m_limitMass1 * Cdot);
			oldImpulse = this.m_limitImpulse1;
			this.m_limitImpulse1 = b2Math.Max(0.0, this.m_limitImpulse1 + impulse);
			impulse = this.m_limitImpulse1 - oldImpulse;
			P1X = (-impulse * this.m_u1.x);
			P1Y = (-impulse * this.m_u1.y);
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
		}
		if (this.m_limitState2 == b2Joint.e_atUpperLimit) {
			v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y));
			v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
			Cdot = (-(this.m_u2.x * v2X + this.m_u2.y * v2Y));
			impulse = (-this.m_limitMass2 * Cdot);
			oldImpulse = this.m_limitImpulse2;
			this.m_limitImpulse2 = b2Math.Max(0.0, this.m_limitImpulse2 + impulse);
			impulse = this.m_limitImpulse2 - oldImpulse;
			P2X = (-impulse * this.m_u2.x);
			P2Y = (-impulse * this.m_u2.y);
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}
	b2PulleyJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		var r1X = 0;
		var r1Y = 0;
		var r2X = 0;
		var r2Y = 0;
		var p1X = 0;
		var p1Y = 0;
		var p2X = 0;
		var p2Y = 0;
		var length1 = 0;
		var length2 = 0;
		var C = 0;
		var impulse = 0;
		var oldImpulse = 0;
		var oldLimitPositionImpulse = 0;
		var tX = 0;
		var linearError = 0.0;
		if (this.m_state == b2Joint.e_atUpperLimit) {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			p1X = bA.m_sweep.c.x + r1X;
			p1Y = bA.m_sweep.c.y + r1Y;
			p2X = bB.m_sweep.c.x + r2X;
			p2Y = bB.m_sweep.c.y + r2Y;
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			length1 = this.m_u1.Length();
			length2 = this.m_u2.Length();
			if (length1 > b2Settings.b2_linearSlop) {
				this.m_u1.Multiply(1.0 / length1);
			}
			else {
				this.m_u1.SetZero();
			}
			if (length2 > b2Settings.b2_linearSlop) {
				this.m_u2.Multiply(1.0 / length2);
			}
			else {
				this.m_u2.SetZero();
			}
			C = this.m_constant - length1 - this.m_ratio * length2;
			linearError = b2Math.Max(linearError, (-C));
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
			impulse = (-this.m_pulleyMass * C);
			p1X = (-impulse * this.m_u1.x);
			p1Y = (-impulse * this.m_u1.y);
			p2X = (-this.m_ratio * impulse * this.m_u2.x);
			p2Y = (-this.m_ratio * impulse * this.m_u2.y);
			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;
			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}
		if (this.m_limitState1 == b2Joint.e_atUpperLimit) {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			p1X = bA.m_sweep.c.x + r1X;
			p1Y = bA.m_sweep.c.y + r1Y;
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			length1 = this.m_u1.Length();
			if (length1 > b2Settings.b2_linearSlop) {
				this.m_u1.x *= 1.0 / length1;
				this.m_u1.y *= 1.0 / length1;
			}
			else {
				this.m_u1.SetZero();
			}
			C = this.m_maxLength1 - length1;
			linearError = b2Math.Max(linearError, (-C));
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
			impulse = (-this.m_limitMass1 * C);
			p1X = (-impulse * this.m_u1.x);
			p1Y = (-impulse * this.m_u1.y);
			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;
			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
			bA.SynchronizeTransform();
		}
		if (this.m_limitState2 == b2Joint.e_atUpperLimit) {
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			p2X = bB.m_sweep.c.x + r2X;
			p2Y = bB.m_sweep.c.y + r2Y;
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			length2 = this.m_u2.Length();
			if (length2 > b2Settings.b2_linearSlop) {
				this.m_u2.x *= 1.0 / length2;
				this.m_u2.y *= 1.0 / length2;
			}
			else {
				this.m_u2.SetZero();
			}
			C = this.m_maxLength2 - length2;
			linearError = b2Math.Max(linearError, (-C));
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
			impulse = (-this.m_limitMass2 * C);
			p2X = (-impulse * this.m_u2.x);
			p2Y = (-impulse * this.m_u2.y);
			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
			bB.SynchronizeTransform();
		}
		return linearError < b2Settings.b2_linearSlop;
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength = 2.0;
	});
	Box2D.inherit(b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2PulleyJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2PulleyJointDef.b2PulleyJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.groundAnchorA = new b2Vec2();
		this.groundAnchorB = new b2Vec2();
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2PulleyJointDef.prototype.b2PulleyJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_pulleyJoint;
		this.groundAnchorA.Set((-1.0), 1.0);
		this.groundAnchorB.Set(1.0, 1.0);
		this.localAnchorA.Set((-1.0), 0.0);
		this.localAnchorB.Set(1.0, 0.0);
		this.lengthA = 0.0;
		this.maxLengthA = 0.0;
		this.lengthB = 0.0;
		this.maxLengthB = 0.0;
		this.ratio = 1.0;
		this.collideConnected = true;
	}
	b2PulleyJointDef.prototype.Initialize = function (bA, bB, gaA, gaB, anchorA, anchorB, r) {
		if (r === undefined) r = 0;
		this.bodyA = bA;
		this.bodyB = bB;
		this.groundAnchorA.SetV(gaA);
		this.groundAnchorB.SetV(gaB);
		this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
		var d1X = anchorA.x - gaA.x;
		var d1Y = anchorA.y - gaA.y;
		this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
		var d2X = anchorB.x - gaB.x;
		var d2Y = anchorB.y - gaB.y;
		this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
		this.ratio = r;
		var C = this.lengthA + this.ratio * this.lengthB;
		this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}
	Box2D.inherit(b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint);
	b2RevoluteJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2RevoluteJoint.b2RevoluteJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.K = new b2Mat22();
		this.K1 = new b2Mat22();
		this.K2 = new b2Mat22();
		this.K3 = new b2Mat22();
		this.impulse3 = new b2Vec3();
		this.impulse2 = new b2Vec2();
		this.reduced = new b2Vec2();
		this.m_localAnchor1 = new b2Vec2();
		this.m_localAnchor2 = new b2Vec2();
		this.m_impulse = new b2Vec3();
		this.m_mass = new b2Mat33();
	};
	b2RevoluteJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
	b2RevoluteJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
	b2RevoluteJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
	b2RevoluteJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return inv_dt * this.m_impulse.z;
	}
	b2RevoluteJoint.prototype.GetJointAngle = function () {
		return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
	}
	b2RevoluteJoint.prototype.GetJointSpeed = function () {
		return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
	}
	b2RevoluteJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
	b2RevoluteJoint.prototype.EnableLimit = function (flag) {
		this.m_enableLimit = flag;
	}
	b2RevoluteJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerAngle;
	}
	b2RevoluteJoint.prototype.GetUpperLimit = function () {
		return this.m_upperAngle;
	}
	b2RevoluteJoint.prototype.SetLimits = function (lower, upper) {
		if (lower === undefined) lower = 0;
		if (upper === undefined) upper = 0;
		this.m_lowerAngle = lower;
		this.m_upperAngle = upper;
	}
	b2RevoluteJoint.prototype.IsMotorEnabled = function () {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		return this.m_enableMotor;
	}
	b2RevoluteJoint.prototype.EnableMotor = function (flag) {
		this.m_enableMotor = flag;
	}
	b2RevoluteJoint.prototype.SetMotorSpeed = function (speed) {
		if (speed === undefined) speed = 0;
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}
	b2RevoluteJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
	b2RevoluteJoint.prototype.SetMaxMotorTorque = function (torque) {
		if (torque === undefined) torque = 0;
		this.m_maxMotorTorque = torque;
	}
	b2RevoluteJoint.prototype.GetMotorTorque = function () {
		return this.m_maxMotorTorque;
	}
	b2RevoluteJoint.prototype.b2RevoluteJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_referenceAngle = def.referenceAngle;
		this.m_impulse.SetZero();
		this.m_motorImpulse = 0.0;
		this.m_lowerAngle = def.lowerAngle;
		this.m_upperAngle = def.upperAngle;
		this.m_maxMotorTorque = def.maxMotorTorque;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = b2Joint.e_inactiveLimit;
	}
	b2RevoluteJoint.prototype.InitVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var tX = 0;
		if (this.m_enableMotor || this.m_enableLimit) {}
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		var m1 = bA.m_invMass;
		var m2 = bB.m_invMass;
		var i1 = bA.m_invI;
		var i2 = bB.m_invI;
		this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
		this.m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2;
		this.m_mass.col3.x = (-r1Y * i1) - r2Y * i2;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
		this.m_mass.col3.y = r1X * i1 + r2X * i2;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = i1 + i2;
		this.m_motorMass = 1.0 / (i1 + i2);
		if (this.m_enableMotor == false) {
			this.m_motorImpulse = 0.0;
		}
		if (this.m_enableLimit) {
			var jointAngle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointAngle <= this.m_lowerAngle) {
				if (this.m_limitState != b2Joint.e_atLowerLimit) {
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atLowerLimit;
			}
			else if (jointAngle >= this.m_upperAngle) {
				if (this.m_limitState != b2Joint.e_atUpperLimit) {
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atUpperLimit;
			}
			else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0.0;
			}
		}
		else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}
		if (step.warmStarting) {
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;
			var PX = this.m_impulse.x;
			var PY = this.m_impulse.y;
			bA.m_linearVelocity.x -= m1 * PX;
			bA.m_linearVelocity.y -= m1 * PY;
			bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.m_motorImpulse + this.m_impulse.z);
			bB.m_linearVelocity.x += m2 * PX;
			bB.m_linearVelocity.y += m2 * PY;
			bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.m_motorImpulse + this.m_impulse.z);
		}
		else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
		}
	}
	b2RevoluteJoint.prototype.SolveVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var tMat;
		var tX = 0;
		var newImpulse = 0;
		var r1X = 0;
		var r1Y = 0;
		var r2X = 0;
		var r2Y = 0;
		var v1 = bA.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var v2 = bB.m_linearVelocity;
		var w2 = bB.m_angularVelocity;
		var m1 = bA.m_invMass;
		var m2 = bB.m_invMass;
		var i1 = bA.m_invI;
		var i2 = bB.m_invI;
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
			var Cdot = w2 - w1 - this.m_motorSpeed;
			var impulse = this.m_motorMass * ((-Cdot));
			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = step.dt * this.m_maxMotorTorque;
			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;
			w1 -= i1 * impulse;
			w2 += i2 * impulse;
		}
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			var Cdot1X = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
			var Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
			var Cdot2 = w2 - w1;
			this.m_mass.Solve33(this.impulse3, (-Cdot1X), (-Cdot1Y), (-Cdot2));
			if (this.m_limitState == b2Joint.e_equalLimits) {
				this.m_impulse.Add(this.impulse3);
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse < 0.0) {
					this.m_mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = (-this.m_impulse.z);
					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0.0;
				}
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse > 0.0) {
					this.m_mass.Solve22(this.reduced, (-Cdot1X), (-Cdot1Y));
					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = (-this.m_impulse.z);
					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0.0;
				}
			}
			v1.x -= m1 * this.impulse3.x;
			v1.y -= m1 * this.impulse3.y;
			w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
			v2.x += m2 * this.impulse3.x;
			v2.y += m2 * this.impulse3.y;
			w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
		}
		else {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			var CdotX = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
			var CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
			this.m_mass.Solve22(this.impulse2, (-CdotX), (-CdotY));
			this.m_impulse.x += this.impulse2.x;
			this.m_impulse.y += this.impulse2.y;
			v1.x -= m1 * this.impulse2.x;
			v1.y -= m1 * this.impulse2.y;
			w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
			v2.x += m2 * this.impulse2.x;
			v2.y += m2 * this.impulse2.y;
			w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
		}
		bA.m_linearVelocity.SetV(v1);
		bA.m_angularVelocity = w1;
		bB.m_linearVelocity.SetV(v2);
		bB.m_angularVelocity = w2;
	}
	b2RevoluteJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var oldLimitImpulse = 0;
		var C = 0;
		var tMat;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var angularError = 0.0;
		var positionError = 0.0;
		var tX = 0;
		var impulseX = 0;
		var impulseY = 0;
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
			var angle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			var limitImpulse = 0.0;
			if (this.m_limitState == b2Joint.e_equalLimits) {
				C = b2Math.Clamp(angle - this.m_lowerAngle, (-b2Settings.b2_maxAngularCorrection), b2Settings.b2_maxAngularCorrection);
				limitImpulse = (-this.m_motorMass * C);
				angularError = b2Math.Abs(C);
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit) {
				C = angle - this.m_lowerAngle;
				angularError = (-C);
				C = b2Math.Clamp(C + b2Settings.b2_angularSlop, (-b2Settings.b2_maxAngularCorrection), 0.0);
				limitImpulse = (-this.m_motorMass * C);
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit) {
				C = angle - this.m_upperAngle;
				angularError = C;
				C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
				limitImpulse = (-this.m_motorMass * C);
			}
			bA.m_sweep.a -= bA.m_invI * limitImpulse;
			bB.m_sweep.a += bB.m_invI * limitImpulse;
			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		} {
			tMat = bA.m_xf.R;
			var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			tMat = bB.m_xf.R;
			var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			var CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
			var CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
			var CLengthSquared = CX * CX + CY * CY;
			var CLength = Math.sqrt(CLengthSquared);
			positionError = CLength;
			var invMass1 = bA.m_invMass;
			var invMass2 = bB.m_invMass;
			var invI1 = bA.m_invI;
			var invI2 = bB.m_invI;
			var k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
			if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
				var uX = CX / CLength;
				var uY = CY / CLength;
				var k = invMass1 + invMass2;
				var m = 1.0 / k;
				impulseX = m * ((-CX));
				impulseY = m * ((-CY));
				var k_beta = 0.5;
				bA.m_sweep.c.x -= k_beta * invMass1 * impulseX;
				bA.m_sweep.c.y -= k_beta * invMass1 * impulseY;
				bB.m_sweep.c.x += k_beta * invMass2 * impulseX;
				bB.m_sweep.c.y += k_beta * invMass2 * impulseY;
				CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
				CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
			}
			this.K1.col1.x = invMass1 + invMass2;
			this.K1.col2.x = 0.0;
			this.K1.col1.y = 0.0;
			this.K1.col2.y = invMass1 + invMass2;
			this.K2.col1.x = invI1 * r1Y * r1Y;
			this.K2.col2.x = (-invI1 * r1X * r1Y);
			this.K2.col1.y = (-invI1 * r1X * r1Y);
			this.K2.col2.y = invI1 * r1X * r1X;
			this.K3.col1.x = invI2 * r2Y * r2Y;
			this.K3.col2.x = (-invI2 * r2X * r2Y);
			this.K3.col1.y = (-invI2 * r2X * r2Y);
			this.K3.col2.y = invI2 * r2X * r2X;
			this.K.SetM(this.K1);
			this.K.AddM(this.K2);
			this.K.AddM(this.K3);
			this.K.Solve(b2RevoluteJoint.tImpulse, (-CX), (-CY));
			impulseX = b2RevoluteJoint.tImpulse.x;
			impulseY = b2RevoluteJoint.tImpulse.y;
			bA.m_sweep.c.x -= bA.m_invMass * impulseX;
			bA.m_sweep.c.y -= bA.m_invMass * impulseY;
			bA.m_sweep.a -= bA.m_invI * (r1X * impulseY - r1Y * impulseX);
			bB.m_sweep.c.x += bB.m_invMass * impulseX;
			bB.m_sweep.c.y += bB.m_invMass * impulseY;
			bB.m_sweep.a += bB.m_invI * (r2X * impulseY - r2Y * impulseX);
			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}
		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2();
	});
	Box2D.inherit(b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2RevoluteJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2RevoluteJointDef.b2RevoluteJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2RevoluteJointDef.prototype.b2RevoluteJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_revoluteJoint;
		this.localAnchorA.Set(0.0, 0.0);
		this.localAnchorB.Set(0.0, 0.0);
		this.referenceAngle = 0.0;
		this.lowerAngle = 0.0;
		this.upperAngle = 0.0;
		this.maxMotorTorque = 0.0;
		this.motorSpeed = 0.0;
		this.enableLimit = false;
		this.enableMotor = false;
	}
	b2RevoluteJointDef.prototype.Initialize = function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	}
	Box2D.inherit(b2WeldJoint, Box2D.Dynamics.Joints.b2Joint);
	b2WeldJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
	b2WeldJoint.b2WeldJoint = function () {
		Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
		this.m_localAnchorA = new b2Vec2();
		this.m_localAnchorB = new b2Vec2();
		this.m_impulse = new b2Vec3();
		this.m_mass = new b2Mat33();
	};
	b2WeldJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
	b2WeldJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
	b2WeldJoint.prototype.GetReactionForce = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
	b2WeldJoint.prototype.GetReactionTorque = function (inv_dt) {
		if (inv_dt === undefined) inv_dt = 0;
		return inv_dt * this.m_impulse.z;
	}
	b2WeldJoint.prototype.b2WeldJoint = function (def) {
		this.__super.b2Joint.call(this, def);
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		this.m_referenceAngle = def.referenceAngle;
		this.m_impulse.SetZero();
		this.m_mass = new b2Mat33();
	}
	b2WeldJoint.prototype.InitVelocityConstraints = function (step) {
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		var mA = bA.m_invMass;
		var mB = bB.m_invMass;
		var iA = bA.m_invI;
		var iB = bB.m_invI;
		this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
		this.m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
		this.m_mass.col3.x = (-rAY * iA) - rBY * iB;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
		this.m_mass.col3.y = rAX * iA + rBX * iB;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = iA + iB;
		if (step.warmStarting) {
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_impulse.z *= step.dtRatio;
			bA.m_linearVelocity.x -= mA * this.m_impulse.x;
			bA.m_linearVelocity.y -= mA * this.m_impulse.y;
			bA.m_angularVelocity -= iA * (rAX * this.m_impulse.y - rAY * this.m_impulse.x + this.m_impulse.z);
			bB.m_linearVelocity.x += mB * this.m_impulse.x;
			bB.m_linearVelocity.y += mB * this.m_impulse.y;
			bB.m_angularVelocity += iB * (rBX * this.m_impulse.y - rBY * this.m_impulse.x + this.m_impulse.z);
		}
		else {
			this.m_impulse.SetZero();
		}
	}
	b2WeldJoint.prototype.SolveVelocityConstraints = function (step) {
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		var vA = bA.m_linearVelocity;
		var wA = bA.m_angularVelocity;
		var vB = bB.m_linearVelocity;
		var wB = bB.m_angularVelocity;
		var mA = bA.m_invMass;
		var mB = bB.m_invMass;
		var iA = bA.m_invI;
		var iB = bB.m_invI;
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		var Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
		var Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;
		var Cdot2 = wB - wA;
		var impulse = new b2Vec3();
		this.m_mass.Solve33(impulse, (-Cdot1X), (-Cdot1Y), (-Cdot2));
		this.m_impulse.Add(impulse);
		vA.x -= mA * impulse.x;
		vA.y -= mA * impulse.y;
		wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
		vB.x += mB * impulse.x;
		vB.y += mB * impulse.y;
		wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	}
	b2WeldJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		if (baumgarte === undefined) baumgarte = 0;
		var tMat;
		var tX = 0;
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		var mA = bA.m_invMass;
		var mB = bB.m_invMass;
		var iA = bA.m_invI;
		var iB = bB.m_invI;
		var C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX;
		var C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY;
		var C2 = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
		var k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
		var positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
		var angularError = b2Math.Abs(C2);
		if (positionError > k_allowedStretch) {
			iA *= 1.0;
			iB *= 1.0;
		}
		this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
		this.m_mass.col2.x = (-rAY * rAX * iA) - rBY * rBX * iB;
		this.m_mass.col3.x = (-rAY * iA) - rBY * iB;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
		this.m_mass.col3.y = rAX * iA + rBX * iB;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = iA + iB;
		var impulse = new b2Vec3();
		this.m_mass.Solve33(impulse, (-C1X), (-C1Y), (-C2));
		bA.m_sweep.c.x -= mA * impulse.x;
		bA.m_sweep.c.y -= mA * impulse.y;
		bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
		bB.m_sweep.c.x += mB * impulse.x;
		bB.m_sweep.c.y += mB * impulse.y;
		bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
	Box2D.inherit(b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2WeldJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2WeldJointDef.b2WeldJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2WeldJointDef.prototype.b2WeldJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_weldJoint;
		this.referenceAngle = 0.0;
	}
	b2WeldJointDef.prototype.Initialize = function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	}
})();
(function () {
	var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
	b2DebugDraw.b2DebugDraw = function () {
		this.m_drawScale = 1.0;
		this.m_lineThickness = 1.0;
		this.m_alpha = 1.0;
		this.m_fillAlpha = 1.0;
		this.m_xformScale = 1.0;
		var __this = this;
		//#WORKAROUND
		this.m_sprite = {
			graphics: {
				clear: function () {
					__this.m_ctx.clearRect(0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height)
				}
			}
		};
	};
	b2DebugDraw.prototype._color = function (color, alpha) {
		return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
	};
	b2DebugDraw.prototype.b2DebugDraw = function () {
		this.m_drawFlags = 0;
	};
	b2DebugDraw.prototype.SetFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags = flags;
	};
	b2DebugDraw.prototype.GetFlags = function () {
		return this.m_drawFlags;
	};
	b2DebugDraw.prototype.AppendFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags |= flags;
	};
	b2DebugDraw.prototype.ClearFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags &= ~flags;
	};
	b2DebugDraw.prototype.SetSprite = function (sprite) {
		this.m_ctx = sprite;
	};
	b2DebugDraw.prototype.GetSprite = function () {
		return this.m_ctx;
	};
	b2DebugDraw.prototype.SetDrawScale = function (drawScale) {
		if (drawScale === undefined) drawScale = 0;
		this.m_drawScale = drawScale;
	};
	b2DebugDraw.prototype.GetDrawScale = function () {
		return this.m_drawScale;
	};
	b2DebugDraw.prototype.SetLineThickness = function (lineThickness) {
		if (lineThickness === undefined) lineThickness = 0;
		this.m_lineThickness = lineThickness;
		this.m_ctx.strokeWidth = lineThickness;
	};
	b2DebugDraw.prototype.GetLineThickness = function () {
		return this.m_lineThickness;
	};
	b2DebugDraw.prototype.SetAlpha = function (alpha) {
		if (alpha === undefined) alpha = 0;
		this.m_alpha = alpha;
	};
	b2DebugDraw.prototype.GetAlpha = function () {
		return this.m_alpha;
	};
	b2DebugDraw.prototype.SetFillAlpha = function (alpha) {
		if (alpha === undefined) alpha = 0;
		this.m_fillAlpha = alpha;
	};
	b2DebugDraw.prototype.GetFillAlpha = function () {
		return this.m_fillAlpha;
	};
	b2DebugDraw.prototype.SetXFormScale = function (xformScale) {
		if (xformScale === undefined) xformScale = 0;
		this.m_xformScale = xformScale;
	};
	b2DebugDraw.prototype.GetXFormScale = function () {
		return this.m_xformScale;
	};
	b2DebugDraw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
		if (!vertexCount) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		for (var i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}
		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
		if (!vertexCount) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.fillStyle = this._color(color.color, this.m_fillAlpha);
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		for (var i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}
		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.closePath();
		s.fill();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawCircle = function (center, radius, color) {
		if (!radius) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
		if (!radius) return;
		var s = this.m_ctx,
			drawScale = this.m_drawScale,
			cx = center.x * drawScale,
			cy = center.y * drawScale;
		s.moveTo(0, 0);
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.fillStyle = this._color(color.color, this.m_fillAlpha);
		s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
		s.moveTo(cx, cy);
		s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
		s.closePath();
		s.fill();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSegment = function (p1, p2, color) {
		var s = this.m_ctx,
			drawScale = this.m_drawScale;
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.beginPath();
		s.moveTo(p1.x * drawScale, p1.y * drawScale);
		s.lineTo(p2.x * drawScale, p2.y * drawScale);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawTransform = function (xf) {
		var s = this.m_ctx,
			drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(0xff0000, this.m_alpha);
		s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
		s.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale);

		s.strokeStyle = this._color(0xff00, this.m_alpha);
		s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
		s.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale);
		s.closePath();
		s.stroke();
	};
})(); //post-definitions
var i;
for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;





var Box2D = {};

(function (a2j, undefined) {

	if(!(Object.prototype.defineProperty instanceof Function)
		&& Object.prototype.__defineGetter__ instanceof Function
		&& Object.prototype.__defineSetter__ instanceof Function)
	{
		Object.defineProperty = function(obj, p, cfg) {
			if(cfg.get instanceof Function)
				obj.__defineGetter__(p, cfg.get);
			if(cfg.set instanceof Function)
				obj.__defineSetter__(p, cfg.set);
		}
	}

	function emptyFn() {};
	a2j.inherit = function(cls, base) {
		var tmpCtr = cls;
		emptyFn.prototype = base.prototype;
		cls.prototype = new emptyFn;
		cls.prototype.constructor = tmpCtr;
	};

	a2j.generateCallback = function generateCallback(context, cb) {
		return function () {
			cb.apply(context, arguments);
		};
	};

	a2j.NVector = function NVector(length) {
		if (length === undefined) length = 0;
		var tmp = new Array(length || 0);
		for (var i = 0; i < length; ++i)
		tmp[i] = 0;
		return tmp;
	};

	a2j.is = function is(o1, o2) {
		if (o1 === null) return false;
		if ((o2 instanceof Function) && (o1 instanceof o2)) return true;
		if ((o1.constructor.__implements != undefined) && (o1.constructor.__implements[o2])) return true;
		return false;
	};

	a2j.parseUInt = function(v) {
		return Math.abs(parseInt(v));
	}

})(Box2D);

//#TODO remove assignments from global namespace
var Vector = Array;
var Vector_a2j_Number = Box2D.NVector;
//package structure
if (typeof(Box2D) === "undefined") Box2D = {};
if (typeof(Box2D.Collision) === "undefined") Box2D.Collision = {};
if (typeof(Box2D.Collision.Shapes) === "undefined") Box2D.Collision.Shapes = {};
if (typeof(Box2D.Common) === "undefined") Box2D.Common = {};
if (typeof(Box2D.Common.Math) === "undefined") Box2D.Common.Math = {};
if (typeof(Box2D.Dynamics) === "undefined") Box2D.Dynamics = {};
if (typeof(Box2D.Dynamics.Contacts) === "undefined") Box2D.Dynamics.Contacts = {};
if (typeof(Box2D.Dynamics.Controllers) === "undefined") Box2D.Dynamics.Controllers = {};
if (typeof(Box2D.Dynamics.Joints) === "undefined") Box2D.Dynamics.Joints = {};
//pre-definitions
(function () {
	Box2D.Collision.IBroadPhase = 'Box2D.Collision.IBroadPhase';
	Box2D.Common.b2internal = 'Box2D.Common.b2internal';

	
