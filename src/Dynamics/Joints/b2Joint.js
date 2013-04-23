function b2Joint(def) {
	b2Settings.b2Assert(def.bodyA !== def.bodyB);
	this.m_type = def.type;
	this.m_prev = null;
	this.m_next = null;
	this.m_bodyA = def.bodyA;
	this.m_bodyB = def.bodyB;
	this.m_collideConnected = def.collideConnected;
	this.m_islandFlag = false;
	this.m_userData = def.userData;
	this.m_edgeA = new b2JointEdge();
	this.m_edgeB = new b2JointEdge();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
}

b2Joint.e_unknownJoint   = 0;
b2Joint.e_revoluteJoint  = 1;
b2Joint.e_prismaticJoint = 2;
b2Joint.e_distanceJoint  = 3;
b2Joint.e_pulleyJoint    = 4;
b2Joint.e_mouseJoint     = 5;
b2Joint.e_gearJoint      = 6;
b2Joint.e_lineJoint      = 7;
b2Joint.e_weldJoint      = 8;
b2Joint.e_frictionJoint  = 9;

b2Joint.e_inactiveLimit  = 0;
b2Joint.e_atLowerLimit   = 1;
b2Joint.e_atUpperLimit   = 2;
b2Joint.e_equalLimits    = 3;

b2Joint.Create = function (def) {
	switch (def.type) {
	case b2Joint.e_distanceJoint:
		return new b2DistanceJoint(def);
	case b2Joint.e_mouseJoint:
		return new b2MouseJoint(def);
	case b2Joint.e_prismaticJoint:
		return new b2PrismaticJoint(def);
	case b2Joint.e_revoluteJoint:
		return new b2RevoluteJoint(def);
	case b2Joint.e_pulleyJoint:
		return new b2PulleyJoint(def);
	case b2Joint.e_gearJoint:
		return new b2GearJoint(def);
	case b2Joint.e_lineJoint:
		return new b2LineJoint(def);
	case b2Joint.e_weldJoint:
		return new b2WeldJoint(def);
	case b2Joint.e_frictionJoint:
		return new b2FrictionJoint(def);
	}
};

b2Joint.Destroy = function (joint, allocator) {};

b2Joint.prototype = {
	GetType : function () {
		return this.m_type;
	},

	GetAnchorA : function () {
		return null;
	},

	GetAnchorB : function () {
		return null;
	},

	GetReactionForce : function (inv_dt) {
		return null;
	},

	GetReactionTorque : function (inv_dt) {
		return 0;
	},

	GetBodyA : function () {
		return this.m_bodyA;
	},

	GetBodyB : function () {
		return this.m_bodyB;
	},

	GetNext : function () {
		return this.m_next;
	},

	GetUserData : function () {
		return this.m_userData;
	},

	SetUserData : function (data) {
		this.m_userData = data;
	},

	IsActive : function () {
		return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
	},

	InitVelocityConstraints : function (step) {},

	SolveVelocityConstraints : function (step) {},

	FinalizeVelocityConstraints : function () {},

	SolvePositionConstraints : function (baumgarte) {
		return false;
	}
};
