
/* Joints */

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
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2World.s_timestep2 = new b2TimeStep();
		Box2D.Dynamics.b2World.s_xf = new b2Transform();
		Box2D.Dynamics.b2World.s_backupA = new b2Sweep();
		Box2D.Dynamics.b2World.s_backupB = new b2Sweep();
		Box2D.Dynamics.b2World.s_timestep = new b2TimeStep();
		Box2D.Dynamics.b2World.s_queue = new Vector();
		Box2D.Dynamics.b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2Contact.s_input = new b2TOIInput();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold = new b2WorldManifold();
		Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new b2PositionSolverManifold();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA = new b2Vec2();
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB = new b2Vec2();
	});
})();
(function () {

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
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2();
	});
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


