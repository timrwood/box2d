
/* Joints */

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


