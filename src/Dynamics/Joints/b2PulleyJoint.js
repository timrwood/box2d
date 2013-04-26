function b2PulleyJoint(def) {
	b2Joint.apply(this, arguments);
	this.m_localAnchor1 = def.localAnchorA.Copy();
	this.m_localAnchor2 = def.localAnchorB.Copy();

	this.m_u1 = new b2Vec2();
	this.m_u2 = new b2Vec2();

	this.m_ground = this.m_bodyA.m_world.m_groundBody;

	this.m_groundAnchor1 = new b2Vec2();
	this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
	this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;

	this.m_groundAnchor2 = new b2Vec2();
	this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
	this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;

	this.m_ratio = def.ratio;
	this.m_constant = def.lengthA + this.m_ratio * def.lengthB;

	this.m_maxLength1 = Math.min(def.maxLengthA, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
	this.m_maxLength2 = Math.min(def.maxLengthB, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);

	this.m_impulse = 0;
	this.m_limitImpulse1 = 0;
	this.m_limitImpulse2 = 0;
}

Box2D.b2PulleyJoint = b2PulleyJoint;

inherit(b2Joint, b2PulleyJoint);

b2PulleyJoint.b2_minPulleyLength = 2;

b2PulleyJoint.prototype = {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
	},

	GetReactionTorque : function (inv_dt) {
		return 0;
	},

	GetGroundAnchorA : function () {
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	},

	GetGroundAnchorB : function () {
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	},

	GetLength1 : function () {
		var p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1),
			sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x,
			sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y,
			dX = p.x - sX,
			dY = p.y - sY;
		return Math.sqrt(dX * dX + dY * dY);
	},

	GetLength2 : function () {
		var p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2),
			sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x,
			sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y,
			dX = p.x - sX,
			dY = p.y - sY;
		return Math.sqrt(dX * dX + dY * dY);
	},

	GetRatio : function () {
		return this.m_ratio;
	},

	InitVelocityConstraints : function (step) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			tMat,
			r1X, r1Y,
			tX,
			r2X, r2Y,
			p1X, p1Y,
			p2X, p2Y,
			s1X, s1Y,
			s2X, s2Y,
			length1, length2,
			C,
			cr1u1, cr2u2,
			P1X, P1Y,
			P2X, P2Y;

		tMat = bA.m_xf.R;
		r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;

		tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
		r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
		r1X = tX;

		tMat = bB.m_xf.R;
		r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

		tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
		r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
		r2X = tX;

		p1X = bA.m_sweep.c.x + r1X;
		p1Y = bA.m_sweep.c.y + r1Y;

		p2X = bB.m_sweep.c.x + r2X;
		p2Y = bB.m_sweep.c.y + r2Y;

		s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;

		s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;

		this.m_u1.Set(p1X - s1X, p1Y - s1Y);
		this.m_u2.Set(p2X - s2X, p2Y - s2Y);

		length1 = this.m_u1.Length();
		length2 = this.m_u2.Length();

		if (length1 > b2Settings.b2_linearSlop) {
			this.m_u1.Multiply(1 / length1);
		} else {
			this.m_u1.SetZero();
		}

		if (length2 > b2Settings.b2_linearSlop) {
			this.m_u2.Multiply(1 / length2);
		} else {
			this.m_u2.SetZero();
		}

		C = this.m_constant - length1 - this.m_ratio * length2;

		if (C > 0) {
			this.m_state = b2Joint.e_inactiveLimit;
			this.m_impulse = 0;
		} else {
			this.m_state = b2Joint.e_atUpperLimit;
		}

		if (length1 < this.m_maxLength1) {
			this.m_limitState1 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse1 = 0;
		} else {
			this.m_limitState1 = b2Joint.e_atUpperLimit;
		}

		if (length2 < this.m_maxLength2) {
			this.m_limitState2 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse2 = 0;
		} else {
			this.m_limitState2 = b2Joint.e_atUpperLimit;
		}

		cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
		cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;

		this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
		this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;

		this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;

		this.m_limitMass1 = 1 / this.m_limitMass1;
		this.m_limitMass2 = 1 / this.m_limitMass2;
		this.m_pulleyMass = 1 / this.m_pulleyMass;

		if (step.warmStarting) {
			this.m_impulse *= step.dtRatio;
			this.m_limitImpulse1 *= step.dtRatio;
			this.m_limitImpulse2 *= step.dtRatio;

			P1X = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.x;
			P1Y = ((-this.m_impulse) - this.m_limitImpulse1) * this.m_u1.y;

			P2X = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.x;
			P2Y = ((-this.m_ratio * this.m_impulse) - this.m_limitImpulse2) * this.m_u2.y;

			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;

			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;

			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		} else {
			this.m_impulse = 0;
			this.m_limitImpulse1 = 0;
			this.m_limitImpulse2 = 0;
		}
	},

	SolveVelocityConstraints : function (step) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			tMat,
			r1X, r1Y,
			tX,
			r2X, r2Y,
			v1X, v1Y,
			v2X, v2Y,
			P1X, P1Y,
			P2X, P2Y,
			Cdot = 0,
			impulse = 0,
			oldImpulse = 0;


		tMat = bA.m_xf.R;
		r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;

		tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
		r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
		r1X = tX;

		tMat = bB.m_xf.R;
		r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

		tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
		r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
		r2X = tX;

		if (this.m_state === b2Joint.e_atUpperLimit) {
			v1X = bA.m_linearVelocity.x - bA.m_angularVelocity * r1Y;
			v1Y = bA.m_linearVelocity.y + bA.m_angularVelocity * r1X;

			v2X = bB.m_linearVelocity.x - bB.m_angularVelocity * r2Y;
			v2Y = bB.m_linearVelocity.y + bB.m_angularVelocity * r2X;

			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);

			impulse = -this.m_pulleyMass * Cdot;
			oldImpulse = this.m_impulse;
			this.m_impulse = Math.max(0, this.m_impulse + impulse);
			impulse = this.m_impulse - oldImpulse;

			P1X = -impulse * this.m_u1.x;
			P1Y = -impulse * this.m_u1.y;

			P2X = -this.m_ratio * impulse * this.m_u2.x;
			P2Y = -this.m_ratio * impulse * this.m_u2.y;

			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;

			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;

			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}

		if (this.m_limitState1 === b2Joint.e_atUpperLimit) {
			v1X = bA.m_linearVelocity.x - bA.m_angularVelocity * r1Y;
			v1Y = bA.m_linearVelocity.y + bA.m_angularVelocity * r1X;

			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);

			impulse = -this.m_limitMass1 * Cdot;
			oldImpulse = this.m_limitImpulse1;

			this.m_limitImpulse1 = Math.max(0, this.m_limitImpulse1 + impulse);

			impulse = this.m_limitImpulse1 - oldImpulse;

			P1X = -impulse * this.m_u1.x;
			P1Y = -impulse * this.m_u1.y;

			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;

			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
		}

		if (this.m_limitState2 === b2Joint.e_atUpperLimit) {
			v2X = bB.m_linearVelocity.x - bB.m_angularVelocity * r2Y;
			v2Y = bB.m_linearVelocity.y + bB.m_angularVelocity * r2X;

			Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);

			impulse = -this.m_limitMass2 * Cdot;
			oldImpulse = this.m_limitImpulse2;

			this.m_limitImpulse2 = Math.max(0, this.m_limitImpulse2 + impulse);

			impulse = this.m_limitImpulse2 - oldImpulse;

			P2X = -impulse * this.m_u2.x;
			P2Y = -impulse * this.m_u2.y;

			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	},

	SolvePositionConstraints : function (baumgarte) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			tMat,
			s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x,
			s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y,
			s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x,
			s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y,
			r1X = 0,
			r1Y = 0,
			r2X = 0,
			r2Y = 0,
			p1X = 0,
			p1Y = 0,
			p2X = 0,
			p2Y = 0,
			length1 = 0,
			length2 = 0,
			C = 0,
			impulse = 0,
			oldImpulse = 0,
			oldLimitPositionImpulse = 0,
			tX = 0,
			linearError = 0;

		baumgarte = baumgarte || 0;

		if (this.m_state === b2Joint.e_atUpperLimit) {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;

			tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
			r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
			r1X = tX;

			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

			tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
			r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
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
				this.m_u1.Multiply(1 / length1);
			} else {
				this.m_u1.SetZero();
			}

			if (length2 > b2Settings.b2_linearSlop) {
				this.m_u2.Multiply(1 / length2);
			} else {
				this.m_u2.SetZero();
			}

			C = this.m_constant - length1 - this.m_ratio * length2;

			linearError = Math.max(linearError, -C);

			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0);

			impulse = -this.m_pulleyMass * C;

			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;

			p2X = -this.m_ratio * impulse * this.m_u2.x;
			p2Y = -this.m_ratio * impulse * this.m_u2.y;

			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;

			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;

			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);

			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}

		if (this.m_limitState1 === b2Joint.e_atUpperLimit) {
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;

			tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
			r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
			r1X = tX;

			p1X = bA.m_sweep.c.x + r1X;
			p1Y = bA.m_sweep.c.y + r1Y;

			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			length1 = this.m_u1.Length();

			if (length1 > b2Settings.b2_linearSlop) {
				this.m_u1.x *= 1 / length1;
				this.m_u1.y *= 1 / length1;
			} else {
				this.m_u1.SetZero();
			}

			C = this.m_maxLength1 - length1;

			linearError = Math.max(linearError, -C);

			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0);

			impulse = -this.m_limitMass1 * C;

			p1X = (-impulse * this.m_u1.x);
			p1Y = (-impulse * this.m_u1.y);

			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;
			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);

			bA.SynchronizeTransform();
		}

		if (this.m_limitState2 === b2Joint.e_atUpperLimit) {
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

			tX = tMat.col1.x * r2X + tMat.col2.x * r2Y;
			r2Y = tMat.col1.y * r2X + tMat.col2.y * r2Y;
			r2X = tX;

			p2X = bB.m_sweep.c.x + r2X;
			p2Y = bB.m_sweep.c.y + r2Y;

			this.m_u2.Set(p2X - s2X, p2Y - s2Y);

			length2 = this.m_u2.Length();

			if (length2 > b2Settings.b2_linearSlop) {
				this.m_u2.x *= 1 / length2;
				this.m_u2.y *= 1 / length2;
			} else {
				this.m_u2.SetZero();
			}

			C = this.m_maxLength2 - length2;

			linearError = Math.max(linearError, -C);

			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0);

			impulse = -this.m_limitMass2 * C;

			p2X = -impulse * this.m_u2.x;
			p2Y = -impulse * this.m_u2.y;

			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);

			bB.SynchronizeTransform();
		}
		return linearError < b2Settings.b2_linearSlop;
	}
};
