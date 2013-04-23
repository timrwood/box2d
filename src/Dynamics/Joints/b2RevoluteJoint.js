function b2RevoluteJoint(def) {
	b2Joint.apply(this, arguments);

	this.K = new b2Mat22();
	this.K1 = new b2Mat22();
	this.K2 = new b2Mat22();
	this.K3 = new b2Mat22();

	this.m_impulse = new b2Vec3();
	this.impulse3 = new b2Vec3();
	this.impulse2 = new b2Vec2();

	this.reduced = new b2Vec2();

	this.m_localAnchor1 = def.localAnchorA.Copy();
	this.m_localAnchor2 = def.localAnchorB.Copy();

	this.m_mass = new b2Mat33();

	this.m_referenceAngle = def.referenceAngle;
	this.m_motorImpulse = 0;

	this.m_lowerAngle = def.lowerAngle;
	this.m_upperAngle = def.upperAngle;

	this.m_maxMotorTorque = def.maxMotorTorque;

	this.m_motorSpeed = def.motorSpeed;
	this.m_enableLimit = def.enableLimit;
	this.m_enableMotor = def.enableMotor;

	this.m_limitState = b2Joint.e_inactiveLimit;
}

b2RevoluteJoint.prototype = extend(new b2Joint(), {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	},

	GetReactionTorque : function (inv_dt) {
		return this.m_impulse.z * (inv_dt || 0);
	},

	GetJointAngle : function () {
		return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
	},

	GetJointSpeed : function () {
		return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
	},

	IsLimitEnabled : function () {
		return this.m_enableLimit;
	},

	EnableLimit : function (flag) {
		this.m_enableLimit = flag;
	},

	GetLowerLimit : function () {
		return this.m_lowerAngle;
	},

	GetUpperLimit : function () {
		return this.m_upperAngle;
	},

	SetLimits : function (lower, upper) {
		this.m_lowerAngle = lower || 0;
		this.m_upperAngle = upper || 0;
	},

	IsMotorEnabled : function () {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		return this.m_enableMotor;
	},

	EnableMotor : function (flag) {
		this.m_enableMotor = flag;
	},

	SetMotorSpeed : function (speed) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed || 0;
	},

	GetMotorSpeed : function () {
		return this.m_motorSpeed;
	},

	SetMaxMotorTorque : function (torque) {
		this.m_maxMotorTorque = torque || 0;
	},

	GetMotorTorque : function () {
		return this.m_maxMotorTorque;
	},

	InitVelocityConstraints : function (step) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			tMat,
			tX = 0,
			r1X, r1Y,
			r2X, r2Y,
			m1, m2,
			i1, i2,
			jointAngle,
			PX, PY;



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

		m1 = bA.m_invMass;
		m2 = bB.m_invMass;

		i1 = bA.m_invI;
		i2 = bB.m_invI;

		this.m_mass.col1.x = m1 + m2 + r1Y * r1Y * i1 + r2Y * r2Y * i2;
		this.m_mass.col2.x = (-r1Y * r1X * i1) - r2Y * r2X * i2;
		this.m_mass.col3.x = (-r1Y * i1) - r2Y * i2;

		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = m1 + m2 + r1X * r1X * i1 + r2X * r2X * i2;
		this.m_mass.col3.y = r1X * i1 + r2X * i2;

		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = i1 + i2;

		this.m_motorMass = 1 / (i1 + i2);

		if (!this.m_enableMotor) {
			this.m_motorImpulse = 0;
		}

		if (this.m_enableLimit) {
			jointAngle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2 * b2Settings.b2_angularSlop) {
				this.m_limitState = b2Joint.e_equalLimits;
			} else if (jointAngle <= this.m_lowerAngle) {
				if (this.m_limitState !== b2Joint.e_atLowerLimit) {
					this.m_impulse.z = 0;
				}
				this.m_limitState = b2Joint.e_atLowerLimit;
			} else if (jointAngle >= this.m_upperAngle) {
				if (this.m_limitState !== b2Joint.e_atUpperLimit) {
					this.m_impulse.z = 0;
				}
				this.m_limitState = b2Joint.e_atUpperLimit;
			} else {
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0;
			}
		} else {
			this.m_limitState = b2Joint.e_inactiveLimit;
		}

		if (step.warmStarting) {
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_motorImpulse *= step.dtRatio;

			PX = this.m_impulse.x;
			PY = this.m_impulse.y;

			bA.m_linearVelocity.x -= m1 * PX;
			bA.m_linearVelocity.y -= m1 * PY;

			bB.m_linearVelocity.x += m2 * PX;
			bB.m_linearVelocity.y += m2 * PY;

			bA.m_angularVelocity -= i1 * ((r1X * PY - r1Y * PX) + this.m_motorImpulse + this.m_impulse.z);
			bB.m_angularVelocity += i2 * ((r2X * PY - r2Y * PX) + this.m_motorImpulse + this.m_impulse.z);
		} else {
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0;
		}
	},

	SolveVelocityConstraints : function (step) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			tMat,
			tX = 0,
			newImpulse = 0,
			r1X = 0,
			r1Y = 0,
			r2X = 0,
			r2Y = 0,
			v1 = bA.m_linearVelocity,
			w1 = bA.m_angularVelocity,
			v2 = bB.m_linearVelocity,
			w2 = bB.m_angularVelocity,
			m1 = bA.m_invMass,
			m2 = bB.m_invMass,
			i1 = bA.m_invI,
			i2 = bB.m_invI,
			Cdot,
			impulse, oldImpulse, maxImpulse,
			Cdot1X, Cdot1Y,
			Cdot2,
			CdotX, CdotY;


		if (this.m_enableMotor && this.m_limitState !== b2Joint.e_equalLimits) {
			Cdot = w2 - w1 - this.m_motorSpeed;

			impulse = -this.m_motorMass * -Cdot;
			oldImpulse = this.m_motorImpulse;
			maxImpulse = step.dt * this.m_maxMotorTorque;

			this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);

			impulse = this.m_motorImpulse - oldImpulse;

			w1 -= i1 * impulse;
			w2 += i2 * impulse;
		}

		if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
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

			Cdot1X = v2.x + ((-w2 * r2Y)) - v1.x - ((-w1 * r1Y));
			Cdot1Y = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);
			Cdot2 = w2 - w1;

			this.m_mass.Solve33(this.impulse3, -Cdot1X, -Cdot1Y, -Cdot2);

			if (this.m_limitState === b2Joint.e_equalLimits) {
				this.m_impulse.Add(this.impulse3);
			} else if (this.m_limitState === b2Joint.e_atLowerLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse < 0) {
					this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);

					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = -this.m_impulse.z;

					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0;
				}
			} else if (this.m_limitState === b2Joint.e_atUpperLimit) {
				newImpulse = this.m_impulse.z + this.impulse3.z;
				if (newImpulse > 0) {
					this.m_mass.Solve22(this.reduced, -Cdot1X, -Cdot1Y);

					this.impulse3.x = this.reduced.x;
					this.impulse3.y = this.reduced.y;
					this.impulse3.z = (-this.m_impulse.z);

					this.m_impulse.x += this.reduced.x;
					this.m_impulse.y += this.reduced.y;
					this.m_impulse.z = 0;
				}
			}
			v1.x -= m1 * this.impulse3.x;
			v1.y -= m1 * this.impulse3.y;

			v2.x += m2 * this.impulse3.x;
			v2.y += m2 * this.impulse3.y;

			w1 -= i1 * (r1X * this.impulse3.y - r1Y * this.impulse3.x + this.impulse3.z);
			w2 += i2 * (r2X * this.impulse3.y - r2Y * this.impulse3.x + this.impulse3.z);
		} else {
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

			CdotX = v2.x + (-w2 * r2Y) - v1.x - (-w1 * r1Y);
			CdotY = v2.y + (w2 * r2X) - v1.y - (w1 * r1X);

			this.m_mass.Solve22(this.impulse2, -CdotX, -CdotY);

			this.m_impulse.x += this.impulse2.x;
			this.m_impulse.y += this.impulse2.y;

			v1.x -= m1 * this.impulse2.x;
			v1.y -= m1 * this.impulse2.y;

			v2.x += m2 * this.impulse2.x;
			v2.y += m2 * this.impulse2.y;

			w1 -= i1 * (r1X * this.impulse2.y - r1Y * this.impulse2.x);
			w2 += i2 * (r2X * this.impulse2.y - r2Y * this.impulse2.x);
		}

		bA.m_linearVelocity.SetV(v1);
		bB.m_linearVelocity.SetV(v2);

		bA.m_angularVelocity = w1;
		bB.m_angularVelocity = w2;
	},

	SolvePositionConstraints : function (baumgarte) {
		var oldLimitImpulse = 0,
			C = 0,
			tMat,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			angularError = 0,
			positionError = 0,
			tX = 0,
			impulseX = 0,
			impulseY = 0,
			k_allowedStretch = 10 * b2Settings.b2_linearSlop,
			angle,
			limitImpulse,
			r1X, r1Y,
			r2X, r2Y,
			CX, CY,
			CLengthSquared, CLength,
			invMass1, invMass2,
			invI1, invI2,
			uX, uY,
			k, m,
			k_beta;

		baumgarte = baumgarte || 0;

		if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
			angle = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
			limitImpulse = 0;

			if (this.m_limitState === b2Joint.e_equalLimits) {
				C = b2Math.Clamp(angle - this.m_lowerAngle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * C;
				angularError = Math.abs(C);
			} else if (this.m_limitState === b2Joint.e_atLowerLimit) {
				C = angle - this.m_lowerAngle;

				angularError = -C;

				C = b2Math.Clamp(C + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0);

				limitImpulse = -this.m_motorMass * C;
			} else if (this.m_limitState === b2Joint.e_atUpperLimit) {
				C = angle - this.m_upperAngle;

				angularError = C;

				C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0, b2Settings.b2_maxAngularCorrection);

				limitImpulse = -this.m_motorMass * C;
			}
			bA.m_sweep.a -= bA.m_invI * limitImpulse;
			bB.m_sweep.a += bB.m_invI * limitImpulse;

			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}

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

		CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		CLengthSquared = CX * CX + CY * CY;
		CLength = Math.sqrt(CLengthSquared);

		positionError = CLength;

		invMass1 = bA.m_invMass;
		invMass2 = bB.m_invMass;

		invI1 = bA.m_invI;
		invI2 = bB.m_invI;

		if (CLengthSquared > k_allowedStretch * k_allowedStretch) {
			uX = CX / CLength;
			uY = CY / CLength;

			k = invMass1 + invMass2;
			m = 1 / k;

			impulseX = -m * CX;
			impulseY = -m * CY;

			k_beta = 0.5;

			bA.m_sweep.c.x -= k_beta * invMass1 * impulseX;
			bA.m_sweep.c.y -= k_beta * invMass1 * impulseY;

			bB.m_sweep.c.x += k_beta * invMass2 * impulseX;
			bB.m_sweep.c.y += k_beta * invMass2 * impulseY;

			CX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
			CY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
		}

		this.K1.col1.x = invMass1 + invMass2;
		this.K1.col2.x = 0;
		this.K1.col1.y = 0;
		this.K1.col2.y = invMass1 + invMass2;

		this.K2.col1.x = invI1 * r1Y * r1Y;
		this.K2.col2.x = -invI1 * r1X * r1Y;
		this.K2.col1.y = -invI1 * r1X * r1Y;
		this.K2.col2.y = invI1 * r1X * r1X;

		this.K3.col1.x = invI2 * r2Y * r2Y;
		this.K3.col2.x = -invI2 * r2X * r2Y;
		this.K3.col1.y = -invI2 * r2X * r2Y;
		this.K3.col2.y = invI2 * r2X * r2X;

		this.K.SetM(this.K1);
		this.K.AddM(this.K2);
		this.K.AddM(this.K3);
		this.K.Solve(b2RevoluteJoint.tImpulse, -CX, -CY);

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

		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
});
