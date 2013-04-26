function b2FrictionJoint(def) {
	b2Joint.apply(this, arguments);
	this.m_localAnchorA = new b2Vec2(def.localAnchorA.x, def.localAnchorA.y);
	this.m_localAnchorB = new b2Vec2(def.localAnchorB.x, def.localAnchorB.y);
	this.m_linearMass = new b2Mat22();
	this.m_linearImpulse = new b2Vec2();
	this.m_angularMass = 0;
	this.m_angularImpulse = 0;
	this.m_maxForce = def.maxForce;
	this.m_maxTorque = def.maxTorque;
}

Box2D.b2FrictionJoint = b2FrictionJoint;

inherit(b2Joint, b2FrictionJoint);

b2FrictionJoint.prototype = {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	},

	GetReactionTorque : function (inv_dt) {
		return this.m_angularImpulse * (inv_dt || 0);
	},

	SetMaxForce : function (force) {
		this.m_maxForce = force || 0;
	},

	GetMaxForce : function () {
		return this.m_maxForce;
	},

	SetMaxTorque : function (torque) {
		this.m_maxTorque = torque || 0;
	},

	GetMaxTorque : function () {
		return this.m_maxTorque;
	},

	InitVelocityConstraints : function (step) {
		var tMat,
			tX = 0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			rAX, rAY,
			rBX, rBY,
			mA, mB,
			iA, iB,
			K, P;

		tMat = bA.m_xf.R;
		rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;

		tMat = bB.m_xf.R;
		rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		mA = bA.m_invMass;
		mB = bB.m_invMass;

		iA = bA.m_invI;
		iB = bB.m_invI;

		K = new b2Mat22();
		K.col1.x = mA + mB;
		K.col2.x = 0;
		K.col1.y = 0;
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

		if (this.m_angularMass > 0) {
			this.m_angularMass = 1 / this.m_angularMass;
		}

		if (step.warmStarting) {
			this.m_linearImpulse.x *= step.dtRatio;
			this.m_linearImpulse.y *= step.dtRatio;
			this.m_angularImpulse *= step.dtRatio;

			P = this.m_linearImpulse;

			bA.m_linearVelocity.x -= mA * P.x;
			bA.m_linearVelocity.y -= mA * P.y;

			bB.m_linearVelocity.x += mB * P.x;
			bB.m_linearVelocity.y += mB * P.y;

			bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);
			bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
		} else {
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0;
		}
	},

	SolveVelocityConstraints : function (step) {
		var tMat,
			tX = 0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			vA = bA.m_linearVelocity,
			wA = bA.m_angularVelocity,
			vB = bB.m_linearVelocity,
			wB = bB.m_angularVelocity,
			mA = bA.m_invMass,
			mB = bB.m_invMass,
			iA = bA.m_invI,
			iB = bB.m_invI,
			rAX, rAY,
			rBX, rBY,
			maxImpulse,
			Cdot,
			impulse,
			oldImpulse,
			CdotX, CdotY,
			impulseV,
			oldImpulseV;

		tMat = bA.m_xf.R;
		rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;

		tMat = bB.m_xf.R;
		rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		maxImpulse = 0;

		Cdot = wB - wA;

		impulse = -this.m_angularMass * Cdot;
		oldImpulse = this.m_angularImpulse;
		maxImpulse = step.dt * this.m_maxTorque;

		this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = this.m_angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;

		CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
		CdotY = vB.y + wB * rBX - vA.y - wA * rAX;

		impulseV = b2Math.MulMV(this.m_linearMass, new b2Vec2(-CdotX, -CdotY));
		oldImpulseV = this.m_linearImpulse.Copy();

		this.m_linearImpulse.Add(impulseV);
		maxImpulse = step.dt * this.m_maxForce;

		if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
			this.m_linearImpulse.Normalize();
			this.m_linearImpulse.Multiply(maxImpulse);
		}

		impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);

		vA.x -= mA * impulseV.x;
		vA.y -= mA * impulseV.y;

		vB.x += mB * impulseV.x;
		vB.y += mB * impulseV.y;

		wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);
		wB += iB * (rBX * impulseV.y - rBY * impulseV.x);

		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	},

	SolvePositionConstraints : function (baumgarte) {
		return true;
	}
};
