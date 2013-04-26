function b2WeldJoint(def) {
	b2Joint.apply(this, arguments);
	this.m_localAnchorA = new b2Vec2(def.localAnchorA.x, def.localAnchorA.y);
	this.m_localAnchorB = new b2Vec2(def.localAnchorB.x, def.localAnchorB.y);
	this.m_impulse = new b2Vec3();
	this.m_mass = new b2Mat33();
	this.m_referenceAngle = def.referenceAngle;
}

Box2D.b2WeldJoint = b2WeldJoint;

inherit(b2Joint, b2WeldJoint);

b2WeldJoint.prototype = {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	},

	GetReactionTorque : function (inv_dt) {
		return this.m_impulse.z * (inv_dt || 0);
	},

	InitVelocityConstraints : function (step) {
		var tMat,
			tX = 0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			mA, mB,
			iA, iB,
			rAX, rAY,
			rBX, rBY;


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

			bB.m_linearVelocity.x += mB * this.m_impulse.x;
			bB.m_linearVelocity.y += mB * this.m_impulse.y;

			bA.m_angularVelocity -= iA * (rAX * this.m_impulse.y - rAY * this.m_impulse.x + this.m_impulse.z);
			bB.m_angularVelocity += iB * (rBX * this.m_impulse.y - rBY * this.m_impulse.x + this.m_impulse.z);
		} else {
			this.m_impulse.SetZero();
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
			Cdot1X, Cdot1Y,
			Cdot2,
			impulse;

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

		Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
		Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;

		Cdot2 = wB - wA;

		impulse = new b2Vec3();

		this.m_mass.Solve33(impulse, -Cdot1X, -Cdot1Y, -Cdot2);
		this.m_impulse.Add(impulse);

		vA.x -= mA * impulse.x;
		vA.y -= mA * impulse.y;


		vB.x += mB * impulse.x;
		vB.y += mB * impulse.y;

		wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
		wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);

		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	},

	SolvePositionConstraints : function (baumgarte) {
		var tMat,
			tX = 0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			rAX, rAY,
			rBX, rBY,
			mA, mB,
			iA, iB,
			C1X, C1Y,
			C2,
			k_allowedStretch = 10 * b2Settings.b2_linearSlop,
			positionError,
			angularError,
			impulse;

		baumgarte = baumgarte || 0;

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

		C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX;
		C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY;

		C2 = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;

		positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
		angularError = Math.abs(C2);

		if (positionError > k_allowedStretch) {
			iA *= 1; // TODO: Remove these??
			iB *= 1; // TODO: Remove these??
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

		impulse = new b2Vec3();
		this.m_mass.Solve33(impulse, -C1X, -C1Y, -C2);

		bA.m_sweep.c.x -= mA * impulse.x;
		bA.m_sweep.c.y -= mA * impulse.y;

		bB.m_sweep.c.x += mB * impulse.x;
		bB.m_sweep.c.y += mB * impulse.y;

		bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);
		bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
	}
};
