function b2GearJoint(def) {
	var type1 = def.joint1.m_type,
		type2 = def.joint2.m_type,
		joint1 = def.joint1,
		joint2 = def.joint2,
		coordinate1 = 0,
		coordinate2 = 0;

	b2Joint.apply(this, arguments);

	this.m_groundAnchor1 = new b2Vec2(joint1.m_localAnchor1.x, joint1.m_localAnchor1.y);
	this.m_groundAnchor2 = new b2Vec2(joint2.m_localAnchor1.x, joint2.m_localAnchor1.y);
	this.m_localAnchor1 = new b2Vec2(joint1.m_localAnchor2.x, joint1.m_localAnchor2.y);
	this.m_localAnchor2 = new b2Vec2(joint2.m_localAnchor2.x, joint2.m_localAnchor2.y);
	this.m_J = new b2Jacobian();

	this.m_ground1 = joint1.GetBodyA();
	this.m_bodyA = joint1.GetBodyB();

	this.m_ground2 = joint2.GetBodyA();
	this.m_bodyB = joint2.GetBodyB();

	if (type1 === b2Joint.e_revoluteJoint) {
		this.m_revolute1 = joint1;
		coordinate1 = this.m_revolute1.GetJointAngle();
	} else {
		this.m_prismatic1 = joint1;
		coordinate1 = this.m_prismatic1.GetJointTranslation();
	}

	if (type2 === b2Joint.e_revoluteJoint) {
		this.m_revolute2 = joint2;
		coordinate2 = joint2.GetJointAngle();
	} else {
		this.m_prismatic2 = joint2;
		coordinate2 = joint2.GetJointTranslation();
	}

	this.m_ratio = def.ratio;
	this.m_constant = coordinate1 + this.m_ratio * coordinate2;
	this.m_impulse = 0;
}

Box2D.b2GearJoint = b2GearJoint;

inherit(b2Joint, b2GearJoint);

b2GearJoint.prototype = {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
	},

	GetReactionTorque : function (inv_dt) {
		inv_dt = inv_dt || 0;

		var tMat = this.m_bodyB.m_xf.R,
			rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x,
			rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y,
			tX = tMat.col1.x * rX + tMat.col2.x * rY,
			PX = this.m_impulse * this.m_J.linearB.x,
			PY = this.m_impulse * this.m_J.linearB.y;

		rY = tMat.col1.y * rX + tMat.col2.y * rY,
		rX = tX;

		return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
	},

	GetRatio : function () {
		return this.m_ratio;
	},

	SetRatio : function (ratio) {
		this.m_ratio = ratio || 0;
	},

	b2GearJoint : function (def) {
		this.__super.b2Joint.call(this, def);
	},

	InitVelocityConstraints : function (step) {
		var g1 = this.m_ground1,
			g2 = this.m_ground2,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			ugX = 0,
			ugY = 0,
			rX = 0,
			rY = 0,
			tMat,
			tVec,
			crug = 0,
			tX = 0,
			K = 0;

		this.m_J.SetZero();

		if (this.m_revolute1) {
			this.m_J.angularA = -1;
			K += bA.m_invI;
		} else {
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

			this.m_J.linearA.Set(-ugX, -ugY);
			this.m_J.angularA = -crug;

			K += bA.m_invMass + bA.m_invI * crug * crug;
		}

		if (this.m_revolute2) {
			this.m_J.angularB = -this.m_ratio;
			K += this.m_ratio * this.m_ratio * bB.m_invI;
		} else {
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

			this.m_J.linearB.Set(-this.m_ratio * ugX, -this.m_ratio * ugY);
			this.m_J.angularB = -this.m_ratio * crug;

			K += this.m_ratio * this.m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
		}

		this.m_mass = K > 0 ? 1 / K : 0;

		if (step.warmStarting) {
			bA.m_linearVelocity.x += bA.m_invMass * this.m_impulse * this.m_J.linearA.x;
			bA.m_linearVelocity.y += bA.m_invMass * this.m_impulse * this.m_J.linearA.y;

			bB.m_linearVelocity.x += bB.m_invMass * this.m_impulse * this.m_J.linearB.x;
			bB.m_linearVelocity.y += bB.m_invMass * this.m_impulse * this.m_J.linearB.y;

			bA.m_angularVelocity += bA.m_invI * this.m_impulse * this.m_J.angularA;
			bB.m_angularVelocity += bB.m_invI * this.m_impulse * this.m_J.angularB;
		} else {
			this.m_impulse = 0;
		}
	},

	SolveVelocityConstraints : function (step) {
		var bA = this.m_bodyA,
			bB = this.m_bodyB,
			Cdot = this.m_J.Compute(bA.m_linearVelocity, bA.m_angularVelocity, bB.m_linearVelocity, bB.m_angularVelocity),
			impulse = -this.m_mass * Cdot;

		this.m_impulse += impulse;

		bA.m_linearVelocity.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_linearVelocity.y += bA.m_invMass * impulse * this.m_J.linearA.y;

		bB.m_linearVelocity.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_linearVelocity.y += bB.m_invMass * impulse * this.m_J.linearB.y;

		bA.m_angularVelocity += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_angularVelocity += bB.m_invI * impulse * this.m_J.angularB;
	},

	SolvePositionConstraints : function (baumgarte) {
		baumgarte = baumgarte || 0;
		var linearError = 0.0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			coordinate1 = 0,
			coordinate2 = 0,
			C,
			impulse;

		if (this.m_revolute1) {
			coordinate1 = this.m_revolute1.GetJointAngle();
		} else {
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		if (this.m_revolute2) {
			coordinate2 = this.m_revolute2.GetJointAngle();
		} else {
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}

		C = this.m_constant - (coordinate1 + this.m_ratio * coordinate2);

		impulse = (-this.m_mass * C);

		bA.m_sweep.c.x += bA.m_invMass * impulse * this.m_J.linearA.x;
		bA.m_sweep.c.y += bA.m_invMass * impulse * this.m_J.linearA.y;

		bB.m_sweep.c.x += bB.m_invMass * impulse * this.m_J.linearB.x;
		bB.m_sweep.c.y += bB.m_invMass * impulse * this.m_J.linearB.y;

		bA.m_sweep.a += bA.m_invI * impulse * this.m_J.angularA;
		bB.m_sweep.a += bB.m_invI * impulse * this.m_J.angularB;

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return linearError < b2Settings.b2_linearSlop;
	}
};
