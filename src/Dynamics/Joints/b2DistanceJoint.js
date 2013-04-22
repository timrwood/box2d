function b2DistanceJoint(def) {
	b2Joint.apply(this, arguments);
	this.m_localAnchor1 = new b2Vec2(def.localAnchorA.x, def.localAnchorA.y);
	this.m_localAnchor2 = new b2Vec2(def.localAnchorB.x, def.localAnchorB.y);
	this.m_u = new b2Vec2();
	this.m_length = def.length;
	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;
	this.m_impulse = 0;
	this.m_gamma = 0;
	this.m_bias = 0;
}

b2DistanceJoint.prototype = extend(new b2Joint(), {
	GetAnchorA : function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
	},

	GetReactionTorque : function (inv_dt) {
		return 0;
	},

	GetLength : function () {
		return this.m_length;
	},

	SetLength : function (length) {
		this.m_length = length || 0;
	},

	GetFrequency : function () {
		return this.m_frequencyHz;
	},

	SetFrequency : function (hz) {
		this.m_frequencyHz = hz || 0;
	},

	GetDampingRatio : function () {
		return this.m_dampingRatio;
	},

	SetDampingRatio : function (ratio) {
		this.m_dampingRatio = ratio || 0;
	},

	b2DistanceJoint : function (def) {
	},

	InitVelocityConstraints : function (step) {
		var tMat,
			tX = 0,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x,
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y,
			PX, PY,
			r2X, r2Y,
			length,
			cr1u, cr2u,
			invMass,
			C,
			omega,
			d,
			k;

		tMat = bA.m_xf.R;

		tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
		r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
		r1X = tX;

		tMat = bB.m_xf.R;

		r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);

		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);

		if (length > b2Settings.b2_linearSlop) {
			this.m_u.Multiply(1 / length);
		} else {
			this.m_u.SetZero();
		}

		cr1u = r1X * this.m_u.y - r1Y * this.m_u.x;
		cr2u = r2X * this.m_u.y - r2Y * this.m_u.x;

		invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;

		this.m_mass = invMass ? 1 / invMass : 0;

		if (this.m_frequencyHz > 0) {
			C = length - this.m_length;
			omega = 2 * Math.PI * this.m_frequencyHz;
			d = 2 * this.m_mass * this.m_dampingRatio * omega;
			k = this.m_mass * omega * omega;

			this.m_gamma = step.dt * (d + step.dt * k);
			this.m_gamma = this.m_gamma ? 1 / this.m_gamma : 0;

			this.m_bias = C * step.dt * k * this.m_gamma;

			this.m_mass = invMass + this.m_gamma;
			this.m_mass = this.m_mass ? 1 / this.m_mass : 0;
		}

		if (step.warmStarting) {
			this.m_impulse *= step.dtRatio;

			PX = this.m_impulse * this.m_u.x;
			PY = this.m_impulse * this.m_u.y;

			bA.m_linearVelocity.x -= bA.m_invMass * PX;
			bA.m_linearVelocity.y -= bA.m_invMass * PY;

			bB.m_linearVelocity.x += bB.m_invMass * PX;
			bB.m_linearVelocity.y += bB.m_invMass * PY;

			bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
			bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
		} else {
			this.m_impulse = 0;
		}
	},

	SolveVelocityConstraints : function (step) {
		var tMat,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x,
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y,
			tX,
			r2X, r2Y,
			v1X, v1Y,
			v2X, v2Y,
			Cdot,
			impulse,
			PX, PY;

		tMat = bA.m_xf.R;

		tX = tMat.col1.x * r1X + tMat.col2.x * r1Y;
		r1Y = tMat.col1.y * r1X + tMat.col2.y * r1Y;
		r1X = tX;

		tMat = bB.m_xf.R;

		r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;

		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;

		v1X = bA.m_linearVelocity.x - bA.m_angularVelocity * r1Y;
		v1Y = bA.m_linearVelocity.y + bA.m_angularVelocity * r1X;

		v2X = bB.m_linearVelocity.x - bB.m_angularVelocity * r2Y;
		v2Y = bB.m_linearVelocity.y + bB.m_angularVelocity * r2X;

		Cdot = this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y);
		impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);

		this.m_impulse += impulse;

		PX = impulse * this.m_u.x;
		PY = impulse * this.m_u.y;

		bA.m_linearVelocity.x -= bA.m_invMass * PX;
		bA.m_linearVelocity.y -= bA.m_invMass * PY;

		bB.m_linearVelocity.x += bB.m_invMass * PX;
		bB.m_linearVelocity.y += bB.m_invMass * PY;

		bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
		bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
	},

	SolvePositionConstraints : function (baumgarte) {
		var tMat,
			bA = this.m_bodyA,
			bB = this.m_bodyB,
			tX,
			r1X, r1Y,
			r2X, r2Y,
			dX, dY,
			length,
			C,
			impulse,
			PX, PY;

		baumgarte = baumgarte || 0;

		if (this.m_frequencyHz > 0) {
			return true;
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

		dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
		dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;

		length = Math.sqrt(dX * dX + dY * dY);
		dX /= length;
		dY /= length;

		C = length - this.m_length;
		C = b2Math.Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);

		impulse = -this.m_mass * C;

		this.m_u.Set(dX, dY);

		PX = impulse * this.m_u.x;
		PY = impulse * this.m_u.y;

		bA.m_sweep.c.x -= bA.m_invMass * PX;
		bA.m_sweep.c.y -= bA.m_invMass * PY;

		bB.m_sweep.c.x += bB.m_invMass * PX;
		bB.m_sweep.c.y += bB.m_invMass * PY;

		bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
		bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return Math.abs(C) < b2Settings.b2_linearSlop;
	}
});
