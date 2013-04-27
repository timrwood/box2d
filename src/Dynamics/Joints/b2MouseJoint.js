function b2MouseJoint(def) {
	var tX, tY, tMat;

	b2Joint.apply(this, arguments);

	this.K = new b2Mat22();
	this.K1 = new b2Mat22();
	this.K2 = new b2Mat22();

	this.m_localAnchor = new b2Vec2();
	this.m_impulse = new b2Vec2();

	this.m_mass = new b2Mat22();
	this.m_C = new b2Vec2();

	this.m_target = def.target.Copy();

	tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
	tY = this.m_target.y - this.m_bodyB.m_xf.position.y;

	tMat = this.m_bodyB.m_xf.R;

	this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
	this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);

	this.m_maxForce = def.maxForce;

	this.m_impulse.SetZero();
	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;

	this.m_beta = 0;
	this.m_gamma = 0;
}

Box2D.b2MouseJoint = b2MouseJoint;

inherit(b2Joint, b2MouseJoint);

b2MouseJoint.prototype = {
	GetAnchorA : function () {
		return this.m_target;
	},

	GetAnchorB : function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
	},

	GetReactionForce : function (inv_dt) {
		inv_dt = inv_dt || 0;
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	},

	GetReactionTorque : function (inv_dt) {
		return 0;
	},

	GetTarget : function () {
		return this.m_target;
	},

	SetTarget : function (target) {
		this.m_bodyB.SetAwake(true);
		this.m_target = target;
	},

	GetMaxForce : function () {
		return this.m_maxForce;
	},

	SetMaxForce : function (maxForce) {
		this.m_maxForce = maxForce || 0;
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

	InitVelocityConstraints : function (step) {
		var b = this.m_bodyB,
			mass = b.GetMass(),
			omega = 2 * Math.PI * this.m_frequencyHz,
			d = 2 * mass * this.m_dampingRatio * omega,
			k = mass * omega * omega,
			tMat,
			rX, rY,
			tX,
			invMass,
			invI;

		this.m_gamma = step.dt * (d + step.dt * k);
		this.m_gamma = this.m_gamma ? 1 / this.m_gamma : 0;
		this.m_beta = step.dt * k * this.m_gamma;

		tMat = b.m_xf.R;
		rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		invMass = b.m_invMass;
		invI = b.m_invI;

		this.K1.col1.x = invMass;
		this.K1.col2.x = 0;
		this.K1.col1.y = 0;
		this.K1.col2.y = invMass;

		this.K2.col1.x = invI * rY * rY;
		this.K2.col2.x = -invI * rX * rY;
		this.K2.col1.y = -invI * rX * rY;
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
	},

	SolveVelocityConstraints : function (step) {
		var b = this.m_bodyB,
			tMat,
			tX = 0,
			tY = 0,
			rX, rY,
			CdotX, CdotY,
			impulseX, impulseY,
			oldImpulseX, oldImpulseY,
			maxImpulse;

		tMat = b.m_xf.R;
		rX = this.m_localAnchor.x - b.m_sweep.localCenter.x;
		rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;

		tX = (tMat.col1.x * rX + tMat.col2.x * rY);
		rY = (tMat.col1.y * rX + tMat.col2.y * rY);
		rX = tX;

		CdotX = b.m_linearVelocity.x - b.m_angularVelocity * rY;
		CdotY = b.m_linearVelocity.y + b.m_angularVelocity * rX;

		tMat = this.m_mass;

		tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
		tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;

		impulseX = -(tMat.col1.x * tX + tMat.col2.x * tY);
		impulseY = -(tMat.col1.y * tX + tMat.col2.y * tY);

		oldImpulseX = this.m_impulse.x;
		oldImpulseY = this.m_impulse.y;

		this.m_impulse.x += impulseX;
		this.m_impulse.y += impulseY;

		maxImpulse = step.dt * this.m_maxForce;

		if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
			this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
		}

		impulseX = this.m_impulse.x - oldImpulseX;
		impulseY = this.m_impulse.y - oldImpulseY;

		b.m_linearVelocity.x += b.m_invMass * impulseX;
		b.m_linearVelocity.y += b.m_invMass * impulseY;

		b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
	},

	SolvePositionConstraints : function (baumgarte) {
		return true;
	}
};
