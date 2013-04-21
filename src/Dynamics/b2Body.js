
function b2Body() {
	var tMat, tVec;

	this.m_xf = new b2Transform();
	this.m_sweep = new b2Sweep();
	this.m_linearVelocity = new b2Vec2();
	this.m_force = new b2Vec2();

	this.m_flags = 0;
	if (bd.bullet) {
		this.m_flags |= b2Body.e_bulletFlag;
	}

	if (bd.fixedRotation) {
		this.m_flags |= b2Body.e_fixedRotationFlag;
	}

	if (bd.allowSleep) {
		this.m_flags |= b2Body.e_allowSleepFlag;
	}

	if (bd.awake) {
		this.m_flags |= b2Body.e_awakeFlag;
	}

	if (bd.active) {
		this.m_flags |= b2Body.e_activeFlag;
	}

	this.m_world = world;

	this.m_xf.position.SetV(bd.position);
	this.m_xf.R.Set(bd.angle);

	this.m_sweep.localCenter.SetZero();
	this.m_sweep.t0 = 1.0;
	this.m_sweep.a0 = this.m_sweep.a = bd.angle;

	tMat = this.m_xf.R;
	tVec = this.m_sweep.localCenter;
	this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
	this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	this.m_sweep.c.x += this.m_xf.position.x;
	this.m_sweep.c.y += this.m_xf.position.y;
	this.m_sweep.c0.SetV(this.m_sweep.c);

	this.m_jointList = null;
	this.m_controllerList = null;
	this.m_contactList = null;
	this.m_controllerCount = 0;
	this.m_prev = null;
	this.m_next = null;

	this.m_linearVelocity.SetV(bd.linearVelocity);
	this.m_angularVelocity = bd.angularVelocity;
	this.m_linearDamping = bd.linearDamping;
	this.m_angularDamping = bd.angularDamping;
	this.m_torque = 0;
	this.m_sleepTime = 0;

	this.m_type = bd.type;
	if (this.m_type == b2Body.b2_dynamicBody) {
		this.m_mass = 1;
		this.m_invMass = 1;
	} else {
		this.m_mass = 0;
		this.m_invMass = 0;
	}

	this.m_I = 0;
	this.m_invI = 0;
	this.m_inertiaScale = bd.inertiaScale;
	this.m_userData = bd.userData;
	this.m_fixtureList = null;
	this.m_fixtureCount = 0;
}

b2Body.prototype = {
	connectEdges : function (s1, s2, angle1) {
		angle1 = angle1 || 0;

		var dv1 = s1.GetDirectionVector(),
			dv2 = s2.GetDirectionVector(),
			angle2 = Math.atan2(dv2.y, dv2.x),
			coreOffset = Math.tan((angle2 - angle1) * 0.5),
			core = b2Math.MulFV(coreOffset, dv2), // TODO: Reuse b2Vec2 ?
			cornerDir,
			convex;

		core = b2Math.SubtractVV(core, s2.GetNormalVector()); // TODO: Reuse b2Vec2 ?
		core = b2Math.MulFV(b2Settings.b2_toiSlop, core); // TODO: Reuse b2Vec2 ?
		core = b2Math.AddVV(core, s2.GetVertex1()); // TODO: Reuse b2Vec2 ?

		cornerDir = b2Math.AddVV(dv1, dv2);
		cornerDir.Normalize();

		convex = b2Math.Dot(dv1, s2.GetNormalVector()) > 0;

		s1.SetNextEdge(s2, core, cornerDir, convex);
		s2.SetPrevEdge(s1, core, cornerDir, convex);

		return angle2;
	},

	CreateFixture : function (def) {
		if (this.m_world.IsLocked()) {
			return null;
		}
		var fixture = new b2Fixture();

		fixture.Create(this, this.m_xf, def);
		if (this.m_flags & b2Body.e_activeFlag) {
			fixture.CreateProxy(this.m_world.m_contactManager.m_broadPhase, this.m_xf);
		}

		fixture.m_next = this.m_fixtureList;
		this.m_fixtureList = fixture;
		this.m_fixtureCount++;

		fixture.m_body = this;
		if (fixture.m_density > 0) {
			this.ResetMassData();
		}

		this.m_world.m_flags |= b2World.e_newFixture;
		return fixture;
	},

	CreateFixture2 : function (shape, density) {
		density = density || 0;
		var def = new b2FixtureDef();
		def.shape = shape;
		def.density = density;
		return this.CreateFixture(def);
	},

	DestroyFixture : function (fixture) {
		if (this.m_world.IsLocked()) {
			return;
		}
		var node = this.m_fixtureList,
			ppF,
			found = false,
			edge,
			c;

		while (node) {
			if (node === fixture) {
				if (ppF) {
					ppF.m_next = fixture.m_next;
				} else {
					this.m_fixtureList = fixture.m_next;
				}
				found = true;
				break;
			}
			ppF = node;
			node = node.m_next;
		}

		edge = this.m_contactList;
		while (edge) {
			c = edge.contact;
			edge = edge.next;
			if (fixture === c.GetFixtureA() || fixture === c.GetFixtureB()) {
				this.m_world.m_contactManager.Destroy(c);
			}
		}

		if (this.m_flags & b2Body.e_activeFlag) {
			fixture.DestroyProxy(this.m_world.m_contactManager.m_broadPhase);
		}

		fixture.Destroy();
		fixture.m_body = null;
		fixture.m_next = null;
		this.m_fixtureCount--;
		this.ResetMassData();
	},

	SetPositionAndAngle : function (position, angle) {
		angle = angle || 0;

		var broadPhase = this.m_world.m_contactManager.m_broadPhase,
			f = this.m_fixtureList,
			tMat, tVec;

		if (this.m_world.IsLocked()) {
			return;
		}

		this.m_xf.R.Set(angle);
		this.m_xf.position.SetV(position);

		tMat = this.m_xf.R;
		tVec = this.m_sweep.localCenter;

		this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		this.m_sweep.c.x += this.m_xf.position.x;
		this.m_sweep.c.y += this.m_xf.position.y;
		this.m_sweep.c0.SetV(this.m_sweep.c);
		this.m_sweep.a0 = this.m_sweep.a = angle;

		while (f) {
			f.Synchronize(broadPhase, this.m_xf, this.m_xf);
			f = f.m_next;
		}

		this.m_world.m_contactManager.FindNewContacts();
	},

	SetTransform : function (xf) {
		this.SetPositionAndAngle(xf.position, xf.GetAngle());
	},

	GetTransform : function () {
		return this.m_xf;
	},

	GetPosition : function () {
		return this.m_xf.position;
	},

	SetPosition : function (position) {
		this.SetPositionAndAngle(position, this.GetAngle());
	},

	GetAngle : function () {
		return this.m_sweep.a;
	},

	SetAngle : function (angle) {
		this.SetPositionAndAngle(this.GetPosition(), angle || 0);
	},

	GetWorldCenter : function () {
		return this.m_sweep.c;
	},

	GetLocalCenter : function () {
		return this.m_sweep.localCenter;
	},

	SetLinearVelocity : function (v) {
		if (this.m_type === b2Body.b2_staticBody) {
			return;
		}
		this.m_linearVelocity.SetV(v);
	},

	GetLinearVelocity : function () {
		return this.m_linearVelocity;
	},

	SetAngularVelocity : function (omega) {
		if (this.m_type === b2Body.b2_staticBody) {
			return;
		}
		this.m_angularVelocity = omega || 0;
	},

	GetAngularVelocity : function () {
		return this.m_angularVelocity;
	},

	GetDefinition : function () {
		var bd = new b2BodyDef();
		bd.type = this.GetType();
		bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
		bd.angle = this.GetAngle();
		bd.angularDamping = this.m_angularDamping;
		bd.angularVelocity = this.m_angularVelocity;
		bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
		bd.bullet = (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
		bd.awake = (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
		bd.linearDamping = this.m_linearDamping;
		bd.linearVelocity.SetV(this.GetLinearVelocity());
		bd.position = this.GetPosition();
		bd.userData = this.GetUserData();
		return bd;
	},

	ApplyForce : function (force, point) {
		if (this.m_type !== b2Body.b2_dynamicBody) {
			return;
		}
		this.SetAwake(true);
		this.m_force.x += force.x;
		this.m_force.y += force.y;
		this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
	},

	ApplyTorque : function (torque) {
		if (this.m_type !== b2Body.b2_dynamicBody) {
			return;
		}
		this.SetAwake(true);
		this.m_torque += torque || 0;
	},

	ApplyImpulse : function (impulse, point) {
		if (this.m_type != b2Body.b2_dynamicBody) {
			return;
		}
		this.SetAwake(true);
		this.m_linearVelocity.x += this.m_invMass * impulse.x;
		this.m_linearVelocity.y += this.m_invMass * impulse.y;
		this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
	},

	Split : function (callback) {
		var linearVelocity = this.GetLinearVelocity().Copy(),
			angularVelocity = this.GetAngularVelocity(),
			center = this.GetWorldCenter(),
			body1 = this,
			body2 = this.m_world.CreateBody(this.GetDefinition()),
			prev,
			f = body1.m_fixtureList,
			next,
			center1, center2,
			velocity1, velocity2;

		while (f) {
			if (callback(f)) {
				next = f.m_next;
				if (prev) {
					prev.m_next = next;
				} else {
					body1.m_fixtureList = next;
				}
				body1.m_fixtureCount--;
				f.m_next = body2.m_fixtureList;
				body2.m_fixtureList = f;
				body2.m_fixtureCount++;
				f.m_body = body2;
				f = next;
			} else {
				prev = f;
				f = f.m_next;
			}
		}

		body1.ResetMassData();
		body2.ResetMassData();

		center1 = body1.GetWorldCenter();
		center2 = body2.GetWorldCenter();

		velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center))); // TODO b2Vec2 reuse?
		velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center))); // TODO b2Vec2 reuse?

		body1.SetLinearVelocity(velocity1);
		body2.SetLinearVelocity(velocity2);

		body1.SetAngularVelocity(angularVelocity);
		body2.SetAngularVelocity(angularVelocity);

		body1.SynchronizeFixtures();
		body2.SynchronizeFixtures();

		return body2;
	},

	Merge : function (other) {
		var f = other.m_fixtureList,
			next;

		while (f) {
			next = f.m_next;
			other.m_fixtureCount--;
			f.m_next = this.m_fixtureList;
			this.m_fixtureList = f;
			this.m_fixtureCount++;
			f.m_body = body2;
			f = next;
		}

		this.ResetMassData();
		this.SynchronizeFixtures();
	},

	GetMass : function () {
		return this.m_mass;
	},

	GetInertia : function () {
		return this.m_I;
	},

	GetMassData : function (data) {
		data.mass = this.m_mass;
		data.I = this.m_I;
		data.center.SetV(this.m_sweep.localCenter);
	},

	SetMassData : function (massData) {
		if (!this.m_world.IsLocked()) {
			b2Settings.b2Assert(false);
			return;
		}
		if (this.m_type !== b2Body.b2_dynamicBody) {
			return;
		}

		var oldCenter;

		this.m_invMass = 0;
		this.m_I = 0;
		this.m_invI = 0;
		this.m_mass = massData.mass;

		if (this.m_mass <= 0) {
			this.m_mass = 1;
		}

		this.m_invMass = 1 / this.m_mass;

		if (massData.I > 0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
			this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
			this.m_invI = 1 / this.m_I;
		}

		oldCenter = this.m_sweep.c.Copy();

		this.m_sweep.localCenter.SetV(massData.center);
		this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.SetV(this.m_sweep.c0);
		this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
		this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
	},

	ResetMassData : function () {
		var center,
			oldCenter,
			massData,
			f = this.m_fixtureList;

		this.m_mass = 0;
		this.m_invMass = 0;
		this.m_I = 0;
		this.m_invI = 0;
		this.m_sweep.localCenter.SetZero();

		if (this.m_type === b2Body.b2_staticBody || this.m_type === b2Body.b2_kinematicBody) {
			return;
		}

		center = new b2Vec2();

		while (f) {
			if (!f.m_density) {
				continue;
			}
			massData = f.GetMassData();
			this.m_mass += massData.mass;
			center.x += massData.center.x * massData.mass;
			center.y += massData.center.y * massData.mass;
			this.m_I += massData.I;
			f = f.m_next;
		}

		if (this.m_mass > 0) {
			this.m_invMass = 1 / this.m_mass;
			center.x *= this.m_invMass;
			center.y *= this.m_invMass;
		} else {
			this.m_mass = 1;
			this.m_invMass = 1;
		}

		if (this.m_I > 0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
			this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
			this.m_I *= this.m_inertiaScale;
			b2Settings.b2Assert(this.m_I > 0);
			this.m_invI = 1 / this.m_I;
		} else {
			this.m_I = 0;
			this.m_invI = 0;
		}

		oldCenter = this.m_sweep.c.Copy();

		this.m_sweep.localCenter.SetV(center);
		this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
		this.m_sweep.c.SetV(this.m_sweep.c0);
		this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
		this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
	},

	GetWorldPoint : function (localPoint) {
		var A = this.m_xf.R,
			u = new b2Vec2(
				A.col1.x * localPoint.x + A.col2.x * localPoint.y,
				A.col1.y * localPoint.x + A.col2.y * localPoint.y
			);
		u.x += this.m_xf.position.x;
		u.y += this.m_xf.position.y;
		return u;
	},

	GetWorldVector : function (localVector) {
		return b2Math.MulMV(this.m_xf.R, localVector);
	},

	GetLocalPoint : function (worldPoint) {
		return b2Math.MulXT(this.m_xf, worldPoint);
	},

	GetLocalVector : function (worldVector) {
		return b2Math.MulTMV(this.m_xf.R, worldVector);
	},

	GetLinearVelocityFromWorldPoint : function (worldPoint) {
		return new b2Vec2(
			this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
			this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x)
		);
	},

	GetLinearVelocityFromLocalPoint : function (localPoint) {
		var A = this.m_xf.R,
			worldPoint = new b2Vec2(
				A.col1.x * localPoint.x + A.col2.x * localPoint.y,
				A.col1.y * localPoint.x + A.col2.y * localPoint.y
			);
		worldPoint.x += this.m_xf.position.x;
		worldPoint.y += this.m_xf.position.y;
		return new b2Vec2(
			this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y),
			this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x)
		);
	},

	GetLinearDamping : function () {
		return this.m_linearDamping;
	},

	SetLinearDamping : function (linearDamping) {
		this.m_linearDamping = linearDamping || 0;
	},

	GetAngularDamping : function () {
		return this.m_angularDamping;
	},

	SetAngularDamping : function (angularDamping) {
		this.m_angularDamping = angularDamping || 0;
	},

	SetType : function (type) {
		var ce = this.m_contactList;

		type = type || 0;
		if (this.m_type === type) {
			return;
		}

		this.m_type = type;
		this.ResetMassData();

		if (this.m_type === b2Body.b2_staticBody) {
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0;
		}

		this.SetAwake(true);
		this.m_force.SetZero();
		this.m_torque = 0;

		while (ce) {
			ce.contact.FlagForFiltering();
			ce = ce.next;
		}
	},

	GetType : function () {
		return this.m_type;
	},

	SetBullet : function (flag) {
		if (flag) {
			this.m_flags |= b2Body.e_bulletFlag;
		} else {
			this.m_flags &= ~b2Body.e_bulletFlag;
		}
	},

	IsBullet : function () {
		return (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
	},

	SetSleepingAllowed : function (flag) {
		if (flag) {
			this.m_flags |= b2Body.e_allowSleepFlag;
		} else {
			this.m_flags &= ~b2Body.e_allowSleepFlag;
			this.SetAwake(true);
		}
	},

	SetAwake : function (flag) {
		if (this.IsAwake() === flag) {
			return;
		}
		if (flag) {
			this.m_flags |= b2Body.e_awakeFlag;
			this.m_sleepTime = 0;
		} else {
			this.m_flags &= ~b2Body.e_awakeFlag;
			this.m_sleepTime = 0;
			this.m_linearVelocity.SetZero();
			this.m_angularVelocity = 0;
			this.m_force.SetZero();
			this.m_torque = 0;
		}
	},

	IsAwake : function () {
		return (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
	},

	SetFixedRotation : function (fixed) {
		if (fixed) {
			this.m_flags |= b2Body.e_fixedRotationFlag;
		} else {
			this.m_flags &= ~b2Body.e_fixedRotationFlag;
		}
		this.ResetMassData();
	},

	IsFixedRotation : function () {
		return (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
	},

	SetActive : function (flag) {
		if (flag === this.IsActive()) {
			return;
		}
		var broadPhase,
			f = this.m_fixtureList,
			ce = this.m_contactList,
			ce0;

		if (flag) {
			this.m_flags |= b2Body.e_activeFlag;
			broadPhase = this.m_world.m_contactManager.m_broadPhase;

			while (f) {
				f.CreateProxy(broadPhase, this.m_xf);
				f = f.m_next;
			}
		} else {
			this.m_flags &= ~b2Body.e_activeFlag;
			broadPhase = this.m_world.m_contactManager.m_broadPhase;

			while (f) {
				f.DestroyProxy(broadPhase);
				f = f.m_next;
			}

			while (ce) {
				ce0 = ce;
				ce = ce.next;
				this.m_world.m_contactManager.Destroy(ce0.contact);
			}

			this.m_contactList = null;
		}
	},

	IsActive : function () {
		return (this.m_flags & b2Body.e_activeFlag) === b2Body.e_activeFlag;
	},

	IsSleepingAllowed : function () {
		return (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
	},

	GetFixtureList : function () {
		return this.m_fixtureList;
	},

	GetJointList : function () {
		return this.m_jointList;
	},

	GetControllerList : function () {
		return this.m_controllerList;
	},

	GetContactList : function () {
		return this.m_contactList;
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

	GetWorld : function () {
		return this.m_world;
	},

	SynchronizeFixtures : function () {
		var xf1 = b2Body.s_xf1,
			tMat, tVec,
			f = this.m_fixtureList,
			broadPhase = this.m_world.m_contactManager.m_broadPhase;

		xf1.R.Set(this.m_sweep.a0);

		tMat = xf1.R;
		tVec = this.m_sweep.localCenter;
		xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		while (f) {
			f.Synchronize(broadPhase, xf1, this.m_xf);
			f = f.m_next;
		}
	},

	SynchronizeTransform : function () {
		var tMat, tVec;

		this.m_xf.R.Set(this.m_sweep.a);

		tMat = this.m_xf.R;
		tVec = this.m_sweep.localCenter;
		this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
	},

	ShouldCollide : function (other) {
		var jn = this.m_jointList;

		if (this.m_type !== b2Body.b2_dynamicBody && other.m_type !== b2Body.b2_dynamicBody) {
			return false;
		}

		while (jn) {
			if (jn.other === other) {
				if (!jn.joint.m_collideConnected) {
					return false;
				}
			}
			jn = jn.next;
		}
		return true;
	},

	Advance : function (t) {
		this.m_sweep.Advance(t || 0);
		this.m_sweep.c.SetV(this.m_sweep.c0);
		this.m_sweep.a = this.m_sweep.a0;
		this.SynchronizeTransform();
	}
};