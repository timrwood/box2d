function b2Contact() {
	this.m_nodeA = new b2ContactEdge();
	this.m_nodeB = new b2ContactEdge();
	this.m_manifold = new b2Manifold();
	this.m_oldManifold = new b2Manifold();
}

b2Contact.e_toiFlag        =  1;
b2Contact.e_filterFlag     =  2;
b2Contact.e_sensorFlag     =  4;
b2Contact.e_islandFlag     =  8;
b2Contact.e_enabledFlag    = 16;
b2Contact.e_touchingFlag   = 32;
b2Contact.e_continuousFlag = 64;


b2Contact.prototype = {
	GetManifold : function () {
		return this.m_manifold;
	},

	GetWorldManifold : function (worldManifold) {
		var bodyA = this.m_fixtureA.GetBody(),
			bodyB = this.m_fixtureB.GetBody(),
			shapeA = this.m_fixtureA.GetShape(),
			shapeB = this.m_fixtureB.GetShape();
		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	},

	IsTouching : function () {
		return (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag;
	},

	IsContinuous : function () {
		return (this.m_flags & b2Contact.e_continuousFlag) === b2Contact.e_continuousFlag;
	},

	SetSensor : function (sensor) {
		if (sensor) {
			this.m_flags |= b2Contact.e_sensorFlag;
		} else {
			this.m_flags &= ~b2Contact.e_sensorFlag;
		}
	},

	IsSensor : function () {
		return (this.m_flags & b2Contact.e_sensorFlag) == b2Contact.e_sensorFlag;
	},

	SetEnabled : function (flag) {
		if (flag) {
			this.m_flags |= b2Contact.e_enabledFlag;
		} else {
			this.m_flags &= ~b2Contact.e_enabledFlag;
		}
	},

	IsEnabled : function () {
		return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
	},

	GetNext : function () {
		return this.m_next;
	},

	GetFixtureA : function () {
		return this.m_fixtureA;
	},

	GetFixtureB : function () {
		return this.m_fixtureB;
	},

	FlagForFiltering : function () {
		this.m_flags |= b2Contact.e_filterFlag;
	},

	Reset : function (fixtureA, fixtureB) {
		var bodyA, bodyB;

		this.m_flags = b2Contact.e_enabledFlag;

		if (!fixtureA || !fixtureB) {
			this.m_fixtureA = null;
			this.m_fixtureB = null;
			return;
		}

		if (fixtureA.IsSensor() || fixtureB.IsSensor()) {
			this.m_flags |= b2Contact.e_sensorFlag;
		}

		bodyA = fixtureA.GetBody();
		bodyB = fixtureB.GetBody();

		if (bodyA.GetType() !== b2Body.b2_dynamicBody ||
			bodyB.GetType() !== b2Body.b2_dynamicBody ||
			bodyA.IsBullet() ||
			bodyB.IsBullet()) {
			this.m_flags |= b2Contact.e_continuousFlag;
		}

		this.m_fixtureA = fixtureA;
		this.m_fixtureB = fixtureB;
		this.m_manifold.m_pointCount = 0;

		this.m_prev = null;
		this.m_next = null;

		this.m_nodeA.contact = null;
		this.m_nodeA.prev = null;
		this.m_nodeA.next = null;
		this.m_nodeA.other = null;

		this.m_nodeB.contact = null;
		this.m_nodeB.prev = null;
		this.m_nodeB.next = null;
		this.m_nodeB.other = null;
	},

	Update : function (listener) {
		var tManifold = this.m_oldManifold,
			touching = false,
			wasTouching = (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag,
			bodyA = this.m_fixtureA.m_body,
			bodyB = this.m_fixtureB.m_body,
			aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb),
			shapeA, shapeB,
			xfA, xfB,
			i, j,
			mp2, mp1,
			id2;

		this.m_oldManifold = this.m_manifold;
		this.m_manifold = tManifold;

		this.m_flags |= b2Contact.e_enabledFlag;

		if (this.m_flags & b2Contact.e_sensorFlag) {
			if (aabbOverlap) {
				shapeA = this.m_fixtureA.GetShape();
				shapeB = this.m_fixtureB.GetShape();
				xfA = bodyA.GetTransform();
				xfB = bodyB.GetTransform();
				touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
			}

			this.m_manifold.m_pointCount = 0;
		} else {
			if (bodyA.GetType() !== b2Body.b2_dynamicBody ||
				bodyB.GetType() !== b2Body.b2_dynamicBody ||
				bodyA.IsBullet() ||
				bodyB.IsBullet()) {
				this.m_flags |= b2Contact.e_continuousFlag;
			} else {
				this.m_flags &= ~b2Contact.e_continuousFlag;
			}

			if (aabbOverlap) {
				this.Evaluate();
				touching = this.m_manifold.m_pointCount > 0;

				for (i = 0; i < this.m_manifold.m_pointCount; i++) {
					mp2 = this.m_manifold.m_points[i];
					mp2.m_normalImpulse = 0;
					mp2.m_tangentImpulse = 0;

					id2 = mp2.m_id;

					for (j = 0; j < this.m_oldManifold.m_pointCount; j++) {
						mp1 = this.m_oldManifold.m_points[j];

						if (mp1.m_id.key == id2.key) {
							mp2.m_normalImpulse = mp1.m_normalImpulse;
							mp2.m_tangentImpulse = mp1.m_tangentImpulse;
							break;
						}
					}
				}
			} else {
				this.m_manifold.m_pointCount = 0;
			}

			if (touching !== wasTouching) {
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		if (touching) {
			this.m_flags |= b2Contact.e_touchingFlag;
		} else {
			this.m_flags &= ~b2Contact.e_touchingFlag;
		}

		if (!wasTouching && touching) {
			listener.BeginContact(this);
		}

		if (wasTouching && !touching) {
			listener.EndContact(this);
		}

		if ((this.m_flags & b2Contact.e_sensorFlag) === 0) {
			listener.PreSolve(this, this.m_oldManifold);
		}
	},

	Evaluate : function () {},

	ComputeTOI : function (sweepA, sweepB) {
		b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
		b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());

		b2Contact.s_input.sweepA = sweepA;
		b2Contact.s_input.sweepB = sweepB;

		b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;

		return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
	}
};