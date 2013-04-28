function b2Fixture() {
	this.m_filter = new b2FilterData();
	this.m_aabb = new b2AABB();
	this.m_userData = null;
	this.m_body = null;
	this.m_next = null;
	this.m_shape = null;
	this.m_density = 0;
	this.m_friction = 0;
	this.m_restitution = 0;
}

Box2D.b2Fixture = b2Fixture;

b2Fixture.prototype = {
	GetType : function () {
		return this.m_shape.GetType();
	},

	GetShape : function () {
		return this.m_shape;
	},

	SetSensor : function (sensor) {
		var edge, contact,
			fixtureA, fixtureB;

		if (this.m_isSensor === sensor) {
			return;
		}

		this.m_isSensor = sensor;

		if (!this.m_body) {
			return;
		}

		edge = this.m_body.GetContactList();

		while (edge) {
			contact = edge.contact;
			fixtureA = contact.GetFixtureA();
			fixtureB = contact.GetFixtureB();
			if (fixtureA === this || fixtureB === this) {
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			}
			edge = edge.next;
		}
	},

	IsSensor : function () {
		return this.m_isSensor;
	},

	SetFilterData : function (filter) {
		var edge, contact,
			fixtureA, fixtureB;

		this.m_filter = filter.Copy();

		if (!this.m_body) {
			return;
		}

		edge = this.m_body.GetContactList();

		while (edge) {
			contact = edge.contact;
			fixtureA = contact.GetFixtureA();
			fixtureB = contact.GetFixtureB();
			if (fixtureA === this || fixtureB === this) {
				contact.FlagForFiltering();
			}
			edge = edge.next;
		}
	},

	GetFilterData : function () {
		return this.m_filter.Copy();
	},

	GetBody : function () {
		return this.m_body;
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

	TestPoint : function (p) {
		return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
	},

	RayCast : function (output, input) {
		return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
	},

	GetMassData : function (massData) {
		massData = massData || new b2MassData();
		this.m_shape.ComputeMass(massData, this.m_density);
		return massData;
	},

	SetDensity : function (density) {
		this.m_density = density || 0;
	},

	GetDensity : function () {
		return this.m_density;
	},

	GetFriction : function () {
		return this.m_friction;
	},

	SetFriction : function (friction) {
		this.m_friction = friction || 0;
	},

	GetRestitution : function () {
		return this.m_restitution;
	},

	SetRestitution : function (restitution) {
		this.m_restitution = restitution || 0;
	},

	GetAABB : function () {
		return this.m_aabb;
	},

	Create : function (body, xf, def) {
		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;
		this.m_body = body;
		this.m_next = null;
		this.m_filter = def.filter.Copy();
		this.m_isSensor = def.isSensor;
		this.m_shape = def.shape.Copy();
		this.m_density = def.density;
	},

	Destroy : function () {
		this.m_shape = null;
	},

	CreateProxy : function (broadPhase, xf) {
		this.m_shape.ComputeAABB(this.m_aabb, xf);
		this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
	},

	DestroyProxy : function (broadPhase) {
		if (!this.m_proxy) {
			return;
		}
		broadPhase.DestroyProxy(this.m_proxy);
		this.m_proxy = null;
	},

	Synchronize : function (broadPhase, transform1, transform2) {
		var aabb1, aabb2,
			displacement = b2Fixture.t_vec2a;

		if (!this.m_proxy) {
			return;
		}

		aabb1 = b2Fixture.t_aabb1;
		aabb2 = b2Fixture.t_aabb2;

		this.m_shape.ComputeAABB(aabb1, transform1);
		this.m_shape.ComputeAABB(aabb2, transform2);
		this.m_aabb.Combine(aabb1, aabb2);

		displacement = b2Math.SubtractVV(transform2.position, transform1.position, displacement);

		broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
	}
};

whenReady(function () {
	b2Fixture.t_vec2a = new b2Vec2();
	b2Fixture.t_aabb1 = new b2AABB();
	b2Fixture.t_aabb2 = new b2AABB();
});
