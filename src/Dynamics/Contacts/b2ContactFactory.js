function b2ContactFactory() {
	var e_circ = b2Shape.e_circleShape,
		e_poly = b2Shape.e_polygonShape,
		e_edge = b2Shape.e_edgeShape,
		m_registers = this.m_registers = [],
		m_contacts = this.m_contacts = [];

	m_registers[e_circ] = [];
	m_registers[e_poly] = [];
	m_registers[e_edge] = [];

	m_registers[e_circ][e_circ] = b2CircleContact;
	m_registers[e_poly][e_circ] = b2PolyAndCircleContact;
	m_registers[e_poly][e_poly] = b2PolygonContact;
	m_registers[e_edge][e_circ] = b2EdgeAndCircleContact;
	m_registers[e_poly][e_edge] = b2PolyAndEdgeContact;

	m_contacts[e_circ] = [];
	m_contacts[e_poly] = [];
	m_contacts[e_edge] = [];

	m_contacts[e_circ][e_circ] = [];
	m_contacts[e_poly][e_circ] = [];
	m_contacts[e_poly][e_poly] = [];
	m_contacts[e_edge][e_circ] = [];
	m_contacts[e_poly][e_edge] = [];
}

Box2D.b2ContactFactory = b2ContactFactory;

b2ContactFactory.prototype = {
	Create : function (fixtureA, fixtureB) {
		var type1 = fixtureA.GetType(),
			type2 = fixtureB.GetType(),
			a = fixtureA,
			b = fixtureB,
			Contact = this.m_registers[type1][type2],
			array = this.m_contacts[type1][type2],
			c;

		if (!Contact) {
			Contact = this.m_registers[type2][type1];
			array = this.m_contacts[type2][type1];
			b = fixtureA;
			a = fixtureB;
		}

		if (array && array.length) {
			c = array.pop();
		} else if (Contact) {
			c = new Contact();
		}

		if (c) {
			c.Reset(a, b);
			return c;
		}
	},

	Destroy : function (contact) {
		var type1 = contact.m_fixtureA.GetType(),
			type2 = contact.m_fixtureB.GetType(),
			array = this.m_contacts[type1][type2] || this.m_contacts[type2][type1];

		array.push(contact);

		if (contact.m_manifold.m_pointCount > 0) {
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
	}
};
