function b2ContactFactory() {
	var e_circ = b2Shape.e_circleShape,
		e_poly = b2Shape.e_polygonShape,
		e_edge = b2Shape.e_edgeShape,
		m_registers = this.m_registers = [];

	m_registers[e_circ] = [];
	m_registers[e_poly] = [];
	m_registers[e_edge] = [];

	m_registers[e_circ][e_circ] = b2CircleContact;
	m_registers[e_poly][e_circ] = b2PolyAndCircleContact;
	m_registers[e_poly][e_poly] = b2PolygonContact;
	m_registers[e_edge][e_circ] = b2EdgeAndCircleContact;
	m_registers[e_poly][e_edge] = b2PolyAndEdgeContact;
}

b2ContactFactory.prototype = {
	Create : function (fixtureA, fixtureB) {
		var type1 = fixtureA.GetType(),
			type2 = fixtureB.GetType(),
			Contact = this.m_registers[type1][type2] || this.m_registers[type2][type1],
			c;

		if (Contact) {
			c = new Contact();
			c.Reset(fixtureB, fixtureA);
			return c;
		}
	},

	Destroy : function (contact) {
		if (contact.m_manifold.m_pointCount > 0) {
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
	}
};