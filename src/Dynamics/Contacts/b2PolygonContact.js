function b2PolygonContact() {
	b2Contact.apply(this, arguments);
}

Box2D.b2PolygonContact = b2PolygonContact;

inherit(b2Contact, b2PolygonContact);

b2PolygonContact.prototype = {
	Evaluate : function () {
		var fA = this.m_fixtureA,
			fB = this.m_fixtureB;

		b2Collision.CollidePolygons(
			this.m_manifold,
			fA.GetShape(),
			fA.GetBody().m_xf,
			fB.GetShape(),
			fB.GetBody().m_xf
		);
	}
};
