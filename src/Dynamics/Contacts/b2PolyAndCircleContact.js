function b2PolyAndCircleContact() {
	b2Contact.apply(this, arguments);
}

Box2D.b2PolyAndCircleContact = b2PolyAndCircleContact;

inherit(b2Contact, b2PolyAndCircleContact);

b2PolyAndCircleContact.prototype = {
	Evaluate : function () {
		var fA = this.m_fixtureA,
			fB = this.m_fixtureB;

		b2Collision.CollidePolygonAndCircle(
			this.m_manifold,
			fA.GetShape(),
			fA.GetBody().m_xf,
			fB.GetShape(),
			fB.GetBody().m_xf
		);
	}
};
