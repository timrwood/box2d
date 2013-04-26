function b2PolyAndEdgeContact() {
	b2Contact.apply(this, arguments);
}

Box2D.b2PolyAndEdgeContact = b2PolyAndEdgeContact;

inherit(b2Contact, b2PolyAndEdgeContact);

b2PolyAndEdgeContact.prototype = {
	Evaluate : function () {
		var fA = this.m_fixtureA,
			fB = this.m_fixtureB;

		this.b2CollidePolyAndEdge(
			this.m_manifold,
			fA.GetShape(),
			fA.GetBody().m_xf,
			fB.GetShape(),
			fB.GetBody().m_xf
		);
	},

	b2CollidePolyAndEdge : function (manifold, polygon, xf1, edge, xf2) {}
};
