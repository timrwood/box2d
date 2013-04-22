function b2PolyAndEdgeContact() {
	b2Contact.apply(this, arguments);
}

b2PolyAndEdgeContact.prototype = extend(new b2Contact(), {
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
});
