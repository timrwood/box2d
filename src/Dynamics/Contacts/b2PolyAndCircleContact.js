function b2PolyAndCircleContact() {
	b2Contact.apply(this, arguments);
}

b2PolyAndCircleContact.prototype = extend(new b2Contact(), {
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
});
