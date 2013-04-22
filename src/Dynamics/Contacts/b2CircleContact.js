function b2CircleContact() {
	b2Contact.apply(this, arguments);
}

b2CircleContact.prototype = extend(new b2Contact(), {
	Evaluate : function () {
		var fA = this.m_fixtureA,
			fB = this.m_fixtureB;

		b2Collision.CollideCircles(
			this.m_manifold,
			fA.GetShape(),
			fA.GetBody().m_xf,
			fB.GetShape(),
			fB.GetBody().m_xf
		);
	}
});
