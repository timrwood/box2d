function b2CircleContact() {
	b2Contact.apply(this, arguments);
}

b2CircleContact.Create = function (allocator) {
	return new b2CircleContact();
};

b2CircleContact.Destroy = function (contact, allocator) {};

b2CircleContact.prototype = extend(new b2Contact(), {
	Evaluate : function () {
		b2Collision.CollideCircles(
			this.m_manifold,
			this.m_fixtureA.GetShape(),
			this.m_fixtureA.GetBody().m_xf,
			this.m_fixtureB.GetShape(),
			this.m_fixtureB.GetBody().m_xf
		);
	}
});