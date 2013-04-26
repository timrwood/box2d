function b2EdgeAndCircleContact() {
	b2Contact.apply(this, arguments);
}

Box2D.b2EdgeAndCircleContact = b2EdgeAndCircleContact;

inherit(b2Contact, b2EdgeAndCircleContact);

b2EdgeAndCircleContact.prototype = {
	Evaluate : function () {
		var fA = this.m_fixtureA,
			fB = this.m_fixtureB;

		this.b2CollideEdgeAndCircle(
			this.m_manifold,
			fA.GetShape(),
			fA.GetBody().m_xf,
			fB.GetShape(),
			fB.GetBody().m_xf
		);
	},

	b2CollideEdgeAndCircle : function (manifold, edge, xf1, circle, xf2) {}
};
