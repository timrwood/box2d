function b2GravityController() {
	b2Controller.apply(this, arguments);
	this.G = 1;
	this.invSqr = true;
}

b2GravityController.prototype = extend(new b2Controller(), {
	Step : function (step) {
		var i, j,
			body1, body2,
			p1, p2,
			mass1,
			dx, dy,
			r2,
			f;

		for (i = this.m_bodyList; i; i = i.nextBody) {
			body1 = i.body;

			p1 = body1.GetWorldCenter();
			mass1 = body1.GetMass();

			for (j = this.m_bodyList; j !== i; j = j.nextBody) {
				body2 = j.body;
				p2 = body2.GetWorldCenter();

				dx = p2.x - p1.x;
				dy = p2.y - p1.y;

				r2 = dx * dx + dy * dy;

				if (r2 < Number.MIN_VALUE) {
					continue;
				}

				f = new b2Vec2(dx, dy); // TODO: Reuse b2Vec2?
				if (this.invSqr) {
					f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
				} else {
					f.Multiply(this.G / r2 * mass1 * body2.GetMass());
				}

				if (body1.IsAwake()) {
					body1.ApplyForce(f, p1);
				}

				f.Multiply(-1);

				if (body2.IsAwake()) {
					body2.ApplyForce(f, p2);
				}
			}
		}
	}
});
