function b2BuoyancyController() {
	b2Controller.apply(this, arguments);
	this.normal = new b2Vec2(0, -1);
	this.offset = 0;
	this.density = 0;
	this.velocity = new b2Vec2(0, 0);
	this.linearDrag = 2;
	this.angularDrag = 1;
	this.useDensity = false;
	this.useWorldGravity = true;
	this.gravity = null;
}

Box2D.b2BuoyancyController = b2BuoyancyController;

inherit(b2Controller, b2BuoyancyController);

b2BuoyancyController.prototype = {
	Step : function (step) {
		var i,
			body,
			areac, area,
			massc, mass,
			fixture,
			sc,
			sarea,
			shapeDensity,
			buoyancyForce,
			dragForce;

		if (!this.m_bodyList) {
			return;
		}

		if (this.useWorldGravity) {
			this.gravity = this.GetWorld().GetGravity().Copy();
		}

		for (i = this.m_bodyList; i; i = i.nextBody) {
			body = i.body;
			if (!body.IsAwake()) {
				continue;
			}

			areac = new b2Vec2(); // TODO: Reuse b2Vec2 ?
			massc = new b2Vec2(); // TODO: Reuse b2Vec2 ?

			area = 0;
			mass = 0;

			for (fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
				sc = new b2Vec2(); // TODO: Reuse b2Vec2 ?
				sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);

				area += sarea;

				areac.x += sarea * sc.x;
				areac.y += sarea * sc.y;

				shapeDensity = 0;

				if (this.useDensity) {
					shapeDensity = 1;
				} else {
					shapeDensity = 1;
				}

				mass += sarea * shapeDensity;

				massc.x += sarea * sc.x * shapeDensity;
				massc.y += sarea * sc.y * shapeDensity;
			}
			areac.x /= area;
			areac.y /= area;

			massc.x /= mass;
			massc.y /= mass;

			if (area < Number.MIN_VALUE) {
				continue;
			}

			buoyancyForce = this.gravity.GetNegative();
			buoyancyForce.Multiply(this.density * area);
			body.ApplyForce(buoyancyForce, massc);

			dragForce = body.GetLinearVelocityFromWorldPoint(areac);
			dragForce.Subtract(this.velocity);
			dragForce.Multiply((-this.linearDrag * area));

			body.ApplyForce(dragForce, areac);
			body.ApplyTorque(-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag);
		}
	},

	Draw : function (debugDraw) {
		var r = 1000,
			p1 = new b2Vec2(),
			p2 = new b2Vec2();

		p1.x = this.normal.x * this.offset + this.normal.y * r;
		p1.y = this.normal.y * this.offset - this.normal.x * r;
		p2.x = this.normal.x * this.offset - this.normal.y * r;
		p2.y = this.normal.y * this.offset + this.normal.x * r;

		debugDraw.DrawSegment(p1, p2, new b2Color(0, 0, 1));
	}
};
