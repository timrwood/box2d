function b2TensorDampingController() {
	b2Controller.apply(this, arguments);
	this.T = new b2Mat22();
	this.maxTimestep = 0;
}

Box2D.b2TensorDampingController = b2TensorDampingController;

inherit(b2Controller, b2TensorDampingController);

b2TensorDampingController.prototype = {
	SetAxisAligned : function (xDamping, yDamping) {
		xDamping = xDamping || 0;
		yDamping = yDamping || 0;
		this.T.col1.x = -xDamping;
		this.T.col1.y = 0;
		this.T.col2.x = 0;
		this.T.col2.y = -yDamping;
		if (xDamping > 0 || yDamping > 0) {
			this.maxTimestep = 1 / Math.max(xDamping, yDamping);
		} else {
			this.maxTimestep = 0;
		}
	},

	Step : function (step) {
		var timestep = step.dt,
			i,
			body,
			vel,
			damping;

		if (timestep <= Number.MIN_VALUE) {
			return;
		}

		if (timestep > this.maxTimestep && this.maxTimestep > 0) {
			timestep = this.maxTimestep;
		}

		for (i = this.m_bodyList; i; i = i.nextBody) {
			body = i.body;

			if (!body.IsAwake()) {
				continue;
			}

			damping = body.GetWorldVector(b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity()))); // TODO: b2Vec2 reuse?

			vel = body.GetLinearVelocity();
			vel.x += damping.x * timestep;
			vel.y += damping.y * timestep;

			body.SetLinearVelocity(vel);
		}
	}
};
