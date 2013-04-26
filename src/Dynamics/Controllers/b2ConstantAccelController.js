function b2ConstantAccelController() {
	b2Controller.apply(this, arguments);
	this.A = new b2Vec2();
}

Box2D.b2ConstantAccelController = b2ConstantAccelController;

inherit(b2Controller, b2ConstantAccelController);

b2ConstantAccelController.prototype = {
	Step : function (step) {
		var vel,
			smallAX = this.A.x * step.dt,
			smallAY = this.A.y * step.dt,
			i, body;

		for (i = this.m_bodyList; i; i = i.nextBody) {
			body = i.body;

			if (!body.IsAwake()) {
				continue;
			}

			vel = body.GetLinearVelocity();

			vel.x += smallAX;
			vel.y += smallAY;

			body.SetLinearVelocity(vel);
		}
	}
};
