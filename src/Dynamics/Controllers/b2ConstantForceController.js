function b2ConstantForceController() {
	b2Controller.apply(this, arguments);
	this.F = new b2Vec2();
}

Box2D.b2ConstantForceController = b2ConstantForceController;

inherit(b2Controller, b2ConstantForceController);

b2ConstantForceController.prototype = {
	Step : function (step) {
		var i, body;

		for (i = this.m_bodyList; i; i = i.nextBody) {
			body = i.body;

			if (!body.IsAwake()) {
				continue;
			}

			body.ApplyForce(this.F, body.GetWorldCenter());
		}
	}
};
