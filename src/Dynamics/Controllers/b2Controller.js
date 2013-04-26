function b2Controller() {

}

Box2D.b2Controller = b2Controller;

b2Controller.prototype = {
	Step : function (step) {},

	Draw : function (debugDraw) {},

	AddBody : function (body) {
		var edge = new b2ControllerEdge();

		edge.controller = this;
		edge.body = body;

		edge.nextBody = this.m_bodyList;
		edge.prevBody = null;

		this.m_bodyList = edge;

		if (edge.nextBody) {
			edge.nextBody.prevBody = edge;
		}

		this.m_bodyCount++;

		edge.nextController = body.m_controllerList;
		edge.prevController = null;

		body.m_controllerList = edge;

		if (edge.nextController) {
			edge.nextController.prevController = edge;
		}

		body.m_controllerCount++;
	},

	RemoveBody : function (body) {
		var edge = body.m_controllerList;

		while (edge && edge.controller !== this) {
			edge = edge.nextController;
		}

		if (edge.prevBody) {
			edge.prevBody.nextBody = edge.nextBody;
		}

		if (edge.nextBody) {
			edge.nextBody.prevBody = edge.prevBody;
		}

		if (edge.nextController) {
			edge.nextController.prevController = edge.prevController;
		}

		if (edge.prevController) {
			edge.prevController.nextController = edge.nextController;
		}

		if (this.m_bodyList === edge) {
			this.m_bodyList = edge.nextBody;
		}

		if (body.m_controllerList === edge) {
			body.m_controllerList = edge.nextController;
		}

		body.m_controllerCount--;
		this.m_bodyCount--;
	},

	Clear : function () {
		while (this.m_bodyList) {
			this.RemoveBody(this.m_bodyList.body);
		}
	},

	GetNext : function () {
		return this.m_next;
	},

	GetWorld : function () {
		return this.m_world;
	},

	GetBodyList : function () {
		return this.m_bodyList;
	}
};
