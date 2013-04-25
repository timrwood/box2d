var Box2D = {},
	Collision = Box2D.Collision = {},
	Shapes = Collision.Shapes = {},
	Common = Box2D.Common = {
		Math : {}
	},
	Dynamics = Box2D.Dynamics = {},
	Contacts = Dynamics.Contacts = {},
	Controllers = Dynamics.Controllers = {},
	Joints = Dynamics.Joints = {},

	callbacks = [];

function generateCallback(context, cb) {
	return function () {
		cb.apply(context, arguments);
	};
}

// TODO: Remove in favor of proper getter/setter methods
function defineProperty(obj, p, get, set) {
	// obj.__defineGetter__(p, get);
	// obj.__defineSetter__(p, set);
}

function whenReady(cb) {
	callbacks.push(cb);
}

function ready() {
	while (callbacks.length) {
		callbacks.pop()();
	}
}

function extend (a, b) {
	return b;
}
