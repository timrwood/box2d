var Box2D = {
	Collision : {
		Shapes : {}
	},
	Common : {
		Math : {}
	},
	Dynamics : {
		Contacts : {},
		Controllers : {},
		Joints : {}
	}
},
callbacks = [];

function generateCallback(context, cb) {
	return function () {
		cb.apply(context, arguments);
	};
}

// TODO: Remove in favor of proper getter/setter methods
function defineProperty(obj, p, get, set) {
	obj.__defineGetter__(p, get);
	obj.__defineSetter__(p, set);
}

function whenReady(cb) {
	callbacks.push(cb);
}

function ready() {
	while (callbacks.length) {
		callbacks.pop()();
	}
}
