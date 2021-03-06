var Box2D = {},
	callbacks = [];

function generateCallback(context, cb) {
	return function () {
		cb.apply(context, arguments);
	};
}

Box2D.generateCallback = generateCallback;

// TODO: Remove in favor of proper getter/setter methods
function defineProperty(obj, p, get, set) {
	if (Object.defineProperty) {
		Object.defineProperty(obj, p, {
			get : get,
			set : set,
			enumerable : false,
			configurable : true
		});
	} else {
		obj.__defineGetter__(p, get);
		obj.__defineSetter__(p, set);
	}
}

function whenReady(cb) {
	callbacks.push(cb);
}

function ready() {
	while (callbacks.length) {
		callbacks.pop()();
	}
}

function inherit(a, b) {
	whenReady(function () {
		var name,
			ap = a.prototype,
			bp = b.prototype;
		for (name in ap) {
			if (ap.hasOwnProperty(name) && !bp.hasOwnProperty(name)) {
				bp[name] = ap[name];
			}
		}
	});
}
