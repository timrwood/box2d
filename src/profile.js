var counts = {};

function beforeMethod(id) {

}

function afterMethod(id) {

}

function profile(proto, method, className) {
	var old = proto[method];

	if (typeof old !== "function") {
		return;
	}

	proto[method] = function () {
		beforeMethod(className + ".prototype." + method);
		old.apply(this, arguments);
		afterMethod(className + ".prototype." + method);
	};
}

function profileClass(object, className) {
	var method,
		proto = object[className].prototype;
	for (method in proto) {
		if (proto.hasOwnProperty(method)) {
			profile(proto, method, className);
		}
	}
}
