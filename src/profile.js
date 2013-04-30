var counts = {},
	calls = {},
	i;

function beforeMethod(id) {
	counts[id] = (counts[id] || 0) - (b2Vec2.count || 0);
	calls[id] = (calls[id] || 0) + 1;
}

function afterMethod(id) {
	counts[id] = (counts[id] || 0) + (b2Vec2.count || 0);
}

function profile(proto, method, className) {
	var old = proto[method];

	if (typeof old !== "function") {
		return;
	}

	proto[method] = function () {
		var out;
		beforeMethod(className + "." + method);
		out = old.apply(this, arguments);
		afterMethod(className + "." + method);
		return out;
	};
}

function profileClass(object, className) {
	var method,
		proto = object.prototype;


	for (method in object) {
		if (object.hasOwnProperty(method)) {
			profile(object, method, className);
		}
	}

	if (proto) {
		for (method in proto) {
			if (proto.hasOwnProperty(method)) {
				profile(proto, method, className + '.prototype');
			}
		}
	}
}

// for (i in Box2D) {
// 	if (Box2D.hasOwnProperty(i)) {
// 		profileClass(Box2D[i], i);
// 	}
// }

Box2D.profile = function () {
	var i,
		c = [],
		a, b, d;
	for (i in counts) {
		if (counts.hasOwnProperty(i)) {
			c.push([i, counts[i], calls[i]]);
			counts[i] = 0;
			calls[i] = 0;
		}
	}
	c.sort(function (a, b) {
		return b[1] - a[1];
	});
	console.log('\n--profile--')
	for (i = 0; i < c.length; i++) {
		a = c[i][0];
		b = c[i][1];
		d = c[i][2];
		if (b) {
			console.log((b / d).toFixed(1) + ' (' + b + '/' + d + ') : ' + a);
		}
	}
};
