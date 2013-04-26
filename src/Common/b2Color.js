function b2Color() {
	this.Set.apply(this, arguments);
}

Box2D.b2Color = b2Color;

b2Color.prototype = {
	Set : function (rr, gg, bb) {
		this._r = ~~(255 * b2Math.Clamp(rr || 0, 0, 1));
		this._g = ~~(255 * b2Math.Clamp(gg || 0, 0, 1));
		this._b = ~~(255 * b2Math.Clamp(bb || 0, 0, 1));
	}
};

defineProperty(b2Color.prototype, 'r', {
	enumerable: false,
	configurable: true,
	set: function (rr) {
		this._r = ~~(255 * b2Math.Clamp(rr || 0, 0, 1));
	}
});

defineProperty(b2Color.prototype, 'g', {
	enumerable: false,
	configurable: true,
	set: function (gg) {
		this._g = ~~(255 * b2Math.Clamp(gg || 0, 0, 1));
	}
});

defineProperty(b2Color.prototype, 'b', {
	enumerable: false,
	configurable: true,
	set: function (bb) {
		this._b = ~~(255 * b2Math.Clamp(bb || 0, 0, 1));
	}
});

defineProperty(b2Color.prototype, 'color', {
	enumerable: false,
	configurable: true,
	get: function () {
		return (this._r << 16) | (this._g << 8) | (this._b);
	}
});
