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

defineProperty(b2Color.prototype, 'r',
	function () {
		return this._r / 255;
	},
	function (rr) {
		this._r = ~~(255 * b2Math.Clamp(rr || 0, 0, 1));
	}
);

defineProperty(b2Color.prototype, 'g',
	function () {
		return this._g / 255;
	},
	function (gg) {
		this._g = ~~(255 * b2Math.Clamp(gg || 0, 0, 1));
	}
);

defineProperty(b2Color.prototype, 'b',
	function () {
		return this._b / 255;
	},
	function (bb) {
		this._b = ~~(255 * b2Math.Clamp(bb || 0, 0, 1));
	}
);

defineProperty(b2Color.prototype, 'color',
	function () {
		return (this._r << 16) | (this._g << 8) | (this._b);
	},
	function () {} // TODO: Add setter
);
