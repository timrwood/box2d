function b2Sweep() {
	this.localCenter = new b2Vec2();
	this.c0 = new b2Vec2();
	this.c = new b2Vec2();
}

Box2D.b2Sweep = b2Sweep;

b2Sweep.prototype = {
	Set : function (other) {
		this.localCenter.SetV(other.localCenter);
		this.c0.SetV(other.c0);
		this.c.SetV(other.c);
		this.a0 = other.a0;
		this.a = other.a;
		this.t0 = other.t0;
	},

	Copy : function () {
		var copy = new b2Sweep();
		copy.Set(this);
		return copy;
	},

	GetTransform : function (xf, alpha) {
		var angle,
			tMat;

		alpha = alpha || 0;
		xf.position.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
		xf.position.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
		angle = (1 - alpha) * this.a0 + alpha * this.a;
		xf.R.Set(angle);
		tMat = xf.R;
		xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
		xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
	},

	Advance : function (t) {
		var alpha;

		t = t || 0;
		if (this.t0 < t && 1 - this.t0 > Number.MIN_VALUE) {
			alpha = (t - this.t0) / (1 - this.t0);
			this.c0.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
			this.c0.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
			this.a0 = (1 - alpha) * this.a0 + alpha * this.a;
			this.t0 = t;
		}
	}
};
