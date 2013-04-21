function b2Transform (pos, r) {
	this.position = new b2Vec2();
	this.R = new b2Mat22();
	this.Initialize(pos, r);
}

b2Transform.prototype = {
	Initialize : function (pos, r) {
		if (pos) {
			this.position.SetV(pos);
		}
		if (r) {
			this.R.SetM(r);
		}
	},

	SetIdentity : function () {
		this.position.SetZero();
		this.R.SetIdentity();
	},

	Set : function (x) {
		this.position.SetV(x.position);
		this.R.SetM(x.R);
	},

	GetAngle : function () {
		return Math.atan2(this.R.col1.y, this.R.col1.x);
	}
};