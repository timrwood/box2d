function b2Vec3(x, y, z) {
	this.x = x || 0;
	this.y = y || 0;
	this.z = z || 0;
}

Box2D.b2Vec3 = b2Vec3;

b2Vec3.prototype = {
	SetZero : function () {
		this.x = this.y = this.z = 0;
	},

	Set : function (x, y, z) {
		this.x = x || 0;
		this.y = y || 0;
		this.z = z || 0;
	},

	SetV : function (v) {
		this.x = v.x;
		this.y = v.y;
		this.z = v.z;
	},

	GetNegative : function () {
		return new b2Vec3(-this.x, -this.y, -this.z);
	},

	NegativeSelf : function () {
		this.x = -this.x;
		this.y = -this.y;
		this.z = -this.z;
	},

	Copy : function () {
		return new b2Vec3(this.x, this.y, this.z);
	},

	Add : function (v) {
		this.x += v.x;
		this.y += v.y;
		this.z += v.z;
	},

	Subtract : function (v) {
		this.x -= v.x;
		this.y -= v.y;
		this.z -= v.z;
	},

	Multiply : function (a) {
		this.x *= a || 0;
		this.y *= a || 0;
		this.z *= a || 0;
	}
};
