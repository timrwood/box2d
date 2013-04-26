function b2Vec2(x, y) {
	this.x = x || 0;
	this.y = y || 0;
}

Box2D.b2Vec2 = b2Vec2;

b2Vec2.prototype = {
	SetZero : function () {
		this.x = this.y = 0;
	},

	Set : function (x, y) {
		this.x = x || 0;
		this.y = y || 0;
	},

	SetV : function (v) {
		this.x = v.x;
		this.y = v.y;
	},

	GetNegative : function () {
		return new b2Vec2(-this.x, -this.y);
	},

	NegativeSelf : function () {
		this.x = -this.x;
		this.y = -this.y;
	},

	Copy : function () {
		return new b2Vec2(this.x, this.y);
	},

	Add : function (v) {
		this.x += v.x;
		this.y += v.y;
	},

	Subtract : function (v) {
		this.x -= v.x;
		this.y -= v.y;
	},

	Multiply : function (a) {
		this.x *= a || 0;
		this.y *= a || 0;
	},

	MulM : function (A) {
		var tX = this.x;
		this.x = A.col1.x * tX + A.col2.x * this.y;
		this.y = A.col1.y * tX + A.col2.y * this.y;
	},

	MulTM : function (A) {
		var tX = b2Math.Dot(this, A.col1);
		this.y = b2Math.Dot(this, A.col2);
		this.x = tX;
	},

	CrossVF : function (s) {
		var tX = this.x;
		s = s || 0;
		this.x = s * this.y;
		this.y = -s * tX;
	},

	CrossFV : function (s) {
		var tX = this.x;
		s = s || 0;
		this.x = -s * this.y;
		this.y = s * tX;
	},

	MinV : function (b) {
		this.x = this.x < b.x ? this.x : b.x;
		this.y = this.y < b.y ? this.y : b.y;
	},

	MaxV : function (b) {
		this.x = this.x > b.x ? this.x : b.x;
		this.y = this.y > b.y ? this.y : b.y;
	},

	Abs : function () {
		this.x = Math.abs(this.x);
		this.y = Math.abs(this.y);
	},

	Length : function () {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	},

	LengthSquared : function () {
		return this.x * this.x + this.y * this.y;
	},

	Normalize : function () {
		var length = Math.sqrt(this.x * this.x + this.y * this.y),
			invLength;

		if (length < Number.MIN_VALUE) {
			return 0;
		}

		invLength = 1 / length;
		this.x *= invLength;
		this.y *= invLength;

		return length;
	},

	IsValid : function () {
		return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
	}
};


b2Vec2.Make = function (x, y) {
	return new b2Vec2(x, y);
};
