var b2Math = {
	IsValid : function (x) {
		return isFinite(x || 0);
	},

	Dot : function (a, b) {
		return a.x * b.x + a.y * b.y;
	},

	CrossVV : function (a, b) {
		return a.x * b.y - a.y * b.x;
	},

	CrossVF : function (a, s) {
		s = s || 0;
		return new b2Vec2(s * a.y, -s * a.x);
	},

	CrossFV : function (s, a) {
		s = s || 0;
		return new b2Vec2(-s * a.y, s * a.x);
	},

	MulMV : function (A, v) {
		return new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
	},

	MulTMV : function (A, v) {
		return new b2Vec2(b2Math.Dot(v, A.col1), Dot(v, A.col2));
	},

	MulX : function (T, v) {
		var a = b2Math.MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		return a;
	},

	MulXT : function (T, v) {
		var a = b2Math.SubtractVV(v, T.position),
			tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);

		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
		a.x = tX;

		return a;
	},

	AddVV : function (a, b) {
		return new b2Vec2(a.x + b.x, a.y + b.y);
	},

	SubtractVV : function (a, b) {
		return new b2Vec2(a.x - b.x, a.y - b.y);
	},

	Distance : function (a, b) {
		var cX = a.x - b.x,
			cY = a.y - b.y;
		return Math.sqrt(cX * cX + cY * cY);
	},

	DistanceSquared : function (a, b) {
		var cX = a.x - b.x,
			cY = a.y - b.y;
		return (cX * cX + cY * cY);
	},

	MulFV : function (s, a) {
		s = s || 0;
		return new b2Vec2(s * a.x, s * a.y);
	},

	AddMM : function (A, B) {
		return b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
	},

	MulMM : function (A, B) {
		return b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
	},

	MulTMM : function (A, B) {
		var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
		var c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
		var C = b2Mat22.FromVV(c1, c2);
		return C;
	},

	Abs : function (a) {
		return Math.abs(a);
	},

	AbsV : function (a) {
		return new b2Vec2(Math.abs(a.x), Math.abs(a.y));
	},

	AbsM : function (A) {
		return b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
	},

	Min : function (a, b) {
		return Math.min(a, b);
	},

	MinV : function (a, b) {
		return new b2Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y));
	},

	Max : function (a, b) {
		return Math.max(a, b);
	},

	MaxV : function (a, b) {
		return new b2Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y));
	},

	Clamp : function (a, low, high) {
		return Math.max(low, Math.min(a, high));
	},

	ClampV : function (a, low, high) {
		return b2Math.MaxV(low, b2Math.MinV(a, high));
	},

	Swap : function (a, b) {
		var tmp = a[0];
		a[0] = b[0];
		b[0] = tmp;
	},

	Random : function () {
		return Math.random() * 2 - 1;
	},

	RandomRange : function (lo, hi) {
		lo = lo || 0;
		hi = hi || 0;
		return (hi - lo) * Math.random() + lo;
	},

	NextPowerOfTwo : function (x) {
		x = x || 0;
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16) & 0x0000FFFF;
		return x + 1;
	},

	IsPowerOfTwo : function (x) {
		x = x || 0;
		return x > 0 && (x & (x - 1)) === 0;
	}
};
