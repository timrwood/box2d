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

	CrossVF : function (a, s, out) {
		s = s || 0;
		out = out || new b2Vec2();
		out.x = s * a.y;
		out.y = -s * a.x;
		return out;
	},

	CrossFV : function (s, a, out) {
		s = s || 0;
		out = out || new b2Vec2();
		out.x = -s * a.y;
		out.y = s * a.x;
		return out;
	},

	MulMV : function (A, v, out) {
		out = out || new b2Vec2();
		out.x = A.col1.x * v.x + A.col2.x * v.y;
		out.y = A.col1.y * v.x + A.col2.y * v.y;
		return out;
	},

	MulTMV : function (A, v, out) {
		out = out || new b2Vec2();
		out.x = b2Math.Dot(v, A.col1);
		out.y = b2Math.Dot(v, A.col2);
		return out;
	},

	MulX : function (T, v, out) {
		var a = b2Math.MulMV(T.R, v, out);
		a.x += T.position.x;
		a.y += T.position.y;
		return a;
	},

	MulXT : function (T, v, out) {
		var a = b2Math.SubtractVV(v, T.position, out),
			tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);

		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
		a.x = tX;

		return a;
	},

	AddVV : function (a, b, out) {
		out = out || new b2Vec2();
		out.x = a.x + b.x;
		out.y = a.y + b.y;
		return out;
	},

	SubtractVV : function (a, b, out) {
		out = out || new b2Vec2();
		out.x = a.x - b.x;
		out.y = a.y - b.y;
		return out;
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

	MulFV : function (s, a, out) {
		s = s || 0;
		out = out || new b2Vec2();
		out.x = s * a.x;
		out.y = s * a.y;
		return out;
	},

	AddMM : function (A, B) {
		return b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
	},

	MulMM : function (A, B) {
		return b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
	},

	MulTMM : function (A, B) {
		var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1)),
			c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
		return b2Mat22.FromVV(c1, c2);
	},

	Abs : function (a) {
		return Math.abs(a);
	},

	AbsV : function (a, out) {
		out = out || new b2Vec2();
		out.x = Math.abs(a.x);
		out.y = Math.abs(a.y);
		return out;
	},

	AbsM : function (A) {
		return b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
	},

	Min : function (a, b) {
		return Math.min(a, b);
	},

	MinV : function (a, b, out) {
		out = out || new b2Vec2();
		out.x = Math.min(a.x, b.x);
		out.y = Math.min(a.y, b.y);
		return out;
	},

	Max : function (a, b) {
		return Math.max(a, b);
	},

	MaxV : function (a, b, out) {
		out = out || new b2Vec2();
		out.x = Math.max(a.x, b.x);
		out.y = Math.max(a.y, b.y);
		return out;
	},

	Clamp : function (a, low, high) {
		return Math.max(low, Math.min(a, high));
	},

	ClampV : function (a, low, high, out) {
		var min = b2Math.MinV(a, high, out);
		return b2Math.MaxV(low, min, min);
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

Box2D.b2Math = b2Math;

whenReady(function () {
	b2Math.b2Vec2_zero = new b2Vec2();
	b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1, 0), new b2Vec2(0, 1));
	b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
});
