function b2Mat22() {
	this.col1 = new b2Vec2();
	this.col2 = new b2Vec2();
	this.SetIdentity();
}


b2Mat22.prototype = {
	Set : function (angle) {
		angle = angle || 0;

		var c = Math.cos(angle),
			s = Math.sin(angle);

		this.col1.x = c;
		this.col2.x = -s;
		this.col1.y = s;
		this.col2.y = c;
	},

	SetVV : function (c1, c2) {
		this.col1.SetV(c1);
		this.col2.SetV(c2);
	},

	Copy : function () {
		var mat = new b2Mat22();
		mat.SetM(this);
		return mat;
	},

	SetM : function (m) {
		this.col1.SetV(m.col1);
		this.col2.SetV(m.col2);
	},

	AddM : function (m) {
		this.col1.x += m.col1.x;
		this.col1.y += m.col1.y;
		this.col2.x += m.col2.x;
		this.col2.y += m.col2.y;
	},

	SetIdentity : function () {
		this.col1.x = 1;
		this.col2.x = 0;
		this.col1.y = 0;
		this.col2.y = 1;
	},

	SetZero : function () {
		this.col1.x = 0;
		this.col2.x = 0;
		this.col1.y = 0;
		this.col2.y = 0;
	},

	GetAngle : function () {
		return Math.atan2(this.col1.y, this.col1.x);
	},

	GetInverse : function (out) {
		var a = this.col1.x,
			b = this.col2.x,
			c = this.col1.y,
			d = this.col2.y,
			det = a * d - b * c;

		if (det) {
			det = 1 / det;
		}

		out.col1.x =  det * d;
		out.col2.x = -det * b;
		out.col1.y = -det * c;
		out.col2.y =  det * a;

		return out;
	},

	Solve : function (out, bX, bY) {
		bX = bX || 0;
		bY = bY || 0;

		var a11 = this.col1.x,
			a12 = this.col2.x,
			a21 = this.col1.y,
			a22 = this.col2.y,
			det = a11 * a22 - a12 * a21;

		if (det) {
			det = 1 / det;
		}

		out.x = det * (a22 * bX - a12 * bY);
		out.y = det * (a11 * bY - a21 * bX);

		return out;
	},

	Abs : function () {
		this.col1.Abs();
		this.col2.Abs();
	}
};

b2Mat22.FromAngle = function (angle) {
	var mat = new b2Mat22();
	mat.Set(angle || 0);
	return mat;
};

b2Mat22.FromVV = function (c1, c2) {
	var mat = new b2Mat22();
	mat.SetVV(c1, c2);
	return mat;
};