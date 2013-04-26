function b2AABB() {
	this.lowerBound = new b2Vec2();
	this.upperBound = new b2Vec2();
}

Box2D.b2AABB = b2AABB;

b2AABB.Combine = function (aabb1, aabb2) {
	var aabb = new b2AABB();
	aabb.Combine(aabb1, aabb2);
	return aabb;
};

b2AABB.prototype = {
	IsValid : function () {
		return this.upperBound.x > this.lowerBound.x &&
			this.upperBound.y > this.lowerBound.y &&
			this.lowerBound.IsValid() &&
			this.upperBound.IsValid();
	},

	GetCenter : function (out) {
		out = out || new b2Vec2(); // TODO: b2Vec2 reuse?
		out.x = (this.lowerBound.x + this.upperBound.x) / 2;
		out.y = (this.lowerBound.y + this.upperBound.y) / 2;
		return out;
	},

	GetExtents : function (out) {
		out = out || new b2Vec2(); // TODO: b2Vec2 reuse?
		out.x = (this.upperBound.x - this.lowerBound.x) / 2;
		out.y = (this.upperBound.y - this.lowerBound.y) / 2;
		return out;
	},

	Contains : function (aabb) {
		return this.lowerBound.x <= aabb.lowerBound.x &&
			this.lowerBound.y <= aabb.lowerBound.y &&
			aabb.upperBound.x <= this.upperBound.x &&
			aabb.upperBound.y <= this.upperBound.y;
	},

	RayCast : function (output, input) {
		var tmin = -Number.MAX_VALUE,
			tmax = Number.MAX_VALUE,
			pX = input.p1.x,
			pY = input.p1.y,
			dX = input.p2.x - input.p1.x,
			dY = input.p2.y - input.p1.y,
			absDX = Math.abs(dX),
			absDY = Math.abs(dY),
			normal = output.normal,
			inv_d = 0,
			t1 = 0,
			t2 = 0,
			t3 = 0,
			s = 0;

		if (absDX < Number.MIN_VALUE) {
			if (pX < this.lowerBound.x || this.upperBound.x < pX) {
				return false;
			}
		} else {
			inv_d = 1 / dX;
			t1 = (this.lowerBound.x - pX) * inv_d;
			t2 = (this.upperBound.x - pX) * inv_d;
			s = -1;
			if (t1 > t2) {
				t3 = t1;
				t1 = t2;
				t2 = t3;
				s = 1;
			}
			if (t1 > tmin) {
				normal.x = s;
				normal.y = 0;
				tmin = t1;
			}
			tmax = Math.min(tmax, t2);
			if (tmin > tmax) {
				return false;
			}
		}
		if (absDY < Number.MIN_VALUE) {
			if (pY < this.lowerBound.y || this.upperBound.y < pY) {
				return false;
			}
		} else {
			inv_d = 1.0 / dY;
			t1 = (this.lowerBound.y - pY) * inv_d;
			t2 = (this.upperBound.y - pY) * inv_d;
			s = -1;
			if (t1 > t2) {
				t3 = t1;
				t1 = t2;
				t2 = t3;
				s = 1;
			}
			if (t1 > tmin) {
				normal.y = s;
				normal.x = 0;
				tmin = t1;
			}
			tmax = Math.min(tmax, t2);
			if (tmin > tmax) {
				return false;
			}
		}
		output.fraction = tmin;
		return true;
	},

	TestOverlap : function (other) {
		if (other.lowerBound.x > this.upperBound.x ||
			other.lowerBound.y > this.upperBound.y ||
			this.lowerBound.x > other.upperBound.x ||
			this.lowerBound.y > other.upperBound.y) {
			return false;
		}
		return true;
	},

	Combine : function (aabb1, aabb2) {
		this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
		this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
		this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
		this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
	}
};
