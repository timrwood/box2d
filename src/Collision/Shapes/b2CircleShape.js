function b2CircleShape (radius) {
	b2Shape.apply(this);
	this.m_p = new b2Vec2();
	this.m_type = b2Shape.e_circleShape;
	this.m_radius = radius || 0;
}

b2CircleShape.prototype = extend(new b2Shape(), {
	Copy : function () {
		var s = new b2CircleShape();
		s.Set(this);
		return s;
	},

	Set : function (other) {
		b2Shape.prototype.Set.call(this, other);
		if (other instanceof b2CircleShape) {
			this.m_p.SetV(other.m_p);
		}
	},

	TestPoint : function (transform, p) {
		var tMat = transform.R,
			dX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
			dY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);

		dX = p.x - dX;
		dY = p.y - dY;

		return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
	},

	RayCast : function (output, input, transform) {
		var tMat = transform.R,
			positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
			positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y),
			sX = input.p1.x - positionX,
			sY = input.p1.y - positionY,
			b = (sX * sX + sY * sY) - this.m_radius * this.m_radius,
			rX = input.p2.x - input.p1.x,
			rY = input.p2.y - input.p1.y,
			c = (sX * rX + sY * rY),
			rr = (rX * rX + rY * rY),
			sigma = c * c - rr * b,
			a;
		if (sigma < 0 || rr < Number.MIN_VALUE) {
			return false;
		}
		a = -(c + Math.sqrt(sigma));
		if (0 <= a && a <= input.maxFraction * rr) {
			a /= rr;
			output.fraction = a;
			output.normal.x = sX + a * rX;
			output.normal.y = sY + a * rY;
			output.normal.Normalize();
			return true;
		}
		return false;
	},

	ComputeAABB : function (aabb, transform) {
		var tMat = transform.R,
			pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
			pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
		aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
		aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
	},

	ComputeMass : function (massData, density) {
		density = density || 0;
		massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
		massData.center.SetV(this.m_p);
		massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
	},

	ComputeSubmergedArea : function (normal, offset, xf, c) {
		offset = offset || 0;

		var p = b2Math.MulX(xf, this.m_p),
			l = -(b2Math.Dot(normal, p) - offset),
			r2, l2,
			area, com;

		if (l < Number.MIN_VALUE -this.m_radius) {
			return 0;
		}
		if (l > this.m_radius) {
			c.SetV(p);
			return Math.PI * this.m_radius * this.m_radius;
		}

		r2 = this.m_radius * this.m_radius;
		l2 = l * l;
		area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
		com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area);
		c.x = p.x + normal.x * com;
		c.y = p.y + normal.y * com;
		return area;
	},

	GetLocalPosition : function () {
		return this.m_p;
	},

	SetLocalPosition : function (position) {
		this.m_p.SetV(position);
	},

	GetRadius : function () {
		return this.m_radius;
	},

	SetRadius : function (radius) {
		this.m_radius = radius || 0;
	}
});