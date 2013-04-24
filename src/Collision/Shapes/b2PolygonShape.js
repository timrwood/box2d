function b2PolygonShape() {
	b2Shape.apply(this, arguments);
	this.m_type = b2Shape.e_polygonShape;
	this.m_centroid = new b2Vec2();
	this.m_vertices = [];
	this.m_normals = [];
}

b2PolygonShape.prototype = extend(new b2Shape(), {
	Copy : function () {
		var s = new b2PolygonShape();
		s.Set(this);
		return s;
	},

	Set : function (other) {
		var i;
		b2Shape.prototype.Set.call(this, other);
		if (other instanceof b2PolygonShape) {
			this.m_centroid.SetV(other2.m_centroid);
			this.m_vertexCount = other2.m_vertexCount;
			this.Reserve(this.m_vertexCount);
			for (i = 0; i < this.m_vertexCount; i++) {
				this.m_vertices[i].SetV(other2.m_vertices[i]);
				this.m_normals[i].SetV(other2.m_normals[i]);
			}
		}
	},

	SetAsArray : function (vertices, vertexCount) {
		var i,
			i2,
			edge;

		vertexCount = vertexCount || vertices.length;

		b2Settings.b2Assert(vertexCount > 1);

		this.m_vertexCount = vertexCount;
		this.Reserve(vertexCount);

		for (i = 0; i < vertexCount; i++) {
			this.m_vertices[i].SetV(vertices[i]);
		}
		for (i = 0; i < vertexCount; i++) {
			i2 = (i + 1) % vertexCount;
			edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i]); // TODO b2Vec2 reuse?
			b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
			this.m_normals[i].SetV(b2Math.CrossVF(edge, 1));
			this.m_normals[i].Normalize();
		}
		this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
	},

	SetAsBox : function (hx, hy) {
		hx = hx || 0;
		hy = hy || 0;

		this.m_vertexCount = 4;
		this.Reserve(4);

		this.m_vertices[0].Set(-hx, -hy);
		this.m_vertices[1].Set( hx, -hy);
		this.m_vertices[2].Set( hx,  hy);
		this.m_vertices[3].Set(-hx,  hy);

		this.m_normals[0].Set( 0, -1);
		this.m_normals[1].Set( 1,  0);
		this.m_normals[2].Set( 0,  1);
		this.m_normals[3].Set(-1,  0);

		this.m_centroid.SetZero();
	},

	SetAsOrientedBox : function (hx, hy, center, angle) {
		var xf = new b2Transform(),
			i;

		hx = hx || 0;
		hy = hy || 0;

		angle = angle || 0;
		this.m_vertexCount = 4;
		this.Reserve(4);

		this.m_vertices[0].Set(-hx, -hy);
		this.m_vertices[1].Set( hx, -hy);
		this.m_vertices[2].Set( hx,  hy);
		this.m_vertices[3].Set(-hx,  hy);

		this.m_normals[0].Set( 0, -1);
		this.m_normals[1].Set( 1,  0);
		this.m_normals[2].Set( 0,  1);
		this.m_normals[3].Set(-1,  0);

		this.m_centroid.Set(center.x, center.y);

		xf.position = center;
		xf.R.Set(angle);

		for (i = 0; i < 4; i++) {
			this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]); // TODO: Reuse b2Vec2?
			this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]); // TODO: Reuse b2Vec2?
		}
	},

	SetAsEdge : function (v1, v2) {
		this.m_vertexCount = 2;
		this.Reserve(2);

		this.m_vertices[0].SetV(v1);
		this.m_vertices[1].SetV(v2);

		this.m_centroid.x = 0.5 * (v1.x + v2.x);
		this.m_centroid.y = 0.5 * (v1.y + v2.y);

		this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0); // TODO: Reuse b2Vec2?
		this.m_normals[0].Normalize();
		this.m_normals[1].x = -this.m_normals[0].x;
		this.m_normals[1].y = -this.m_normals[0].y;
	},

	TestPoint : function (xf, p) {
		var tVec,
			tMat = xf.R,
			tX = p.x - xf.position.x,
			tY = p.y - xf.position.y,
			pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y),
			pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y),
			i, dot;

		for (i = 0; i < this.m_vertexCount; ++i) {
			tVec = this.m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			tVec = this.m_normals[i];
			dot = (tVec.x * tX + tVec.y * tY);
			if (dot > 0) {
				return false;
			}
		}
		return true;
	},

	RayCast : function (output, input, transform) {
		var lower = 0,
			upper = input.maxFraction,
			tX = input.p1.x - transform.position.x,
			tY = input.p1.y - transform.position.y,
			tMat = transform.R,
			tVec,
			p1X = (tX * tMat.col1.x + tY * tMat.col1.y),
			p1Y = (tX * tMat.col2.x + tY * tMat.col2.y),
			p2X, p2Y,
			dX, dY,
			index,
			numerator,
			denominator;

		tX = input.p2.x - transform.position.x;
		tY = input.p2.y - transform.position.y;
		tMat = transform.R;
		p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
		p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);

		dX = p2X - p1X;
		dY = p2Y - p1Y;
		index = -1;

		for (i = 0; i < this.m_vertexCount; i++) {
			tVec = this.m_vertices[i];
			tX = tVec.x - p1X;
			tY = tVec.y - p1Y;
			tVec = this.m_normals[i];

			numerator = (tVec.x * tX + tVec.y * tY);
			denominator = (tVec.x * dX + tVec.y * dY);

			if (denominator === 0) {
				if (numerator < 0) {
					return false;
				}
			} else {
				if (denominator < 0 && numerator < lower * denominator) {
					lower = numerator / denominator;
					index = i;
				} else if (denominator > 0 && numerator < upper * denominator) {
					upper = numerator / denominator;
				}
			}
			if (upper < lower - Number.MIN_VALUE) {
				return false;
			}
		}
		if (index >= 0) {
			output.fraction = lower;
			tMat = transform.R;
			tVec = this.m_normals[index];
			output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			return true;
		}
		return false;
	},

	ComputeAABB : function (aabb, xf) {
		var tMat = xf.R,
			tVec = this.m_vertices[0],
			lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y),
			lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y),
			upperX = lowerX,
			upperY = lowerY,
			i, vX, vY;

		for (i = 1; i < this.m_vertexCount; i++) {
			tVec = this.m_vertices[i];
			vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			lowerX = lowerX < vX ? lowerX : vX;
			lowerY = lowerY < vY ? lowerY : vY;
			upperX = upperX > vX ? upperX : vX;
			upperY = upperY > vY ? upperY : vY;
		}

		aabb.lowerBound.x = lowerX - this.m_radius;
		aabb.lowerBound.y = lowerY - this.m_radius;
		aabb.upperBound.x = upperX + this.m_radius;
		aabb.upperBound.y = upperY + this.m_radius;
	},

	// TODO: Check this against the C++ source
	ComputeMass : function (massData, density) {
		var centerX = 0,
			centerY = 0,
			area = 0,
			I = 0,
			k_inv3 = 1 / 3,
			p2, p3,
			i,
			e1X, e1Y,
			e2X, e2Y, D,
			triangleArea,
			intx2, inty2;

		density = density || 0;

		if (this.m_vertexCount === 2) {
			massData.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x);
			massData.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y);
			massData.mass = 0;
			massData.I = 0;
			return;
		}

		for (i = 0; i < this.m_vertexCount; i++) {
			p2 = this.m_vertices[i];
			p3 = this.m_vertices[(i + 1) % this.m_vertexCount];
			e1X = p2.x;
			e1Y = p2.y;
			e2X = p3.x;
			e2Y = p3.y;
			D = e1X * e2Y - e1Y * e2X;
			triangleArea = 0.5 * D;
			area += triangleArea;
			centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
			centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
			intx2 = k_inv3 * (0.25 * (e1X * e1X + e2X * e1X + e2X * e2X));
			inty2 = k_inv3 * (0.25 * (e1Y * e1Y + e2Y * e1Y + e2Y * e2Y));
			I += D * (intx2 + inty2);
		}

		massData.mass = density * area;
		centerX *= 1 / area;
		centerY *= 1 / area;
		massData.center.Set(centerX, centerY);
		massData.I = density * I;
	},

	ComputeSubmergedArea : function (normal, offset, xf, c) {
		offset = offset || 0;

		var normalL = b2Math.MulTMV(xf.R, normal),
			offsetL = offset - b2Math.Dot(normal, xf.position),
			depths = [],
			diveCount = 0,
			intoIndex = -1,
			outoIndex = -1,
			lastSubmerged = false,
			i,
			isSubmerged,
			md,
			intoIndex2, outoIndex2,
			intoLamdda, outoLamdda,
			intoVec, outoVec,
			area,
			center,
			p2, p3,
			triangleArea;

		for (i = 0; i < this.m_vertexCount; i++) {
			depths[i] = b2Math.Dot(normalL, this.m_vertices[i]) - offsetL;
			isSubmerged = depths[i] < -Number.MIN_VALUE;
			if (i > 0) {
				if (isSubmerged) {
					if (!lastSubmerged) {
						intoIndex = i - 1;
						diveCount++;
					}
				} else {
					if (lastSubmerged) {
						outoIndex = i - 1;
						diveCount++;
					}
				}
			}
			lastSubmerged = isSubmerged;
		}
		switch (diveCount) {
		case 0:
			if (lastSubmerged) {
				md = new b2MassData();
				this.ComputeMass(md, 1);
				c.SetV(b2Math.MulX(xf, md.center));
				return md.mass;
			}
			return 0;
		case 1:
			if (intoIndex === -1) {
				intoIndex = this.m_vertexCount - 1;
			} else {
				outoIndex = this.m_vertexCount - 1;
			}
			break;
		}

		intoIndex2 = (intoIndex + 1) % this.m_vertexCount;
		outoIndex2 = (outoIndex + 1) % this.m_vertexCount;
		intoLamdda = -depths[intoIndex] / (depths[intoIndex2] - depths[intoIndex]);
		outoLamdda = -depths[outoIndex] / (depths[outoIndex2] - depths[outoIndex]);

		intoVec = new b2Vec2(
			this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda,
			this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda
		);
		outoVec = new b2Vec2(
			this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda,
			this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda
		);

		area = 0;
		center = new b2Vec2();
		p2 = this.m_vertices[intoIndex2];
		i = intoIndex2;

		while (i !== outoIndex2) {
			i = (i + 1) % this.m_vertexCount;
			if (i == outoIndex2) {
				p3 = outoVec;
			} else {
				p3 = this.m_vertices[i];
			}
			triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
			area += triangleArea;
			center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
			center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
			p2 = p3;
		}

		center.Multiply(1 / area); // TODO Reuse b2Vec2?
		c.SetV(b2Math.MulX(xf, center)); // TODO Reuse b2Vec2?
		return area;
	},

	GetVertexCount : function () {
		return this.m_vertexCount;
	},

	GetVertices : function () {
		return this.m_vertices;
	},

	GetNormals : function () {
		return this.m_normals;
	},

	GetSupport : function (d) {
		var bestIndex = 0,
			bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y,
			i,
			value;
		for (i = 1; i < this.m_vertexCount; i++) {
			value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	},

	GetSupportVertex : function (d) {
		return this.m_vertices[this.GetSupport(d)];
	},

	Validate : function () {
		return false;
	},

	Reserve : function (count) {
		var i;
		count = count || 0;
		for (i = this.m_vertices.length; i < count; i++) {
			this.m_vertices[i] = new b2Vec2();
			this.m_normals[i] = new b2Vec2();
		}
	}
});

// Aliased method
b2PolygonShape.prototype.SetAsVector = b2PolygonShape.prototype.SetAsArray;

b2PolygonShape.ComputeCentroid = function (vs, count) {
	count = count || vs.length;

	var c = new b2Vec2(),
		area = 0,
		inv3 = 1 / 3,
		i,
		p2, p3,
		D,
		triangleArea;

	for (i = 0; i < count; i++) {
		p2 = vs[i],
		p3 = vs[(i + 1) % count];
		D = (p2.x * p3.y - p2.y * p3.x);
		triangleArea = D / 2;
		area += triangleArea;
		c.x += triangleArea * inv3 * (p2.x + p3.x);
		c.y += triangleArea * inv3 * (p2.y + p3.y);
	}

	c.x *= 1 / area;
	c.y *= 1 / area;

	return c;
};

b2PolygonShape.ComputeOBB = function (obb, vs, count) {
	count = count || 0;

	var i, j,
		p = [],
		minArea = Number.MAX_VALUE,
		root,
		uxX, uxY,
		uyX, uyY,
		length,
		lowerX, lowerY,
		upperX, upperY,
		dX, dY,
		rX, rY,
		area,
		centerX, centerY,
		tMat;


	for (i = 0; i < count; ++i) {
		p[i] = vs[i];
	}
	p[count] = p[0];

	for (i = 1; i <= count; i++) {
		root = p[i - 1];
		uxX = p[i].x - root.x;
		uxY = p[i].y - root.y;

		length = Math.sqrt(uxX * uxX + uxY * uxY);
		uxX /= length;
		uxY /= length;

		uyX = -uxY;
		uyY = uxX;

		lowerX = Number.MAX_VALUE;
		lowerY = Number.MAX_VALUE;
		upperX = -Number.MAX_VALUE;
		upperY = -Number.MAX_VALUE;

		for (j = 0; j < count; j++) {
			dX = p[j].x - root.x;
			dY = p[j].y - root.y;
			rX = (uxX * dX + uxY * dY);
			rY = (uyX * dX + uyY * dY);
			if (rX < lowerX) {
				lowerX = rX;
			}
			if (rY < lowerY) {
				lowerY = rY;
			}
			if (rX > upperX) {
				upperX = rX;
			}
			if (rY > upperY) {
				upperY = rY;
			}
		}

		area = (upperX - lowerX) * (upperY - lowerY);

		if (area < 0.95 * minArea) {
			minArea = area;

			obb.R.col1.x = uxX;
			obb.R.col1.y = uxY;
			obb.R.col2.x = uyX;
			obb.R.col2.y = uyY;

			centerX = 0.5 * (lowerX + upperX);
			centerY = 0.5 * (lowerY + upperY);
			tMat = obb.R;

			obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
			obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);

			obb.extents.x = 0.5 * (upperX - lowerX);
			obb.extents.y = 0.5 * (upperY - lowerY);
		}
	}
};

b2PolygonShape.AsArray = function (vertices, vertexCount) {
	var polygonShape = new b2PolygonShape();
	polygonShape.SetAsArray(vertices, vertexCount);
	return polygonShape;
};

b2PolygonShape.AsVector = b2PolygonShape.AsArray;

b2PolygonShape.AsBox = function (hx, hy) {
	var polygonShape = new b2PolygonShape();
	polygonShape.SetAsBox(hx, hy);
	return polygonShape;
};

b2PolygonShape.AsEdge = function (v1, v2) {
	var polygonShape = new b2PolygonShape();
	polygonShape.SetAsEdge(v1, v2);
	return polygonShape;
};

b2PolygonShape.AsOrientedBox = function (hx, hy, center, angle) {
	var polygonShape = new b2PolygonShape();
	polygonShape.SetAsOrientedBox(hx, hy, center, angle);
	return polygonShape;
};

whenReady(function () {
	b2PolygonShape.s_mat = new b2Mat22();
});
