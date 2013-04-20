function b2EdgeShape(v1, v2) {
	b2Shape.apply(this, arguments);
	this.s_supportVec = new b2Vec2();
	this.m_v1 = v1.Copy();
	this.m_v2 = v1.Copy();
	this.m_direction = new b2Vec2(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
	this.m_length = this.m_direction.Normalize();
	this.m_normal = new b2Vec2(this.m_direction.y, -this.m_direction.x);
	this.m_type = b2Shape.e_edgeShape;
	this.m_coreV1 = new b2Vec2(
		(-b2Settings.b2_toiSlop * (this.m_normal.x - this.m_direction.x)) + this.m_v1.x,
		(-b2Settings.b2_toiSlop * (this.m_normal.y - this.m_direction.y)) + this.m_v1.y
	);
	this.m_coreV2 = new b2Vec2(
		(-b2Settings.b2_toiSlop * (this.m_normal.x + this.m_direction.x)) + this.m_v2.x,
		(-b2Settings.b2_toiSlop * (this.m_normal.y + this.m_direction.y)) + this.m_v2.y
	);
	this.m_cornerDir1 = this.m_normal.Copy();
	this.m_cornerDir2 = new b2Vec2(-this.m_normal.x, -this.m_normal.y);
	this.m_prevEdge = null;
	this.m_nextEdge = null;
}

b2EdgeShape.prototype = extend(new b2Shape(), {
	TestPoint : function (transform, p) {
		return false;
	},

	RayCast : function (output, input, transform) {
		var tMat,
			rX = input.p2.x - input.p1.x,
			rY = input.p2.y - input.p1.y,
			v1X, v1Y,
			nX, nY,
			k_slop,
			denom,
			bX, bY,
			a,
			mu2,
			nLen;

		tMat = transform.R;
		v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
		v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
		nX = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) - v1Y;
		nY = (-(transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) - v1X));

		k_slop = 100 * Number.MIN_VALUE;
		denom = (-(rX * nX + rY * nY));

		if (denom > k_slop) {
			bX = input.p1.x - v1X;
			bY = input.p1.y - v1Y;
			a = (bX * nX + bY * nY);

			if (0 <= a && a <= input.maxFraction * denom) {
				mu2 = (-rX * bY) + rY * bX;
				if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					output.fraction = a;
					nLen = Math.sqrt(nX * nX + nY * nY);
					output.normal.x = nX / nLen;
					output.normal.y = nY / nLen;
					return true;
				}
			}
		}
		return false;
	},

	ComputeAABB : function (aabb, transform) {
		var tMat = transform.R,
			v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y),
			v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y),
			v2X = transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y),
			v2Y = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
		if (v1X < v2X) {
			aabb.lowerBound.x = v1X;
			aabb.upperBound.x = v2X;
		} else {
			aabb.lowerBound.x = v2X;
			aabb.upperBound.x = v1X;
		}
		if (v1Y < v2Y) {
			aabb.lowerBound.y = v1Y;
			aabb.upperBound.y = v2Y;
		} else {
			aabb.lowerBound.y = v2Y;
			aabb.upperBound.y = v1Y;
		}
	},

	ComputeMass : function (massData, density) {
		density = density || 0;
		massData.mass = 0;
		massData.center.SetV(this.m_v1);
		massData.I = 0;
	},

	ComputeSubmergedArea : function (normal, offset, xf, c) {
		offset = offset || 0;
		var v0 = new b2Vec2(normal.x * offset, normal.y * offset), //TODO: Reuse b2Vec2?
			v1 = b2Math.MulX(xf, this.m_v1),
			v2 = b2Math.MulX(xf, this.m_v2),
			d1 = b2Math.Dot(normal, v1) - offset,
			d2 = b2Math.Dot(normal, v2) - offset;

		if (d1 > 0) {
			if (d2 > 0) {
				return 0;
			} else {
				v1.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
				v1.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
			}
		} else {
			if (d2 > 0) {
				v2.x = (-d2 / (d1 - d2) * v1.x) + d1 / (d1 - d2) * v2.x;
				v2.y = (-d2 / (d1 - d2) * v1.y) + d1 / (d1 - d2) * v2.y;
			}
		}
		c.x = (v0.x + v1.x + v2.x) / 3;
		c.y = (v0.y + v1.y + v2.y) / 3;
		return 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
	},

	GetLength : function () {
		return this.m_length;
	},

	GetVertex1 : function () {
		return this.m_v1;
	},

	GetVertex2 : function () {
		return this.m_v2;
	},

	GetCoreVertex1 : function () {
		return this.m_coreV1;
	},

	GetCoreVertex2 : function () {
		return this.m_coreV2;
	},

	GetNormalVector : function () {
		return this.m_normal;
	},

	GetDirectionVector : function () {
		return this.m_direction;
	},

	GetCorner1Vector : function () {
		return this.m_cornerDir1;
	},

	GetCorner2Vector : function () {
		return this.m_cornerDir2;
	},

	Corner1IsConvex : function () {
		return this.m_cornerConvex1;
	},

	Corner2IsConvex : function () {
		return this.m_cornerConvex2;
	},

	GetFirstVertex : function (xf) {
		var tMat = xf.R;
		return new b2Vec2(
			xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y),
			xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y)
		);
	},

	GetNextEdge : function () {
		return this.m_nextEdge;
	},

	GetPrevEdge : function () {
		return this.m_prevEdge;
	},

	Support : function (xf, dX, dY) {
		dX = dX || 0;
		dY = dY || 0;
		var tMat = xf.R,
			v1X = xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y),
			v1Y = xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y),
			v2X = xf.position.x + (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y),
			v2Y = xf.position.y + (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);

		if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
			this.s_supportVec.x = v1X;
			this.s_supportVec.y = v1Y;
		} else {
			this.s_supportVec.x = v2X;
			this.s_supportVec.y = v2Y;
		}
		return this.s_supportVec;
	},

	SetPrevEdge : function (edge, core, cornerDir, convex) {
		this.m_prevEdge = edge;
		this.m_coreV1 = core;
		this.m_cornerDir1 = cornerDir;
		this.m_cornerConvex1 = convex;
	},

	SetNextEdge : function (edge, core, cornerDir, convex) {
		this.m_nextEdge = edge;
		this.m_coreV2 = core;
		this.m_cornerDir2 = cornerDir;
		this.m_cornerConvex2 = convex;
	}
});