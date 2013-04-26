function b2Segment() {
	this.p1 = new b2Vec2();
	this.p2 = new b2Vec2();
}

Box2D.b2Segment = b2Segment;

b2Segment.prototype = {
	TestSegment : function (lambda, normal, segment, maxLambda) {
		maxLambda = maxLambda || 0;

		var s = segment.p1,
			rX = segment.p2.x - s.x,
			rY = segment.p2.y - s.y,
			dX = this.p2.x - this.p1.x,
			dY = this.p2.y - this.p1.y,
			nX = dY,
			nY = -dX,
			k_slop = 100.0 * Number.MIN_VALUE,
			denom = (-(rX * nX + rY * nY)),
			bX, bY, a,
			mu2, nLen;

		if (denom > k_slop) {
			bX = s.x - this.p1.x;
			bY = s.y - this.p1.y;
			a = (bX * nX + bY * nY);
			if (a >= 0 && a <= maxLambda * denom) {
				mu2 = (-rX * bY) + rY * bX;

				if (mu2 >= -k_slop * denom && mu2 <= denom * (1.0 + k_slop)) {
					a /= denom;
					nLen = Math.sqrt(nX * nX + nY * nY);
					nX /= nLen;
					nY /= nLen;
					lambda[0] = a;
					normal.Set(nX, nY);
					return true;
				}
			}
		}
		return false;
	},

	Extend : function (aabb) {
		this.ExtendForward(aabb);
		this.ExtendBackward(aabb);
	},

	ExtendForward : function (aabb) {
		var p1 = this.p1,
			p2 = this.p2,
			dX = p2.x - p1.x,
			dY = p2.y - p1.y,
			lambdaX, lambdaY, lambda;

		if (dX > 0) {
			lambdaX = (aabb.upperBound.x - p1.x) / dX;
		} else if (dX < 0) {
			lambdaX = (aabb.lowerBound.x - p1.x) / dX;
		} else {
			lambdaX = Number.POSITIVE_INFINITY;
		}

		if (dY > 0) {
			lambdaY = (aabb.upperBound.y - p1.y) / dY;
		} else if (dY < 0) {
			lambdaY = (aabb.lowerBound.y - p1.y) / dY;
		} else {
			lambdaY = Number.POSITIVE_INFINITY;
		}

		lambda = Math.min(lambdaX, lambdaY);

		p2.x = p1.x + dX * lambda;
		p2.y = p1.y + dY * lambda;
	},

	ExtendBackward : function (aabb) {
		var p1 = this.p1,
			p2 = this.p2,
			dX = p1.x - p2.x,
			dY = p1.y - p2.y,
			lambdaX, lambdaY, lambda;

		if (dX > 0) {
			lambdaX = (aabb.upperBound.x - p2.x) / dX;
		} else if (dX < 0) {
			lambdaX = (aabb.lowerBound.x - p2.x) / dX;
		} else {
			lambdaX = Number.POSITIVE_INFINITY;
		}

		if (dY > 0) {
			lambdaY = (aabb.upperBound.y - p2.y) / dY;
		} else if (dY < 0) {
			lambdaY = (aabb.lowerBound.y - p2.y) / dY;
		} else {
			lambdaY = Number.POSITIVE_INFINITY;
		}

		lambda = Math.min(lambdaX, lambdaY);

		p1.x = p2.x + dX * lambda;
		p1.y = p2.y + dY * lambda;
	}
};
