function b2SeparationFunction() {
	this.m_localPoint = new b2Vec2();
	this.m_axis = new b2Vec2();
}

Box2D.b2SeparationFunction = b2SeparationFunction;

b2SeparationFunction.prototype = {
	Initialize : function (cache, proxyA, transformA, proxyB, transformB) {
		var count = cache.count,
			localPointA, localPointA1, localPointA2,
			localPointB, localPointB1, localPointB2,
			pointAX = 0,
			pointAY = 0,
			pointBX = 0,
			pointBY = 0,
			normalX = 0,
			normalY = 0,
			tMat, tVec,
			tVecA = b2SeparationFunction.t_vec2a,
			tVecB = b2SeparationFunction.t_vec2b,
			tVecC = b2SeparationFunction.t_vec2c,
			s = 0,
			sgn = 0,
			pA, dA, pB, dB,
			a, e, r, c, f, b, t, denom;

		this.m_proxyA = proxyA;
		this.m_proxyB = proxyB;

		b2Settings.b2Assert(count >= 0 && count < 3);

		if (count === 1) {
			this.m_type = b2SeparationFunction.e_points;
			localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
			localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);

			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			this.m_axis.x = pointBX - pointAX;
			this.m_axis.y = pointBY - pointAY;
			this.m_axis.Normalize();
		} else if (cache.indexB[0] === cache.indexB[1]) {
			this.m_type = b2SeparationFunction.e_faceA;
			localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
			localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);

			this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
			this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
			this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1, this.m_axis), 1, this.m_axis);
			this.m_axis.Normalize();

			tVec = this.m_axis;
			tMat = transformA.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			tVec = this.m_localPoint;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
			if (s < 0) {
				this.m_axis.NegativeSelf();
			}
		} else if (cache.indexA[0] === cache.indexA[0]) {
			this.m_type = b2SeparationFunction.e_faceB;
			localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
			localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);

			this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
			this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
			this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1, this.m_axis), 1, this.m_axis);
			this.m_axis.Normalize();

			tVec = this.m_axis;
			tMat = transformB.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			tVec = this.m_localPoint;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

			s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
			if (s < 0) {
				this.m_axis.NegativeSelf();
			}
		} else {
			localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
			localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);

			dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1, tVecA), tVecA);
			dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1, tVecB), tVecB);

			a = dA.x * dA.x + dA.y * dA.y;
			e = dB.x * dB.x + dB.y * dB.y;
			r = b2Math.SubtractVV(dB, dA, tVecC);
			c = dA.x * r.x + dA.y * r.y;
			f = dB.x * r.x + dB.y * r.y;
			b = dA.x * dB.x + dA.y * dB.y;
			denom = a * e - b * b;
			s = 0;

			if (denom !== 0) {
				s = b2Math.Clamp((b * f - c * e) / denom, 0, 1);
			}

			t = (b * s + f) / e;

			if (t < 0) {
				t = 0;
				s = b2Math.Clamp((b - c) / a, 0, 1);
			}

			localPointA = tVecA;
			localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
			localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);

			localPointB = tVecB;
			localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
			localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);

			if (s === 0 || s === 1) {
				this.m_type = b2SeparationFunction.e_faceB;
				this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1, this.m_axis), 1, this.m_axis);
				this.m_axis.Normalize();
				this.m_localPoint = localPointB;

				tVec = this.m_axis;
				tMat = transformB.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

				tVec = this.m_localPoint;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

				tVec = localPointA;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

				s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
				if (s < 0) {
					this.m_axis.NegativeSelf();
				}
			} else {
				this.m_type = b2SeparationFunction.e_faceA;
				this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1, this.m_axis), 1, this.m_axis);
				this.m_localPoint = localPointA;

				tVec = this.m_axis;
				tMat = transformA.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

				tVec = this.m_localPoint;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

				tVec = localPointB;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

				s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;

				if (s < 0) {
					this.m_axis.NegativeSelf();
				}
			}
		}
	},

	Evaluate : function (transformA, transformB) {
		var axisA, axisB,
			localPointA, localPointB,
			pointA, pointB,
			normal,
			tVecA = b2SeparationFunction.t_vec2a,
			tVecB = b2SeparationFunction.t_vec2b,
			tVecC = b2SeparationFunction.t_vec2c;
		// TODO : Check for b2Vec2 reuse?
		switch (this.m_type) {
		case b2SeparationFunction.e_points:
			axisA = b2Math.MulTMV(transformA.R, this.m_axis, tVecA);
			axisB = b2Math.MulTMV(transformB.R, this.m_axis.GetNegative(tVecB), tVecB);

			localPointA = this.m_proxyA.GetSupportVertex(axisA);
			localPointB = this.m_proxyB.GetSupportVertex(axisB);

			pointA = b2Math.MulX(transformA, localPointA, tVecA);
			pointB = b2Math.MulX(transformB, localPointB, tVecB);

			return (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
		case b2SeparationFunction.e_faceA:
			normal = b2Math.MulMV(transformA.R, this.m_axis, tVecA);
			pointA = b2Math.MulX(transformA, this.m_localPoint, tVecB);

			axisB = b2Math.MulTMV(transformB.R, normal.GetNegative(tVecC), tVecC);
			localPointB = this.m_proxyB.GetSupportVertex(axisB);
			pointB = b2Math.MulX(transformB, localPointB, tVecC);

			return (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
		case b2SeparationFunction.e_faceB:
			normal = b2Math.MulMV(transformB.R, this.m_axis, tVecA);
			pointB = b2Math.MulX(transformB, this.m_localPoint, tVecB);

			axisA = b2Math.MulTMV(transformA.R, normal.GetNegative(tVecC), tVecC);
			localPointA = this.m_proxyA.GetSupportVertex(axisA);
			pointA = b2Math.MulX(transformA, localPointA, tVecC);
			return (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
		default:
			b2Settings.b2Assert(false);
			return 0.0;
		}
	}
};

b2SeparationFunction.e_points = 1;
b2SeparationFunction.e_faceA  = 2;
b2SeparationFunction.e_faceB  = 4;


whenReady(function () {
	b2SeparationFunction.t_vec2a = new b2Vec2();
	b2SeparationFunction.t_vec2b = new b2Vec2();
	b2SeparationFunction.t_vec2c = new b2Vec2();
});
