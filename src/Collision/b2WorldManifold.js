function b2WorldManifold() {
	var i;
	this.m_normal = new b2Vec2();
	this.m_points = [];

	for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
		this.m_points[i] = new b2Vec2();
	}
}

Box2D.b2WorldManifold = b2WorldManifold;

b2WorldManifold.prototype = {
	Initialize : function (manifold, xfA, radiusA, xfB, radiusB) {
		var i = 0,
			tVec, tMat,
			normalX = 0,
			normalY = 0,
			planePointX = 0,
			planePointY = 0,
			clipPointX = 0,
			clipPointY = 0,
			pointAX, pointAY,
			pointBX, pointBY,
			dX, dY, d, d2,
			cAX, cAY, cBX, cBY;

		radiusA = radiusA || 0;
		radiusB = radiusB || 0;

		if (!manifold.m_pointCount) {
			return;
		}

		switch (manifold.m_type) {
		case b2Manifold.e_circles:
			tMat = xfA.R;
			tVec = manifold.m_localPoint;
			pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			tMat = xfB.R;
			tVec = manifold.m_points[0].m_localPoint;
			pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			dX = pointBX - pointAX;
			dY = pointBY - pointAY;
			d2 = dX * dX + dY * dY;

			if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
				d = Math.sqrt(d2);
				this.m_normal.x = dX / d;
				this.m_normal.y = dY / d;
			} else {
				this.m_normal.x = 1;
				this.m_normal.y = 0;
			}

			cAX = pointAX + radiusA * this.m_normal.x;
			cAY = pointAY + radiusA * this.m_normal.y;
			cBX = pointBX - radiusB * this.m_normal.x;
			cBY = pointBY - radiusB * this.m_normal.y;

			this.m_points[0].x = 0.5 * (cAX + cBX);
			this.m_points[0].y = 0.5 * (cAY + cBY);
			break;
		case b2Manifold.e_faceA:
			tMat = xfA.R;
			tVec = manifold.m_localPlaneNormal;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			tMat = xfA.R;
			tVec = manifold.m_localPoint;
			planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			this.m_normal.x = normalX;
			this.m_normal.y = normalY;

			for (i = 0; i < manifold.m_pointCount; i++) {
				tMat = xfB.R;
				tVec = manifold.m_points[i].m_localPoint;
				clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

				this.m_points[i].x = clipPointX + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
				this.m_points[i].y = clipPointY + 0.5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;
			}
			break;
		case b2Manifold.e_faceB:
			tMat = xfB.R;
			tVec = manifold.m_localPlaneNormal;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			tMat = xfB.R;
			tVec = manifold.m_localPoint;
			planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

			this.m_normal.x = -normalX;
			this.m_normal.y = -normalY;
			for (i = 0; i < manifold.m_pointCount; i++) {
				tMat = xfA.R;
				tVec = manifold.m_points[i].m_localPoint;
				clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

				this.m_points[i].x = clipPointX + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
				this.m_points[i].y = clipPointY + 0.5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;
			}
			break;
		}
	}
};
