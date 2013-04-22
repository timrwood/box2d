var b2Collision = {
	s_edgeAO : [0],
	s_edgeBO : [0],

	ClipSegmentToLine : function (vOut, vIn, normal, offset) {
		offset = offset || 0;
		var interp, tVec,
			numOut = 0,
			vIn0 = vIn[0].v,
			vIn1 = vIn[1].v,
			distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset,
			distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
		if (distance0 <= 0) {
			vOut[numOut++].Set(vIn[0]);
		}
		if (distance1 <= 0) {
			vOut[numOut++].Set(vIn[1]);
		}
		if (distance0 * distance1 < 0) {
			interp = distance0 / (distance0 - distance1);
			tVec = vOut[numOut].v;
			tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
			tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
			if (distance0 > 0) {
				vOut[numOut].id = vIn[0].id;
			} else {
				vOut[numOut].id = vIn[1].id;
			}
			numOut++;
		}
		return numOut;
	},

	EdgeSeparation : function (poly1, xf1, edge1, poly2, xf2) {
		edge1 = edge1 || 0;
		var count1 = poly1.m_vertexCount,
			vertices1 = poly1.m_vertices,
			normals1 = poly1.m_normals,
			count2 = poly2.m_vertexCount,
			vertices2 = poly2.m_vertices,
			tMat = xf1.R,
			tVec = normals1[edge1],
			normal1WorldX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y,
			normal1WorldY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y,
			normal1X, normal1Y,
			index = 0,
			minDot = Number.MAX_VALUE,
			i,
			dot,
			v1X, v1Y,
			v2X, v2Y;

		tMat = xf2.R;
		normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
		normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);

		for (i = 0; i < count2; i++) {
			tVec = vertices2[i];
			dot = tVec.x * normal1X + tVec.y * normal1Y;
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

		tVec = vertices1[edge1];
		tMat = xf1.R;
		v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		tVec = vertices2[index];
		tMat = xf2.R;
		v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		v2X -= v1X;
		v2Y -= v1Y;

		return v2X * normal1WorldX + v2Y * normal1WorldY;
	},

	FindMaxSeparation : function (edgeIndex, poly1, xf1, poly2, xf2) {
		var count1 = poly1.m_vertexCount,
			normals1 = poly1.m_normals,
			tVec, tMat,
			dX, dY,
			dLocal1X, dLocal1Y,
			edge = 0,
			maxDot = -Number.MAX_VALUE,
			i,
			dot,
			s,
			prevEdge,
			sPrev,
			nextEdge,
			sNext,
			bestEdge,
			bestSeparation,
			increment;

		tMat = xf2.R;
		tVec = poly2.m_centroid;
		dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		tMat = xf1.R;
		tVec = poly1.m_centroid;
		dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
		dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);

		for (i = 0; i < count1; i++) {
			tVec = normals1[i];
			dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
			if (dot > maxDot) {
				maxDot = dot;
				edge = i;
			}
		}

		s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);

		prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

		nextEdge = edge + 1 < count1 ? edge + 1 : 0;
		sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

		bestEdge = 0;
		bestSeparation = 0;
		increment = 0;

		if (sPrev > s && sPrev > sNext) {
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		} else if (sNext > s) {
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		} else {
			edgeIndex[0] = edge;
			return s;
		}
		while (true) {
			if (increment === -1) {
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			} else {
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			}
			s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
			if (s > bestSeparation) {
				bestEdge = edge;
				bestSeparation = s;
			} else {
				break;
			}
		}
		edgeIndex[0] = bestEdge;
		return bestSeparation;
	},

	FindIncidentEdge : function (c, poly1, xf1, edge1, poly2, xf2) {
		edge1 = edge1 || 0;
		var count1 = poly1.m_vertexCount,
			normals1 = poly1.m_normals,
			count2 = poly2.m_vertexCount,
			vertices2 = poly2.m_vertices,
			normals2 = poly2.m_normals,
			tMat, tVec,
			normal1X, normal1Y,
			index = 0,
			minDot = Number.MAX_VALUE,
			i,
			dot,
			tClip,
			i1, i2;


		tMat = xf1.R;
		tVec = normals1[edge1];
		normal1X = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
		normal1Y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;

		tMat = xf2.R;
		normal1Y = tMat.col2.x * normal1X + tMat.col2.y * normal1Y;
		normal1X = tMat.col1.x * normal1X + tMat.col1.y * normal1Y;

		for (i = 0; i < count2; i++) {
			tVec = normals2[i];
			dot = normal1X * tVec.x + normal1Y * tVec.y;
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}
		i1 = index;
		i2 = i1 + 1 < count2 ? i1 + 1 : 0;

		tClip = c[0];
		tVec = vertices2[i1];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i1;
		tClip.id.features.incidentVertex = 0;

		tClip = c[1];
		tVec = vertices2[i2];
		tMat = xf2.R;
		tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		tClip.id.features.referenceEdge = edge1;
		tClip.id.features.incidentEdge = i2;
		tClip.id.features.incidentVertex = 1;
	},

	MakeClipPointVector : function () {
		return [
			new ClipVertex(),
			new ClipVertex()
		];
	},

	CollidePolygons : function (manifold, polyA, xfA, polyB, xfB) {
		var totalRadius = polyA.m_radius + polyB.m_radius,
			edgeA, edgeB,
			separationA, separationB,
			poly1, poly2,
			xf1, xf2,
			edge1 = 0,
			flip = 0,
			k_relativeTol = 0.98,
			k_absoluteTol = 0.001,
			tMat,
			cv,
			incidentEdge,
			count1,
			vertices1,
			local_v11, local_v12,
			localTangent, localNormal,
			planePoint,
			tangent, tangent2,
			normal,
			v11, v12,
			frontOffset,
			sideOffset1, sideOffset2,
			clipPoints1, clipPoints2,
			np = 0,
			pointCount = 0,
			i,
			separation,
			cp,
			tX,
			tY;

		b2Collision.s_edgeAO[0] = b2Collision.s_edgeBO[0] = 0;

		separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
		edgeA = b2Collision.s_edgeAO[0];

		if (separationA > totalRadius) {
			return;
		}

		separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
		edgeB = b2Collision.s_edgeBO[0];

		if (separationB > totalRadius) {
			return;
		}

		manifold.m_pointCount = 0;

		if (separationB > k_relativeTol * separationA + k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = edgeB;
			manifold.m_type = b2Manifold.e_faceB;
			flip = 1;
		} else {
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			edge1 = edgeA;
			manifold.m_type = b2Manifold.e_faceA;
			flip = 0;
		}

		incidentEdge = b2Collision.s_incidentEdge;
		b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

		count1 = poly1.m_vertexCount;
		vertices1 = poly1.m_vertices;
		local_v11 = vertices1[edge1];

		if (edge1 + 1 < count1) {
			local_v12 = vertices1[edge1 + 1];
		} else {
			local_v12 = vertices1[0];
		}

		localTangent = b2Collision.s_localTangent;
		localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
		localTangent.Normalize();

		localNormal = b2Collision.s_localNormal;
		localNormal.x = localTangent.y;
		localNormal.y = -localTangent.x;

		planePoint = b2Collision.s_planePoint;
		planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));

		tangent = b2Collision.s_tangent;
		tMat = xf1.R;
		tangent.x = tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y;
		tangent.y = tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y;

		tangent2 = b2Collision.s_tangent2;
		tangent2.x = -tangent.x;
		tangent2.y = -tangent.y;

		normal = b2Collision.s_normal;
		normal.x = tangent.y;
		normal.y = -tangent.x;

		v11 = b2Collision.s_v11;
		v12 = b2Collision.s_v12;
		v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
		v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
		v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
		v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);

		frontOffset = normal.x * v11.x + normal.y * v11.y;
		sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius;
		sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
		clipPoints1 = b2Collision.s_clipPoints1;
		clipPoints2 = b2Collision.s_clipPoints2;

		np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);
		if (np < 2) {
			return;
		}

		np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
		if (np < 2) {
			return;
		}

		manifold.m_localPlaneNormal.SetV(localNormal);
		manifold.m_localPoint.SetV(planePoint);

		for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			cv = clipPoints2[i];
			separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
			if (separation <= totalRadius) {
				cp = manifold.m_points[pointCount];
				tMat = xf2.R;
				tX = cv.v.x - xf2.position.x;
				tY = cv.v.y - xf2.position.y;
				cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y);
				cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y);
				cp.m_id.Set(cv.id);
				cp.m_id.features.flip = flip;
				pointCount++;
			}
		}
		manifold.m_pointCount = pointCount;
	},

	CollideCircles : function (manifold, circle1, xf1, circle2, xf2) {
		var tMat, tVec,
			p1X, p1Y,
			p2X, p2Y,
			dX, dY,
			distSqr, radius;

		manifold.m_pointCount = 0;

		tMat = xf1.R;
		tVec = circle1.m_p;
		p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		tMat = xf2.R;
		tVec = circle2.m_p;
		p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

		dX = p2X - p1X;
		dY = p2Y - p1Y;
		distSqr = dX * dX + dY * dY;
		radius = circle1.m_radius + circle2.m_radius;

		if (distSqr > radius * radius) {
			return;
		}

		manifold.m_type = b2Manifold.e_circles;
		manifold.m_localPoint.SetV(circle1.m_p);
		manifold.m_localPlaneNormal.SetZero();
		manifold.m_pointCount = 1;
		manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
		manifold.m_points[0].m_id.key = 0;
	},

	CollidePolygonAndCircle : function (manifold, polygon, xf1, circle, xf2) {
		var tPoint,
			dX = 0,
			dY = 0,
			positionX = 0,
			positionY = 0,
			tVec, tMat,
			cX, cY,
			cLocalX, cLocalY,
			dist = 0,
			normalIndex = 0,
			separation = -Number.MAX_VALUE,
			radius = polygon.m_radius + circle.m_radius,
			vertexCount = polygon.m_vertexCount,
			vertices = polygon.m_vertices,
			normals = polygon.m_normals,
			i, s,
			vertIndex1, vertIndex2,
			v1, v2,
			u1, u2,
			faceCenterX, faceCenterY;

		manifold.m_pointCount = 0;

		tMat = xf2.R;
		tVec = circle.m_p;
		cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		dX = cX - xf1.position.x;
		dY = cY - xf1.position.y;

		tMat = xf1.R;
		cLocalX = dX * tMat.col1.x + dY * tMat.col1.y;
		cLocalY = dX * tMat.col2.x + dY * tMat.col2.y;

		for (i = 0; i < vertexCount; i++) {
			tVec = vertices[i];
			dX = cLocalX - tVec.x;
			dY = cLocalY - tVec.y;
			tVec = normals[i];
			s = tVec.x * dX + tVec.y * dY;
			if (s > radius) {
				return;
			}
			if (s > separation) {
				separation = s;
				normalIndex = i;
			}
		}

		vertIndex1 = normalIndex;
		vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		v1 = vertices[vertIndex1];
		v2 = vertices[vertIndex2];

		if (separation < Number.MIN_VALUE) {
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
			manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
			manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
			return;
		}

		u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
		u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);

		if (u1 <= 0) {
			if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) {
				return;
			}
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v1.x;
			manifold.m_localPlaneNormal.y = cLocalY - v1.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v1);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		} else if (u2 <= 0) {
			if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) {
				return;
			}
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = cLocalX - v2.x;
			manifold.m_localPlaneNormal.y = cLocalY - v2.y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.SetV(v2);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		} else {
			faceCenterX = 0.5 * (v1.x + v2.x);
			faceCenterY = 0.5 * (v1.y + v2.y);
			separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
			if (separation > radius) {
				return;
			}
			manifold.m_pointCount = 1;
			manifold.m_type = b2Manifold.e_faceA;
			manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
			manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
			manifold.m_localPlaneNormal.Normalize();
			manifold.m_localPoint.Set(faceCenterX, faceCenterY);
			manifold.m_points[0].m_localPoint.SetV(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
	},

	TestOverlap : function (a, b) {
		if (a.lowerBound.x > b.upperBound.x ||
			a.lowerBound.y > b.upperBound.y ||
			b.lowerBound.x > a.upperBound.x ||
			b.lowerBound.y > a.upperBound.y) {
			return false;
		}
		return true;
	}
};
