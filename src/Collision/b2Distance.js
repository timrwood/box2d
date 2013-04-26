var b2Distance = {
	s_saveA : [],
	s_saveB : [],

	Distance : function (output, cache, input) {
		var proxyA = input.proxyA,
			proxyB = input.proxyB,
			transformA = input.transformA,
			transformB = input.transformB,
			simplex = b2Distance.s_simplex,
			vertices,
			closestPoint,
			k_maxIters = 20,
			saveCount,
			saveA = b2Distance.s_saveA,
			saveB = b2Distance.s_saveB,
			distanceSqr1, distanceSqr2,
			i = 0,
			p, d,
			iter = 0,
			vertex,
			duplicate,
			rA, rB,
			normal;

		b2Distance.b2_gjkCalls++; // TODO

		simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
		vertices = simplex.m_vertices;
		closestPoint = simplex.GetClosestPoint();

		distanceSqr1 = distanceSqr2 = closestPoint.LengthSquared();

		while (iter < k_maxIters) {
			saveCount = simplex.m_count;

			for (i = 0; i < saveCount; i++) {
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count) {
			case 1:
				break;
			case 2:
				simplex.Solve2();
				break;
			case 3:
				simplex.Solve3();
				break;
			default:
				b2Settings.b2Assert(false);
			}

			if (simplex.m_count === 3) {
				break;
			}

			p = simplex.GetClosestPoint();
			distanceSqr1 = distanceSqr2 = p.LengthSquared();
			d = simplex.GetSearchDirection();
			if (d.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) {
				break;
			}

			vertex = vertices[simplex.m_count];
			vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
			vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
			vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
			vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
			vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);

			iter++;
			b2Distance.b2_gjkIters++; // TODO

			duplicate = false;
			for (i = 0; i < saveCount; i++) {
				if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
					duplicate = true;
					break;
				}
			}
			if (duplicate) {
				break;
			}
			simplex.m_count++;
		}

		b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter); // TODO

		simplex.GetWitnessPoints(output.pointA, output.pointB);
		output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
		output.iterations = iter;
		simplex.WriteCache(cache);

		if (input.useRadii) {
			rA = proxyA.m_radius;
			rB = proxyB.m_radius;

			if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
				output.distance -= rA + rB;
				normal = b2Math.SubtractVV(output.pointB, output.pointA);
				normal.Normalize();
				output.pointA.x += rA * normal.x;
				output.pointA.y += rA * normal.y;
				output.pointB.x -= rB * normal.x;
				output.pointB.y -= rB * normal.y;
			} else {
				p = new b2Vec2();
				p.x = 0.5 * (output.pointA.x + output.pointB.x);
				p.y = 0.5 * (output.pointA.y + output.pointB.y);
				output.pointA.x = output.pointB.x = p.x;
				output.pointA.y = output.pointB.y = p.y;
				output.distance = 0;
			}
		}
	}
};


function b2DistanceInput() {}

function b2DistanceOutput() {
	this.pointA = new b2Vec2();
	this.pointB = new b2Vec2();
}


Box2D.b2DistanceOutput = b2DistanceOutput;
Box2D.b2DistanceInput = b2DistanceInput;
Box2D.b2Distance = b2Distance;

whenReady(function () {
	b2Distance.s_simplex = new b2Simplex();
});
