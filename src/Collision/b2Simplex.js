function b2Simplex() {
	this.m_v1 = new b2SimplexVertex();
	this.m_v2 = new b2SimplexVertex();
	this.m_v3 = new b2SimplexVertex();
	this.m_vertices = [
		this.m_v1,
		this.m_v2,
		this.m_v3
	];
}

Box2D.b2Simplex = b2Simplex;

b2Simplex.prototype = {
	ReadCache : function (cache, proxyA, transformA, proxyB, transformB) {
		var wALocal, wBLocal,
			vertices = this.m_vertices,
			i, v,
			metric1, metric2;

		b2Settings.b2Assert(cache.count >= 0 && cache.count <= 3);
		this.m_count = cache.count;

		for (i = 0; i < this.m_count; i++) {
			v = vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			wALocal = proxyA.GetVertex(v.indexA);
			wBLocal = proxyB.GetVertex(v.indexB);
			v.wA = b2Math.MulX(transformA, wALocal, v.wA);
			v.wB = b2Math.MulX(transformB, wBLocal, v.wB);
			v.w = b2Math.SubtractVV(v.wB, v.wA, v.w);
			v.a = 0;
		}
		if (this.m_count > 1) {
			metric1 = cache.metric;
			metric2 = this.GetMetric();
			if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < Number.MIN_VALUE) {
				this.m_count = 0;
			}
		}
		if (this.m_count === 0) {
			v = vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			wALocal = proxyA.GetVertex(0);
			wBLocal = proxyB.GetVertex(0);
			v.wA = b2Math.MulX(transformA, wALocal, v.wA);
			v.wB = b2Math.MulX(transformB, wBLocal, v.wB);
			v.w = b2Math.SubtractVV(v.wB, v.wA, v.w);
			this.m_count = 1;
		}
	},

	WriteCache : function (cache) {
		var vertices = this.m_vertices,
			i;

		cache.metric = this.GetMetric();
		cache.count = this.m_count;

		for (i = 0; i < this.m_count; i++) {
			cache.indexA[i] = vertices[i].indexA;
			cache.indexB[i] = vertices[i].indexB;
		}
	},

	GetSearchDirection : function (out) {
		out = out || new b2Vec2();
		var e12, sgn,
			tVecA = b2Simplex.t_vec2a,
			tVecB = b2Simplex.t_vec2b;
		switch (this.m_count) {
		case 1:
			return this.m_v1.w.GetNegative(out);
		case 2:
			e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w, tVecA);
			sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative(tVecB), tVecB);
			if (sgn > 0) {
				return b2Math.CrossFV(1, e12, out);
			}
			return b2Math.CrossVF(e12, 1, out);
		default:
			b2Settings.b2Assert(false);
			return new b2Vec2();
		}
	},

	GetClosestPoint : function (out) {
		out = out || new b2Vec2();
		switch (this.m_count) {
		case 1:
			return this.m_v1.w;
		case 2:
			out.x = this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x;
			out.y = this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y;
			break;
		default:
			b2Settings.b2Assert(false);
			break;
		}
		return out;
	},

	GetWitnessPoints : function (pA, pB) {
		switch (this.m_count) {
		case 0:
			b2Settings.b2Assert(false);
			break;
		case 1:
			pA.SetV(this.m_v1.wA);
			pB.SetV(this.m_v1.wB);
			break;
		case 2:
			pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
			pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
			pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
			pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
			break;
		case 3:
			pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
			pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
			break;
		default:
			b2Settings.b2Assert(false);
			break;
		}
	},

	GetMetric : function () {
		var tVec2a = b2Simplex.t_vec2a,
			tVec2b = b2Simplex.t_vec2b;
		switch (this.m_count) {
		case 1:
			return 0;
		case 2:
			return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w, tVec2a).Length();
		case 3:
			b2Math.SubtractVV(this.m_v2.w, this.m_v1.w, tVec2a);
			b2Math.SubtractVV(this.m_v3.w, this.m_v1.w, tVec2b);
			return b2Math.CrossVV(tVec2a, tVec2b, tVec2a);
		default:
			b2Settings.b2Assert(false);
			return 0;
		}
	},

	Solve2 : function () {
		var w1 = this.m_v1.w,
			w2 = this.m_v2.w,
			e12 = b2Math.SubtractVV(w2, w1, b2Simplex.t_vec2a),
			d12_2 = -(w1.x * e12.x + w1.y * e12.y),
			d12_1,
			inv_d12;

		if (d12_2 <= 0) {
			this.m_v1.a = 1;
			this.m_count = 1;
			return;
		}

		d12_1 = (w2.x * e12.x + w2.y * e12.y);
		if (d12_1 <= 0) {
			this.m_v2.a = 1;
			this.m_count = 1;
			this.m_v1.Set(this.m_v2);
			return;
		}
		inv_d12 = 1 / (d12_1 + d12_2);
		this.m_v1.a = d12_1 * inv_d12;
		this.m_v2.a = d12_2 * inv_d12;
		this.m_count = 2;
	},

	Solve3 : function () {
		var w1 = this.m_v1.w,
			w2 = this.m_v2.w,
			w3 = this.m_v3.w,
			tVecA = b2Simplex.t_vec2a,
			tVecB = b2Simplex.t_vec2b,
			tVecC = b2Simplex.t_vec2c,
			e12 = b2Math.SubtractVV(w2, w1, tVecA),
			w1e12 = b2Math.Dot(w1, e12),
			w2e12 = b2Math.Dot(w2, e12),
			d12_1 = w2e12,
			d12_2 = -w1e12,
			e13 = b2Math.SubtractVV(w3, w1, tVecB),
			w1e13 = b2Math.Dot(w1, e13),
			w3e13 = b2Math.Dot(w3, e13),
			d13_1 = w3e13,
			d13_2 = -w1e13,
			e23 = b2Math.SubtractVV(w3, w2, tVecC),
			w2e23 = b2Math.Dot(w2, e23),
			w3e23 = b2Math.Dot(w3, e23),
			d23_1 = w3e23,
			d23_2 = -w2e23,
			n123 = b2Math.CrossVV(e12, e13),
			d123_1 = n123 * b2Math.CrossVV(w2, w3),
			d123_2 = n123 * b2Math.CrossVV(w3, w1),
			d123_3 = n123 * b2Math.CrossVV(w1, w2),
			inv_d12, inv_d13, inv_d23, inv_d123;

		if (d12_2 <= 0 && d13_2 <= 0) {
			this.m_v1.a = 1;
			this.m_count = 1;
			return;
		}
		if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
			inv_d12 = 1 / (d12_1 + d12_2);
			this.m_v1.a = d12_1 * inv_d12;
			this.m_v2.a = d12_2 * inv_d12;
			this.m_count = 2;
			return;
		}
		if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
			inv_d13 = 1 / (d13_1 + d13_2);
			this.m_v1.a = d13_1 * inv_d13;
			this.m_v3.a = d13_2 * inv_d13;
			this.m_count = 2;
			this.m_v2.Set(this.m_v3);
			return;
		}
		if (d12_1 <= 0 && d23_2 <= 0) {
			this.m_v2.a = 1;
			this.m_count = 1;
			this.m_v1.Set(this.m_v2);
			return;
		}
		if (d13_1 <= 0 && d23_1 <= 0) {
			this.m_v3.a = 1;
			this.m_count = 1;
			this.m_v1.Set(this.m_v3);
			return;
		}
		if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
			inv_d23 = 1 / (d23_1 + d23_2);
			this.m_v2.a = d23_1 * inv_d23;
			this.m_v3.a = d23_2 * inv_d23;
			this.m_count = 2;
			this.m_v1.Set(this.m_v3);
			return;
		}
		inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
		this.m_v1.a = d123_1 * inv_d123;
		this.m_v2.a = d123_2 * inv_d123;
		this.m_v3.a = d123_3 * inv_d123;
		this.m_count = 3;
	}
};

whenReady(function () {
	b2Simplex.t_vec2a = new b2Vec2();
	b2Simplex.t_vec2b = new b2Vec2();
	b2Simplex.t_vec2c = new b2Vec2();
});
