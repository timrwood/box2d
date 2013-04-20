function b2Manifold() {
	var i;
	this.m_pointCount = 0;
	this.m_points = [];
	for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
		this.m_points[i] = new b2ManifoldPoint();
	}
	this.m_localPlaneNormal = new b2Vec2();
	this.m_localPoint = new b2Vec2();
}

b2Manifold.prototype = {
	Reset : function () {
		var i;
		for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.m_points[i].Reset();
		}
		this.m_localPlaneNormal.SetZero();
		this.m_localPoint.SetZero();
		this.m_type = 0;
		this.m_pointCount = 0;
	},

	Set : function (m) {
		var i;
		this.m_pointCount = m.m_pointCount;
		for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
			this.m_points[i].Set(m.m_points[i]);
		}
		this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
		this.m_localPoint.SetV(m.m_localPoint);
		this.m_type = m.m_type;
	},

	Copy : function () {
		var copy = new b2Manifold();
		copy.Set(this);
		return copy;
	}
};

b2Manifold.e_circles = 0x0001;
b2Manifold.e_faceA = 0x0002;
b2Manifold.e_faceB = 0x0004;
