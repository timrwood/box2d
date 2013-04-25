
function b2DistanceProxy() {

}

b2DistanceProxy.prototype = {
	Set : function (shape) {
		switch (shape.GetType()) {
		case b2Shape.e_circleShape :
			this.m_vertices = [shape.m_p];
			this.m_count = 1;
			this.m_radius = shape.m_radius;
			break;
		case b2Shape.e_polygonShape:
			this.m_vertices = shape.m_vertices;
			this.m_count = shape.m_vertexCount;
			this.m_radius = shape.m_radius;
			break;
		default:
			b2Settings.b2Assert(false);
		}
	},

	GetSupport : function (d) {
		var bestIndex = 0,
			bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y,
			i, value;
		for (i = 1; i < this.m_count; i++) {
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

	GetVertexCount : function () {
		return this.m_count;
	},

	GetVertex : function (index) {
		index = index || 0;
		b2Settings.b2Assert(index >= 0 && index < this.m_count);
		return this.m_vertices[index];
	}
};
