function b2DebugDrawCanvas() {
	b2DebugDraw.apply(this, arguments);
}

Box2D.b2DebugDrawCanvas = b2DebugDrawCanvas;

inherit(b2DebugDraw, b2DebugDrawCanvas);

b2DebugDrawCanvas.prototype = {
	Clear : function () {
		this.canvas.width = this.canvas.width;
		this.canvas.height = this.canvas.height;
	},

	SetCanvas : function (canvas) {
		this.ctx = canvas.getContext('2d');
		this.canvas = canvas;
	},

	DrawPolygon : function (vertices, vertexCount, color) {
		if (!vertexCount) {
			return;
		}
		var s = this.ctx,
			i,
			drawScale = this.m_drawScale;

		s.beginPath();
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);

		for (i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}

		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.stroke();
	},

	DrawSolidPolygon : function (vertices, vertexCount, color) {
		if (!vertexCount) {
			return;
		}
		var s = this.ctx,
			i,
			drawScale = this.m_drawScale;

		s.beginPath();
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);

		for (i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}

		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.stroke();
	},

	DrawCircle : function (center, radius, color) {
		if (!radius) {
			return;
		}
		var s = this.ctx,
			drawScale = this.m_drawScale;

		s.beginPath();
		s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
		s.closePath();
		s.stroke();
	},

	DrawSolidCircle : function (center, radius, axis, color) {
		var s = this.ctx,
			drawScale = this.m_drawScale,
			cx = center.x * drawScale,
			cy = center.y * drawScale;
		s.moveTo(0, 0);
		s.beginPath();
		// s.strokeStyle = this._color(color.color, this.m_alpha);
		// s.fillStyle = this._color(color.color, this.m_fillAlpha);
		s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
		s.moveTo(cx, cy);
		s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
		s.closePath();
		s.fill();
		s.stroke();
	}
};
