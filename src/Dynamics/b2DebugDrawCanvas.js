function b2DebugDrawCanvas() {
	b2DebugDraw.apply(this, arguments);
}

Box2D.b2DebugDrawCanvas = b2DebugDrawCanvas;

inherit(b2DebugDraw, b2DebugDrawCanvas);

b2DebugDrawCanvas.prototype = {
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

		console.log('draw poly');
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

		console.log('draw circle');
	}
};
