function b2DebugDrawCanvas() {
	b2DebugDraw.apply(this, arguments);
}

Box2D.b2DebugDrawCanvas = b2DebugDrawCanvas;

inherit(b2DebugDraw, b2DebugDrawCanvas);

b2DebugDrawCanvas.prototype = {
	Clear : function () {
		this.ctx.save();
		this.ctx.setTransform(1, 0, 0, 1, 0, 0);
		this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
		this.ctx.restore();
	},

	Prepare : function () {
		this.ctx.lineWidth = 1;
	},

	Cleanup : function () {

	},

	SetCanvas : function (canvas) {
		this.ctx = canvas.getContext('2d');
		this.canvas = canvas;
	},

	_lineTo : function (x, y) {
		x = Math.round(x);
		y = Math.round(y);
		this.ctx.lineTo(x, y);
	},

	_moveTo : function (x, y) {
		x = Math.round(x);
		y = Math.round(y);
		this.ctx.moveTo(x, y);
	},

	DrawPolygon : function (vertices, vertexCount, color) {
		if (!vertexCount) {
			return;
		}
		var ctx = this.ctx,
			i,
			len = vertices.length,
			drawScale = this.m_drawScale;

		ctx.beginPath();
		this._moveTo(vertices[len - 1].x * drawScale, vertices[len - 1].y * drawScale);
		for (i = 0; i < len; i++) {
			this._lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}

		ctx.strokeStyle = color;
		ctx.stroke();
	},

	DrawSolidPolygon : function (vertices, vertexCount, color) {
		if (!vertexCount) {
			return;
		}
		var ctx = this.ctx,
			i,
			len = vertices.length,
			drawScale = this.m_drawScale;

		ctx.beginPath();
		this._moveTo(vertices[len - 1].x * drawScale, vertices[len - 1].y * drawScale);
		for (i = 0; i < len; i++) {
			this._lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}

		ctx.fillStyle = color;
		ctx.fill();
		ctx.strokeStyle = b2DebugDraw.c_outline;
		ctx.stroke();
	},

	DrawCircle : function (center, radius, color) {
		if (!radius) {
			return;
		}
		var s = this.ctx,
			drawScale = this.m_drawScale;

		s.beginPath();
		s.strokeStyle = color;
		s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
		s.closePath();
		s.stroke();
	},

	DrawSolidCircle : function (center, radius, axis, color) {
		var ctx = this.ctx,
			drawScale = this.m_drawScale,
			ax = axis.x * radius * drawScale,
			ay = axis.y * radius * drawScale,
			cx = center.x * drawScale,
			cy = center.y * drawScale;

		ctx.beginPath();
		ctx.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
		this._moveTo(cx, cy);
		this._lineTo(cx + ax, cy + ay);

		ctx.fillStyle = color;
		ctx.fill();
		ctx.strokeStyle = b2DebugDraw.c_outline;
		ctx.stroke();
	},

	DrawSegment : function (p1, p2, color) {
		var s = this.ctx,
			drawScale = this.m_drawScale;
		s.strokeStyle = color;
		s.beginPath();
		s.moveTo(p1.x * drawScale, p1.y * drawScale);
		s.lineTo(p2.x * drawScale, p2.y * drawScale);
		s.closePath();
		s.stroke();
	},

	DrawTransform : function (xf) {
		var ctx = this.ctx,
			drawScale = this.m_drawScale,
			px = xf.position.x * drawScale,
			py = xf.position.y * drawScale,
			sx = xf.R.col1.x * this.m_xformScale,
			sy = xf.R.col1.y * this.m_xformScale;

		ctx.beginPath();
		ctx.strokeStyle = b2DebugDraw.c_transformX;
		this._moveTo(px, py);
		this._lineTo(px + sx, py + sy);
		ctx.stroke();

		ctx.beginPath();
		ctx.strokeStyle = b2DebugDraw.c_transformY;
		this._moveTo(px, py);
		this._lineTo(px - sy, py + sx);
		ctx.stroke();
	}
};
