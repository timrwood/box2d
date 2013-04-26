function b2DebugDrawCanvas() {
	b2DebugDraw.apply(this, arguments);
}

Box2D.b2DebugDrawCanvas = b2DebugDrawCanvas;

inherit(b2DebugDraw, b2DebugDrawCanvas);

b2DebugDrawCanvas.prototype = {
};
