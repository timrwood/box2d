function b2DebugDraw() {
	this.m_drawFlags = 63;
	this.m_xformScale = 10;
}

Box2D.b2DebugDraw = b2DebugDraw;

b2DebugDraw.e_aabbBit         = 1;
b2DebugDraw.e_pairBit         = 2;
b2DebugDraw.e_shapeBit        = 4;
b2DebugDraw.e_jointBit        = 8;
b2DebugDraw.e_controllerBit   = 16;
b2DebugDraw.e_centerOfMassBit = 32;

b2DebugDraw.c_inactive   = "rgba(0, 0, 0, 0.1)";
b2DebugDraw.c_static     = "rgba(0, 0, 0, 0.1)";
b2DebugDraw.c_kinematic  = "rgba(0, 0, 0, 0.1)";
b2DebugDraw.c_asleep     = "rgba(0, 0, 0, 0.1)";
b2DebugDraw.c_active     = "rgba(0, 0, 0, 0.1)";
b2DebugDraw.c_pair       = "#999";
b2DebugDraw.c_aabb       = "#999";
b2DebugDraw.c_joint      = "#999";
b2DebugDraw.c_transformX = "#f00";
b2DebugDraw.c_transformY = "#00f";

b2DebugDraw.prototype = {
	Clear : function () {},
	Prepare : function () {},
	Cleanup : function () {},

	SetFlags : function (flags) {
		this.m_drawFlags = flags || 0;
	},

	GetFlags : function () {
		return this.m_drawFlags;
	},

	AppendFlags : function (flags) {
		this.m_drawFlags |= flags || 0;
	},

	ClearFlags : function (flags) {
		this.m_drawFlags = 0;
	},

	SetDrawScale : function (drawScale) {
		this.m_drawScale = drawScale || 0;
	},

	GetDrawScale : function () {
		return this.m_drawScale;
	},

	SetLineThickness : function (lineThickness) {},
	GetLineThickness : function () {},

	SetAlpha : function (alpha) {},
	GetAlpha : function () {},
	SetFillAlpha : function (alpha) {},
	GetFillAlpha : function () {},

	SetXFormScale : function (xformScale) {},
	GetXFormScale : function () {},

	DrawPolygon : function (vertices, vertexCount, color) {},
	DrawSolidPolygon : function (vertices, vertexCount, color) {},

	DrawCircle : function (center, radius, color) {},
	DrawSolidCircle : function (center, radius, axis, color) {},

	DrawSegment : function (p1, p2, color) {},

	DrawTransform : function (xf) {}
};
