function b2DebugDraw() {

}

b2DebugDraw.e_aabbBit         = 1;
b2DebugDraw.e_pairBit         = 2;
b2DebugDraw.e_shapeBit        = 4;
b2DebugDraw.e_jointBit        = 8;
b2DebugDraw.e_controllerBit   = 16;
b2DebugDraw.e_centerOfMassBit = 32;

b2DebugDraw.prototype = {
	SetFlags : function (flags) {},
	GetFlags : function () {},
	AppendFlags : function (flags) {},
	ClearFlags : function (flags) {},

	SetSprite : function (sprite) {},
	GetSprite : function () {},

	SetDrawScale : function (drawScale) {},
	GetDrawScale : function () {},

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