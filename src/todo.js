
/* Joints */


	function b2PulleyJointDef() {
		b2PulleyJointDef.b2PulleyJointDef.apply(this, arguments);
		if (this.constructor === b2PulleyJointDef) this.b2PulleyJointDef.apply(this, arguments);
	};
	Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef;
})(); //definitions
Box2D.postDefs = [];
(function () {
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();
		Box2D.Collision.b2Collision.s_localTangent = new b2Vec2();
		Box2D.Collision.b2Collision.s_localNormal = new b2Vec2();
		Box2D.Collision.b2Collision.s_planePoint = new b2Vec2();
		Box2D.Collision.b2Collision.s_normal = new b2Vec2();
		Box2D.Collision.b2Collision.s_tangent = new b2Vec2();
		Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2();
		Box2D.Collision.b2Collision.s_v11 = new b2Vec2();
		Box2D.Collision.b2Collision.s_v12 = new b2Vec2();
		Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2();
		Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff;
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2Distance.s_simplex = new b2Simplex();
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.b2TimeOfImpact.s_cache = new b2SimplexCache();
		Box2D.Collision.b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
		Box2D.Collision.b2TimeOfImpact.s_xfA = new b2Transform();
		Box2D.Collision.b2TimeOfImpact.s_xfB = new b2Transform();
		Box2D.Collision.b2TimeOfImpact.s_fcn = new b2SeparationFunction();
		Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
	});
	Box2D.postDefs.push(function () {
		Box2D.Collision.Shapes.b2PolygonShape.s_mat = new b2Mat22();
	});
	Box2D.postDefs.push(function () {
		Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);
		Box2D.Common.Math.b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
		Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2Body.s_xf1 = new b2Transform();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2ContactManager.s_evalCP = new b2ContactPoint();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2Island.s_impulse = new b2ContactImpulse();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.b2World.s_timestep2 = new b2TimeStep();
		Box2D.Dynamics.b2World.s_xf = new b2Transform();
		Box2D.Dynamics.b2World.s_backupA = new b2Sweep();
		Box2D.Dynamics.b2World.s_backupB = new b2Sweep();
		Box2D.Dynamics.b2World.s_timestep = new b2TimeStep();
		Box2D.Dynamics.b2World.s_queue = new Vector();
		Box2D.Dynamics.b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2Contact.s_input = new b2TOIInput();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold = new b2WorldManifold();
		Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new b2PositionSolverManifold();
	});
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA = new b2Vec2();
		Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB = new b2Vec2();
	});
})();
(function () {


	Box2D.inherit(b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef);
	b2PulleyJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
	b2PulleyJointDef.b2PulleyJointDef = function () {
		Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
		this.groundAnchorA = new b2Vec2();
		this.groundAnchorB = new b2Vec2();
		this.localAnchorA = new b2Vec2();
		this.localAnchorB = new b2Vec2();
	};
	b2PulleyJointDef.prototype.b2PulleyJointDef = function () {
		this.__super.b2JointDef.call(this);
		this.type = b2Joint.e_pulleyJoint;
		this.groundAnchorA.Set((-1.0), 1.0);
		this.groundAnchorB.Set(1.0, 1.0);
		this.localAnchorA.Set((-1.0), 0.0);
		this.localAnchorB.Set(1.0, 0.0);
		this.lengthA = 0.0;
		this.maxLengthA = 0.0;
		this.lengthB = 0.0;
		this.maxLengthB = 0.0;
		this.ratio = 1.0;
		this.collideConnected = true;
	}
	b2PulleyJointDef.prototype.Initialize = function (bA, bB, gaA, gaB, anchorA, anchorB, r) {
		if (r === undefined) r = 0;
		this.bodyA = bA;
		this.bodyB = bB;
		this.groundAnchorA.SetV(gaA);
		this.groundAnchorB.SetV(gaB);
		this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
		var d1X = anchorA.x - gaA.x;
		var d1Y = anchorA.y - gaA.y;
		this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
		var d2X = anchorB.x - gaB.x;
		var d2Y = anchorB.y - gaB.y;
		this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
		this.ratio = r;
		var C = this.lengthA + this.ratio * this.lengthB;
		this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}
	Box2D.postDefs.push(function () {
		Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2();
	});
})();
(function () {
	var b2DebugDraw = Box2D.Dynamics.b2DebugDraw;
	b2DebugDraw.b2DebugDraw = function () {
		this.m_drawScale = 1.0;
		this.m_lineThickness = 1.0;
		this.m_alpha = 1.0;
		this.m_fillAlpha = 1.0;
		this.m_xformScale = 1.0;
		var __this = this;
		//#WORKAROUND
		this.m_sprite = {
			graphics: {
				clear: function () {
					__this.m_ctx.clearRect(0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height)
				}
			}
		};
	};
	b2DebugDraw.prototype._color = function (color, alpha) {
		return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
	};
	b2DebugDraw.prototype.b2DebugDraw = function () {
		this.m_drawFlags = 0;
	};
	b2DebugDraw.prototype.SetFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags = flags;
	};
	b2DebugDraw.prototype.GetFlags = function () {
		return this.m_drawFlags;
	};
	b2DebugDraw.prototype.AppendFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags |= flags;
	};
	b2DebugDraw.prototype.ClearFlags = function (flags) {
		if (flags === undefined) flags = 0;
		this.m_drawFlags &= ~flags;
	};
	b2DebugDraw.prototype.SetSprite = function (sprite) {
		this.m_ctx = sprite;
	};
	b2DebugDraw.prototype.GetSprite = function () {
		return this.m_ctx;
	};
	b2DebugDraw.prototype.SetDrawScale = function (drawScale) {
		if (drawScale === undefined) drawScale = 0;
		this.m_drawScale = drawScale;
	};
	b2DebugDraw.prototype.GetDrawScale = function () {
		return this.m_drawScale;
	};
	b2DebugDraw.prototype.SetLineThickness = function (lineThickness) {
		if (lineThickness === undefined) lineThickness = 0;
		this.m_lineThickness = lineThickness;
		this.m_ctx.strokeWidth = lineThickness;
	};
	b2DebugDraw.prototype.GetLineThickness = function () {
		return this.m_lineThickness;
	};
	b2DebugDraw.prototype.SetAlpha = function (alpha) {
		if (alpha === undefined) alpha = 0;
		this.m_alpha = alpha;
	};
	b2DebugDraw.prototype.GetAlpha = function () {
		return this.m_alpha;
	};
	b2DebugDraw.prototype.SetFillAlpha = function (alpha) {
		if (alpha === undefined) alpha = 0;
		this.m_fillAlpha = alpha;
	};
	b2DebugDraw.prototype.GetFillAlpha = function () {
		return this.m_fillAlpha;
	};
	b2DebugDraw.prototype.SetXFormScale = function (xformScale) {
		if (xformScale === undefined) xformScale = 0;
		this.m_xformScale = xformScale;
	};
	b2DebugDraw.prototype.GetXFormScale = function () {
		return this.m_xformScale;
	};
	b2DebugDraw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
		if (!vertexCount) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		for (var i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}
		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
		if (!vertexCount) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.fillStyle = this._color(color.color, this.m_fillAlpha);
		s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		for (var i = 1; i < vertexCount; i++) {
			s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
		}
		s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
		s.closePath();
		s.fill();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawCircle = function (center, radius, color) {
		if (!radius) return;
		var s = this.m_ctx;
		var drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
		if (!radius) return;
		var s = this.m_ctx,
			drawScale = this.m_drawScale,
			cx = center.x * drawScale,
			cy = center.y * drawScale;
		s.moveTo(0, 0);
		s.beginPath();
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.fillStyle = this._color(color.color, this.m_fillAlpha);
		s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
		s.moveTo(cx, cy);
		s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
		s.closePath();
		s.fill();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawSegment = function (p1, p2, color) {
		var s = this.m_ctx,
			drawScale = this.m_drawScale;
		s.strokeStyle = this._color(color.color, this.m_alpha);
		s.beginPath();
		s.moveTo(p1.x * drawScale, p1.y * drawScale);
		s.lineTo(p2.x * drawScale, p2.y * drawScale);
		s.closePath();
		s.stroke();
	};
	b2DebugDraw.prototype.DrawTransform = function (xf) {
		var s = this.m_ctx,
			drawScale = this.m_drawScale;
		s.beginPath();
		s.strokeStyle = this._color(0xff0000, this.m_alpha);
		s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
		s.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale);

		s.strokeStyle = this._color(0xff00, this.m_alpha);
		s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
		s.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale);
		s.closePath();
		s.stroke();
	};
})(); //post-definitions
var i;
for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;





var Box2D = {};

(function (a2j, undefined) {

	if(!(Object.prototype.defineProperty instanceof Function)
		&& Object.prototype.__defineGetter__ instanceof Function
		&& Object.prototype.__defineSetter__ instanceof Function)
	{
		Object.defineProperty = function(obj, p, cfg) {
			if(cfg.get instanceof Function)
				obj.__defineGetter__(p, cfg.get);
			if(cfg.set instanceof Function)
				obj.__defineSetter__(p, cfg.set);
		}
	}

	function emptyFn() {};
	a2j.inherit = function(cls, base) {
		var tmpCtr = cls;
		emptyFn.prototype = base.prototype;
		cls.prototype = new emptyFn;
		cls.prototype.constructor = tmpCtr;
	};

	a2j.generateCallback = function generateCallback(context, cb) {
		return function () {
			cb.apply(context, arguments);
		};
	};

	a2j.NVector = function NVector(length) {
		if (length === undefined) length = 0;
		var tmp = new Array(length || 0);
		for (var i = 0; i < length; ++i)
		tmp[i] = 0;
		return tmp;
	};

	a2j.is = function is(o1, o2) {
		if (o1 === null) return false;
		if ((o2 instanceof Function) && (o1 instanceof o2)) return true;
		if ((o1.constructor.__implements != undefined) && (o1.constructor.__implements[o2])) return true;
		return false;
	};

	a2j.parseUInt = function(v) {
		return Math.abs(parseInt(v));
	}

})(Box2D);

//#TODO remove assignments from global namespace
var Vector = Array;
var Vector_a2j_Number = Box2D.NVector;
//package structure
if (typeof(Box2D) === "undefined") Box2D = {};
if (typeof(Box2D.Collision) === "undefined") Box2D.Collision = {};
if (typeof(Box2D.Collision.Shapes) === "undefined") Box2D.Collision.Shapes = {};
if (typeof(Box2D.Common) === "undefined") Box2D.Common = {};
if (typeof(Box2D.Common.Math) === "undefined") Box2D.Common.Math = {};
if (typeof(Box2D.Dynamics) === "undefined") Box2D.Dynamics = {};
if (typeof(Box2D.Dynamics.Contacts) === "undefined") Box2D.Dynamics.Contacts = {};
if (typeof(Box2D.Dynamics.Controllers) === "undefined") Box2D.Dynamics.Controllers = {};
if (typeof(Box2D.Dynamics.Joints) === "undefined") Box2D.Dynamics.Joints = {};
//pre-definitions
(function () {
	Box2D.Collision.IBroadPhase = 'Box2D.Collision.IBroadPhase';
	Box2D.Common.b2internal = 'Box2D.Common.b2internal';


