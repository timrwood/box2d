function b2Shape() {
	this.m_type = b2Shape.e_unknownShape;
	this.m_radius = b2Settings.b2_linearSlop;
}

Box2D.b2Shape = b2Shape;

b2Shape.e_unknownShape = -1;
b2Shape.e_circleShape = 0;
b2Shape.e_polygonShape = 1;
b2Shape.e_edgeShape = 2;
b2Shape.e_shapeTypeCount = 3;
b2Shape.e_hitCollide = 1;
b2Shape.e_missCollide = 0;
b2Shape.e_startsInsideCollide = -1;

// TODO: Cache b2DistanceInput, b2SimplexCache, b2DistanceOutput?

b2Shape.TestOverlap = function (shape1, transform1, shape2, transform2) {
	var input = new b2DistanceInput(),
		simplexCache = new b2SimplexCache(),
		output = new b2DistanceOutput();

	input.proxyA = new b2DistanceProxy();
	input.proxyA.Set(shape1);
	input.proxyB = new b2DistanceProxy();
	input.proxyB.Set(shape2);
	input.transformA = transform1;
	input.transformB = transform2;
	input.useRadii = true;

	simplexCache.count = 0;

	b2Distance.Distance(output, simplexCache, input);

	return output.distance < 10 * Number.MIN_VALUE;
};

b2Shape.prototype = {
	Copy : function () {
		return null;
	},

	Set : function (other) {
		this.m_radius = other.m_radius;
	},

	GetType : function () {
		return this.m_type;
	},

	TestPoint : function (xf, p) {
		return false;
	},

	RayCast : function (output, input, transform) {
		return false;
	},

	ComputeAABB : function (aabb, xf) {},

	ComputeMass : function (massData, density) {},

	ComputeSubmergedArea : function (normal, offset, xf, c) {
		return 0;
	}
};
