function b2ContactConstraint() {
	var i;

	this.localPlaneNormal = new b2Vec2();
	this.localPoint = new b2Vec2();

	this.normal = new b2Vec2();
	this.normalMass = new b2Mat22();

	this.K = new b2Mat22();

	this.points = [];
	for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
		this.points[i] = new b2ContactConstraintPoint();
	}
}

Box2D.b2ContactConstraint = b2ContactConstraint;
