function b2DistanceJointDef(def) {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_distanceJoint;
	this.length = 1;
	this.frequencyHz = 0;
	this.dampingRatio = 0;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
}

b2DistanceJointDef.prototype = extend(new b2JointDef(), {
	Initialize : function (bA, bB, anchorA, anchorB) {
		var dX = anchorB.x - anchorA.x,
			dY = anchorB.y - anchorA.y;

		this.bodyA = bA;
		this.bodyB = bB;

		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));

		this.length = Math.sqrt(dX * dX + dY * dY);
	}
});
