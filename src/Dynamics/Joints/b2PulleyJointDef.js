function b2PulleyJointDef() {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_pulleyJoint;

	this.groundAnchorA = new b2Vec2(-1, 1);
	this.groundAnchorB = new b2Vec2(1, 1);

	this.localAnchorA = new b2Vec2(-1, 0);
	this.localAnchorB = new b2Vec2(1, 0);

	this.lengthA = 0;
	this.maxLengthA = 0;
	this.lengthB = 0;
	this.maxLengthB = 0;

	this.ratio = 1;
	this.collideConnected = true;
}

Box2D.b2PulleyJointDef = b2PulleyJointDef;

b2PulleyJointDef.prototype = {
	Initialize : function (bA, bB, gaA, gaB, anchorA, anchorB, r) {
		var d1X, d1Y,
			d2X, d2Y,
			C;

		this.bodyA = bA;
		this.bodyB = bB;

		this.groundAnchorA.SetV(gaA);
		this.groundAnchorB.SetV(gaB);

		this.localAnchorA = bA.GetLocalPoint(anchorA);
		this.localAnchorB = bB.GetLocalPoint(anchorB);

		d1X = anchorA.x - gaA.x;
		d1Y = anchorA.y - gaA.y;

		this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);

		d2X = anchorB.x - gaB.x;
		d2Y = anchorB.y - gaB.y;

		this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
		this.ratio = r || 0;

		C = this.lengthA + this.ratio * this.lengthB;

		this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
		this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
	}
};
