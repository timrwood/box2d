function b2WeldJointDef(def) {
	b2JointDef.apply(this, arguments);
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.type = b2Joint.e_weldJoint;
	this.referenceAngle = 0;
}

b2WeldJointDef.prototype = extend(new b2JointDef(), {
	Initialize : function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(bA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(bB.GetLocalPoint(anchor));
		this.referenceAngle = bB.GetAngle() - bA.GetAngle();
	}
});
