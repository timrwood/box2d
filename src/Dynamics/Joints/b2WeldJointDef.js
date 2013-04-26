function b2WeldJointDef(def) {
	b2JointDef.apply(this, arguments);
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.type = b2Joint.e_weldJoint;
	this.referenceAngle = 0;
}

Box2D.b2WeldJointDef = b2WeldJointDef;

b2WeldJointDef.prototype = {
	Initialize : function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(bA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(bB.GetLocalPoint(anchor));
		this.referenceAngle = bB.GetAngle() - bA.GetAngle();
	}
};
