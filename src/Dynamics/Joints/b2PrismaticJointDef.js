function b2PrismaticJointDef() {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_prismaticJoint;
	this.referenceAngle = 0;

	this.enableLimit = false;
	this.lowerTranslation = 0;
	this.upperTranslation = 0;

	this.enableMotor = false;
	this.maxMotorForce = 0;
	this.motorSpeed = 0;

	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.localAxisA = new b2Vec2(1, 0);
}

b2PrismaticJointDef.prototype = extend(new b2JointDef(), {
	Initialize : function (bA, bB, anchor, axis) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = bA.GetLocalPoint(anchor);
		this.localAnchorB = bB.GetLocalPoint(anchor);
		this.localAxisA = bA.GetLocalVector(axis);
		this.referenceAngle = bB.GetAngle() - bA.GetAngle();
	}
});
