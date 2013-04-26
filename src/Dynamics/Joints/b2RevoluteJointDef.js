function b2RevoluteJointDef() {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_revoluteJoint;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.referenceAngle = 0;
	this.lowerAngle = 0;
	this.upperAngle = 0;
	this.maxMotorTorque = 0;
	this.motorSpeed = 0;
	this.enableLimit = false;
	this.enableMotor = false;
}

Box2D.b2RevoluteJointDef = b2RevoluteJointDef;

b2RevoluteJointDef.prototype = {
	Initialize : function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(bA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(bB.GetLocalPoint(anchor));
		this.referenceAngle = bB.GetAngle() - bA.GetAngle();
	}
};
