function b2MouseJointDef() {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_mouseJoint;

	this.maxForce = 0;
	this.frequencyHz = 5;
	this.dampingRatio = 0.7;

	this.target = new b2Vec2();
}

Box2D.b2MouseJointDef = b2MouseJointDef;
