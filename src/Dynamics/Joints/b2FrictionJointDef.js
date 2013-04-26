function b2FrictionJointDef(def) {
	b2JointDef.apply(this, arguments);
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.type = b2Joint.e_frictionJoint;
	this.maxForce = 0;
	this.maxTorque = 0;
}

Box2D.b2FrictionJointDef = b2FrictionJointDef;

b2FrictionJointDef.prototype = {
	Initialize : function (bA, bB, anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
	}
};
