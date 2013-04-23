function b2GearJointDef() {
	b2JointDef.apply(this, arguments);
	this.type = b2Joint.e_gearJoint;
	this.joint1 = null;
	this.joint2 = null;
	this.ratio = 1;
}

b2GearJointDef.prototype = new b2JointDef();
