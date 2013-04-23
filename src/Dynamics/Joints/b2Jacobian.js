function b2Jacobian() {
	this.linearA = new b2Vec2();
	this.linearB = new b2Vec2();
}

b2GearJointDef.prototype = {
	SetZero : function () {
		this.linearA.SetZero();
		this.linearB.SetZero();
		this.angularA = 0;
		this.angularB = 0;
	},

	Set : function (x1, a1, x2, a2) {
		a1 = a1 || 0;
		a2 = a2 || 0;
		this.linearA.SetV(x1);
		this.angularA = a1;
		this.linearB.SetV(x2);
		this.angularB = a2;
	},

	Compute : function (x1, a1, x2, a2) {
		a1 = a1 || 0;
		a2 = a2 || 0;
		return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
	}
};
