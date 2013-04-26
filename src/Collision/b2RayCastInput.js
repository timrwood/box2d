function b2RayCastInput() {
	this.p1 = new b2Vec2();
	this.p2 = new b2Vec2();
}

Box2D.b2RayCastInput = b2RayCastInput;

b2RayCastInput.prototype = {
	b2RayCastInput : function (p1, p2, maxFraction) {
		if (p1) {
			this.p1.SetV(p1);
		}
		if (p2) {
			this.p2.SetV(p2);
		}
		this.maxFraction = maxFraction || 1;
	}
};
