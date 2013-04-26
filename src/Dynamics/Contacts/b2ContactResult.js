function b2ContactResult() {
	this.position = new b2Vec2();
	this.normal = new b2Vec2();
	this.id = new b2ContactID();
}

Box2D.b2ContactResult = b2ContactResult;
