function b2FixtureDef() {
	this.filter = new b2FilterData();
}

b2FixtureDef.prototype = {
	shape : null,
	userData : null,
	friction : 0.2,
	restitution : 0,
	density : 0,
	isSensor : false
};