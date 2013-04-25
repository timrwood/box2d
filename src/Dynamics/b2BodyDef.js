function b2BodyDef() {
	this.position = new b2Vec2();
	this.linearVelocity = new b2Vec2();
}

b2BodyDef.prototype = {
	userData        : null,
	angle           : 0,
	angularVelocity : 0,
	linearDamping   : 0,
	angularDamping  : 0,
	allowSleep      : true,
	awake           : true,
	fixedRotation   : false,
	bullet          : false,
	type            : b2Body.b2_staticBody,
	active          : true,
	inertiaScale    : 1
};
