function b2ContactListener() {

}

Box2D.b2ContactListener = b2ContactListener;

b2ContactListener.prototype = {
	BeginContact : function (contact) {},
	EndContact : function (contact) {},
	PreSolve : function (contact, oldManifold) {},
	PostSolve : function (contact, impulse) {}
};

b2ContactListener.b2_defaultListener = new b2ContactListener();
