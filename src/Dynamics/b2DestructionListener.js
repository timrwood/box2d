function b2DestructionListener() {

}

Box2D.b2DestructionListener = b2DestructionListener;

b2DestructionListener.prototype = {
	SayGoodbyeJoint : function (joint) {},
	SayGoodbyeFixture : function (fixture) {}
};
