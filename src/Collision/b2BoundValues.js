function b2BoundValues() {

}

Box2D.b2BoundValues = b2BoundValues;

b2BoundValues.prototype = {
	b2BoundValues : function () {
		this.lowerValues = [0, 0];
		this.upperValues = [0, 0];
	}
};
