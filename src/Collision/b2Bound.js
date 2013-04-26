function b2Bound() {

}

Box2D.b2Bound = b2Bound;

b2Bound.prototype = {
	IsLower : function () {
		return (this.value & 1) === 0;
	},

	IsUpper : function () {
		return (this.value & 1) === 1;
	},

	Swap : function (b) {
		var tempValue = this.value,
			tempProxy = this.proxy,
			tempStabbingCount = this.stabbingCount;

		this.value = b.value;
		this.proxy = b.proxy;
		this.stabbingCount = b.stabbingCount;

		b.value = tempValue;
		b.proxy = tempProxy;
		b.stabbingCount = tempStabbingCount;
	}
};
