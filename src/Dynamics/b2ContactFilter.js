function b2ContactFilter() {

}

b2ContactFilter.prototype = {
	ShouldCollide : function (fixtureA, fixtureB) {
		var filter1 = fixtureA.GetFilterData(),
			filter2 = fixtureB.GetFilterData();

		if (filter1.groupIndex === filter2.groupIndex && filter1.groupIndex !== 0) {
			return filter1.groupIndex > 0;
		}

		return (filter1.maskBits & filter2.categoryBits) !== 0 && (filter1.categoryBits & filter2.maskBits) !== 0;
	},

	RayCollide : function (userData, fixture) {
		if (!userData || !userData instanceof b2Fixture) {
			return true;
		}
		return this.ShouldCollide(userData, fixture);
	}
};

b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
