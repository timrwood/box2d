function b2ContactImpulse() {
	var i;
	this.normalImpulses = [];
	this.tangentImpulses = [];

	for (i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
		this.normalImpulses[i] = this.tangentImpulses[i] = 0;
	}
}