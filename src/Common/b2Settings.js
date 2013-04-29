var b2Settings = {
	VERSION : "2.1alpha",
	USHRT_MAX : 0x0000ffff,
	b2_pi : Math.PI,
	b2_linearSlop : 0.005,
	b2_timeToSleep : 0.5,
	b2_aabbExtension : 0.1,
	b2_aabbMultiplier : 2,
	b2_maxTranslation : 2,
	b2_contactBaumgarte : 0.2,
	b2_maxManifoldPoints : 2,
	b2_velocityThreshold : 1,
	b2_maxLinearCorrection : 0.2,
	b2_linearSleepTolerance : 0.01,
	b2_maxTOIJointsPerIsland : 32,
	b2_maxTOIContactsPerIsland : 32,

	b2MixFriction : function (friction1, friction2) {
		return Math.sqrt((friction1 || 0) * (friction2 || 0));
	},

	b2MixRestitution : function (restitution1, restitution2) {
		restitution1 = restitution1 || 0;
		restitution2 = restitution2 || 0;
		return restitution1 > restitution2 ? restitution1 : restitution2;
	},

	b2Assert : function (a) {
		if (!a) {
			/*global console:false */
			if (typeof console !== 'undefined') {
				console.error("Assertion Failed");
			} else {
				throw "Assertion Failed";
			}
		}
	}
};

b2Settings.b2_toiSlop = 8 * b2Settings.b2_linearSlop;
b2Settings.b2_angularSlop = 2 / 180 * b2Settings.b2_pi;
b2Settings.b2_maxRotation = b2Settings.b2_pi / 2;
b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;
b2Settings.b2_maxAngularCorrection = 8 / 180 * b2Settings.b2_pi;
b2Settings.b2_angularSleepTolerance = 2 / 180 * b2Settings.b2_pi;
b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;

Box2D.b2Settings = b2Settings;
