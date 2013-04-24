var b2TimeOfImpact = {
	b2_toiCalls : 0,
	b2_toiIters : 0,
	b2_toiMaxIters : 0,
	b2_toiRootIters : 0,
	b2_toiMaxRootIters : 0,

	TimeOfImpact : function (input) {
		b2TimeOfImpact.b2_toiCalls++;

		var proxyA = input.proxyA,
			proxyB = input.proxyB,
			sweepA = input.sweepA,
			sweepB = input.sweepB,
			radius = proxyA.m_radius + proxyB.m_radius,
			tolerance = input.tolerance,
			alpha = 0,
			k_maxIterations = 1000,
			iter = 0,
			target = 0,
			separation,
			newAlpha,
			x1, x2,
			f1, f2,
			rootIterCount,
			x, f;

		b2Settings.b2Assert(sweepA.t0 === sweepB.t0);
		b2Settings.b2Assert(1 - sweepA.t0 > Number.MIN_VALUE);

		b2TimeOfImpact.s_cache.count = 0;
		b2TimeOfImpact.s_distanceInput.useRadii = false;

		while (true) {
			sweepA.GetTransform(b2TimeOfImpact.s_xfA, alpha);
			sweepB.GetTransform(b2TimeOfImpact.s_xfB, alpha);

			b2TimeOfImpact.s_distanceInput.proxyA = proxyA;
			b2TimeOfImpact.s_distanceInput.proxyB = proxyB;
			b2TimeOfImpact.s_distanceInput.transformA = b2TimeOfImpact.s_xfA;
			b2TimeOfImpact.s_distanceInput.transformB = b2TimeOfImpact.s_xfB;

			b2Distance.Distance(b2TimeOfImpact.s_distanceOutput, b2TimeOfImpact.s_cache, b2TimeOfImpact.s_distanceInput);
			if (b2TimeOfImpact.s_distanceOutput.distance <= 0) {
				alpha = 1;
				break;
			}

			b2TimeOfImpact.s_fcn.Initialize(b2TimeOfImpact.s_cache, proxyA, b2TimeOfImpact.s_xfA, proxyB, b2TimeOfImpact.s_xfB);
			separation = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);

			if (separation <= 0) {
				alpha = 1;
				break;
			}

			if (iter === 0) {
				if (separation > radius) {
					target = Math.max(radius - tolerance, 0.75 * radius);
				}
				else {
					target = Math.max(separation - tolerance, 0.02 * radius);
				}
			}
			if (separation - target < 0.5 * tolerance) {
				if (iter === 0) {
					alpha = 1;
					break;
				}
				break;
			}
			newAlpha = x1 = alpha;
			x2 = 1;
			f1 = separation;

			sweepA.GetTransform(b2TimeOfImpact.s_xfA, x2);
			sweepB.GetTransform(b2TimeOfImpact.s_xfB, x2);
			f2 = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
			if (f2 >= target) {
				alpha = 1;
				break;
			}

			rootIterCount = 0;
			while (true) {
				x = 0;
				if (rootIterCount & 1) {
					x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
				} else {
					x = 0.5 * (x1 + x2);
				}
				sweepA.GetTransform(b2TimeOfImpact.s_xfA, x);
				sweepB.GetTransform(b2TimeOfImpact.s_xfB, x);
				f = b2TimeOfImpact.s_fcn.Evaluate(b2TimeOfImpact.s_xfA, b2TimeOfImpact.s_xfB);
				if (Math.abs(f - target) < 0.025 * tolerance) {
					newAlpha = x;
					break;
				}
				if (f > target) {
					x1 = x;
					f1 = f;
				} else {
					x2 = x;
					f2 = f;
				}
				rootIterCount++;
				b2TimeOfImpact.b2_toiRootIters++;
				if (rootIterCount === 50) {
					break;
				}
			}
			b2TimeOfImpact.b2_toiMaxRootIters = Math.max(b2TimeOfImpact.b2_toiMaxRootIters, rootIterCount);
			if (newAlpha < (1 + 100 * Number.MIN_VALUE) * alpha) {
				break;
			}
			alpha = newAlpha;
			iter++;
			b2TimeOfImpact.b2_toiIters++;
			if (iter === k_maxIterations) {
				break;
			}
		}
		b2TimeOfImpact.b2_toiMaxIters = Math.max(b2TimeOfImpact.b2_toiMaxIters, iter);
		return alpha;
	}
};

whenReady(function () {
	b2TimeOfImpact.s_cache = new b2SimplexCache();
	b2TimeOfImpact.s_distanceInput = new b2DistanceInput();
	b2TimeOfImpact.s_xfA = new b2Transform();
	b2TimeOfImpact.s_xfB = new b2Transform();
	b2TimeOfImpact.s_fcn = new b2SeparationFunction();
	b2TimeOfImpact.s_distanceOutput = new b2DistanceOutput();
});
