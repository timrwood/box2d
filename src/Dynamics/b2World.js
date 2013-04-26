function b2World(gravity, doSleep) {
	this.s_stack = [];

	this.m_contactManager = new b2ContactManager();
	this.m_contactManager.m_world = this;

	this.m_contactSolver = new b2ContactSolver();
	this.m_island = new b2Island();

	this.m_destructionListener = null;

	this.m_debugDraw = null;

	this.m_bodyList = null;
	this.m_contactList = null;
	this.m_jointList = null;
	this.m_controllerList = null;

	this.m_bodyCount = 0;
	this.m_contactCount = 0;
	this.m_jointCount = 0;
	this.m_controllerCount = 0;

	this.m_allowSleep = doSleep;
	this.m_gravity = gravity || new b2Vec2(0, 9.8);

	this.m_inv_dt0 = 0;

	this.m_groundBody = this.CreateBody(new b2BodyDef());
}

Box2D.b2World = b2World;

b2World.m_warmStarting = true;
b2World.m_continuousPhysics = true;

b2World.e_locked     = 1;
b2World.e_newFixture = 2;

b2World.prototype = {
	SetDestructionListener : function (listener) {
		this.m_destructionListener = listener;
	},

	SetContactFilter : function (filter) {
		this.m_contactManager.m_contactFilter = filter;
	},

	SetContactListener : function (listener) {
		this.m_contactManager.m_contactListener = listener;
	},

	SetDebugDraw : function (debugDraw) {
		this.m_debugDraw = debugDraw;
	},

	SetBroadPhase : function (broadPhase) {
		var oldBroadPhase = this.m_contactManager.m_broadPhase,
			b = this.m_bodyList,
			f;

		this.m_contactManager.m_broadPhase = broadPhase;

		while (b) {
			f = b.m_fixtureList;
			while (f) {
				f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
				f = f.m_next;
			}
			b = b.m_next;
		}
	},

	Validate : function () {
		this.m_contactManager.m_broadPhase.Validate();
	},

	GetProxyCount : function () {
		return this.m_contactManager.m_broadPhase.GetProxyCount();
	},

	CreateBody : function (def) {
		var b;

		if (this.IsLocked()) {
			return null; // TODO: Allow to create bodies while locked by adding them to a queue
		}

		b = new b2Body(def, this);
		b.m_prev = null;
		b.m_next = this.m_bodyList;

		if (this.m_bodyList) {
			this.m_bodyList.m_prev = b;
		}

		this.m_bodyList = b;
		this.m_bodyCount++;

		return b;
	},

	DestroyBody : function (b) {
		var jn = b.m_jointList, jn0,
			coe = b.m_controllerList, coe0,
			ce = b.m_contactList, ce0,
			f = b.m_fixtureList, f0;

		if (this.IsLocked()) {
			return; // TODO: Allow to remove bodies while locked by adding them to a queue
		}

		while (jn) {
			jn0 = jn;
			jn = jn.next;
			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
			}
			this.DestroyJoint(jn0.joint);
		}

		while (coe) {
			coe0 = coe;
			coe = coe.nextController;
			coe0.controller.RemoveBody(b);
		}

		while (ce) {
			ce0 = ce;
			ce = ce.next;
			this.m_contactManager.Destroy(ce0.contact);
		}
		b.m_contactList = null;

		while (f) {
			f0 = f;
			f = f.m_next;
			if (this.m_destructionListener) {
				this.m_destructionListener.SayGoodbyeFixture(f0);
			}
			f0.DestroyProxy(this.m_contactManager.m_broadPhase);
			f0.Destroy();
		}

		b.m_fixtureList = null;
		b.m_fixtureCount = 0;

		if (b.m_prev) {
			b.m_prev.m_next = b.m_next;
		}

		if (b.m_next) {
			b.m_next.m_prev = b.m_prev;
		}

		if (b === this.m_bodyList) {
			this.m_bodyList = b.m_next;
		}

		this.m_bodyCount--;
	},

	CreateJoint : function (def) {
		var j = b2Joint.Create(def, null),
			bodyA, bodyB,
			edge;

		j.m_prev = null;
		j.m_next = this.m_jointList;

		if (this.m_jointList) {
			this.m_jointList.m_prev = j;
		}

		this.m_jointList = j;

		this.m_jointCount++;

		j.m_edgeA.joint = j;
		j.m_edgeA.other = j.m_bodyB;
		j.m_edgeA.prev = null;
		j.m_edgeA.next = j.m_bodyA.m_jointList;

		if (j.m_bodyA.m_jointList) {
			j.m_bodyA.m_jointList.prev = j.m_edgeA;
		}

		j.m_bodyA.m_jointList = j.m_edgeA;
		j.m_edgeB.joint = j;
		j.m_edgeB.other = j.m_bodyA;
		j.m_edgeB.prev = null;
		j.m_edgeB.next = j.m_bodyB.m_jointList;

		if (j.m_bodyB.m_jointList) {
			j.m_bodyB.m_jointList.prev = j.m_edgeB;
		}

		j.m_bodyB.m_jointList = j.m_edgeB;

		bodyA = def.bodyA;
		bodyB = def.bodyB;

		if (!def.collideConnected) {
			edge = bodyB.GetContactList();
			while (edge) {
				if (edge.other === bodyA) {
					edge.contact.FlagForFiltering();
				}
				edge = edge.next;
			}
		}
		return j;
	},

	DestroyJoint : function (j) {
		var collideConnected = j.m_collideConnected,
			bodyA = j.m_bodyA,
			bodyB = j.m_bodyB,
			edge;

		bodyA.SetAwake(true);
		bodyB.SetAwake(true);

		if (j.m_prev) {
			j.m_prev.m_next = j.m_next;
		}

		if (j.m_next) {
			j.m_next.m_prev = j.m_prev;
		}

		if (j === this.m_jointList) {
			this.m_jointList = j.m_next;
		}

		if (j.m_edgeA.prev) {
			j.m_edgeA.prev.next = j.m_edgeA.next;
		}

		if (j.m_edgeA.next) {
			j.m_edgeA.next.prev = j.m_edgeA.prev;
		}

		if (j.m_edgeA === bodyA.m_jointList) {
			bodyA.m_jointList = j.m_edgeA.next;
		}

		j.m_edgeA.prev = null;
		j.m_edgeA.next = null;

		if (j.m_edgeB.prev) {
			j.m_edgeB.prev.next = j.m_edgeB.next;
		}

		if (j.m_edgeB.next) {
			j.m_edgeB.next.prev = j.m_edgeB.prev;
		}

		if (j.m_edgeB === bodyB.m_jointList) {
			bodyB.m_jointList = j.m_edgeB.next;
		}

		j.m_edgeB.prev = null;
		j.m_edgeB.next = null;

		b2Joint.Destroy(j, null);

		this.m_jointCount--;

		if (!collideConnected) {
			edge = bodyB.GetContactList();

			while (edge) {
				if (edge.other === bodyA) {
					edge.contact.FlagForFiltering();
				}
				edge = edge.next;
			}
		}
	},

	AddController : function (c) {
		c.m_world = this;
		c.m_next = this.m_controllerList;
		c.m_prev = null;

		this.m_controllerList = c;
		this.m_controllerCount++;

		return c;
	},

	RemoveController : function (c) {
		if (c.m_prev) {
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next) {
			c.m_next.m_prev = c.m_prev;
		}

		if (this.m_controllerList === c) {
			this.m_controllerList = c.m_next;
		}

		this.m_controllerCount--;
	},

	CreateController : function (controller) {
		b2Settings.b2Assert(controller.m_world === this);

		controller.m_next = this.m_controllerList;
		controller.m_prev = null;

		if (this.m_controllerList) {
			this.m_controllerList.m_prev = controller;
		}

		this.m_controllerList = controller;
		this.m_controllerCount++;

		controller.m_world = this;

		return controller;
	},

	DestroyController : function (controller) {
		controller.Clear();

		if (controller.m_next) {
			controller.m_next.m_prev = controller.m_prev;
		}

		if (controller.m_prev) {
			controller.m_prev.m_next = controller.m_next;
		}

		if (controller === this.m_controllerList) {
			this.m_controllerList = controller.m_next;
		}

		this.m_controllerCount--;
	},

	SetWarmStarting : function (flag) {
		b2World.m_warmStarting = flag;
	},

	SetContinuousPhysics : function (flag) {
		b2World.m_continuousPhysics = flag;
	},

	GetBodyCount : function () {
		return this.m_bodyCount;
	},

	GetJointCount : function () {
		return this.m_jointCount;
	},

	GetContactCount : function () {
		return this.m_contactCount;
	},

	SetGravity : function (gravity) {
		this.m_gravity = gravity;
	},

	GetGravity : function () {
		return this.m_gravity;
	},

	GetGroundBody : function () {
		return this.m_groundBody;
	},

	Step : function (dt, velocityIterations, positionIterations) {
		var step = b2World.s_timestep2;

		dt = dt || 0;
		velocityIterations = velocityIterations || 0;
		positionIterations = positionIterations || 0;

		if (this.m_flags & b2World.e_newFixture) {
			this.m_contactManager.FindNewContacts();
			this.m_flags &= ~b2World.e_newFixture;
		}

		this.m_flags |= b2World.e_locked;

		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;

		if (dt > 0) {
			step.inv_dt = 1 / dt;
		} else {
			step.inv_dt = 0;
		}

		step.dtRatio = this.m_inv_dt0 * dt;
		step.warmStarting = b2World.m_warmStarting;
		this.m_contactManager.Collide();

		if (step.dt > 0) {
			this.Solve(step);
		}

		if (b2World.m_continuousPhysics && step.dt > 0) {
			this.SolveTOI(step);
		}

		if (step.dt > 0) {
			this.m_inv_dt0 = step.inv_dt;
		}

		this.m_flags &= ~b2World.e_locked;
	},

	ClearForces : function () {
		var body = this.m_bodyList;

		while (body) {
			body.m_force.SetZero();
			body.m_torque = 0;
			body = body.m_next;
		}
	},

	DrawDebugData : function () {
		if (!this.m_debugDraw) {
			return;
		}

		var debugDraw = this.m_debugDraw,
			flags = this.m_debugDraw.GetFlags(),
			i,
			b = this.m_bodyList,
			j = this.m_jointList,
			c = this.m_controllerList,
			f,
			s,
			bp = this.m_contactManager.m_broadPhase,
			invQ = new b2Vec2(),
			x1 = new b2Vec2(),
			x2 = new b2Vec2(),
			xf,
			b1 = new b2AABB(),
			b2 = new b2AABB(),
			vs = [
				new b2Vec2(),
				new b2Vec2(),
				new b2Vec2(),
				new b2Vec2()
			],
			color = new b2Color(),
			contact = this.m_contactManager.m_contactList,
			aabb;

		debugDraw.Clear();

		if (flags & b2DebugDraw.e_shapeBit) {
			while (b) {
				xf = b.m_xf;
				f = b.GetFixtureList();

				while (f) {
					s = f.GetShape();

					if (!b.IsActive()) {
						color.Set(0.5, 0.5, 0.3);
						this.DrawShape(s, xf, color);
					} else if (b.GetType() === b2Body.b2_staticBody) {
						color.Set(0.5, 0.9, 0.5);
						this.DrawShape(s, xf, color);
					} else if (b.GetType() === b2Body.b2_kinematicBody) {
						color.Set(0.5, 0.5, 0.9);
						this.DrawShape(s, xf, color);
					} else if (!b.IsAwake()) {
						color.Set(0.6, 0.6, 0.6);
						this.DrawShape(s, xf, color);
					} else {
						color.Set(0.9, 0.7, 0.7);
						this.DrawShape(s, xf, color);
					}

					f = f.m_next;
				}
				b = b.m_next;
			}
		}

		if (flags & b2DebugDraw.e_jointBit) {
			while (j) {
				this.DrawJoint(j);
				j = j.m_next;
			}
		}

		if (flags & b2DebugDraw.e_controllerBit) {
			while (c) {
				c.Draw(debugDraw);
				c = c.m_next;
			}
		}

		if (flags & b2DebugDraw.e_pairBit) {
			color.Set(0.3, 0.9, 0.9);

			while (contact) {
				debugDraw.DrawSegment(
					contact.GetFixtureA().GetAABB().GetCenter(),
					contact.GetFixtureB().GetAABB().GetCenter(),
				color);
				contact = contact.GetNext();
			}
		}

		if (flags & b2DebugDraw.e_aabbBit) {
			for (b = this.m_bodyList; b; b = b.m_next) {
				if (!b.IsActive()) {
					continue;
				}

				f = b.GetFixtureList();

				while (f) {
					aabb = bp.GetFatAABB(f.m_proxy);

					vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
					vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
					vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
					vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

					debugDraw.DrawPolygon(vs, 4, color);

					f = f.m_next;
				}
			}
		}

		if (flags & b2DebugDraw.e_centerOfMassBit) {
			b = this.m_bodyList;

			while (b) {
				xf = b2World.s_xf;
				xf.R = b.m_xf.R;
				xf.position = b.GetWorldCenter();
				debugDraw.DrawTransform(xf);

				b = b.m_next;
			}
		}
	},

	QueryAABB : function (callback, aabb) {
		var broadPhase = this.m_contactManager.m_broadPhase;

		broadPhase.Query(function (proxy) {
			return callback(broadPhase.GetUserData(proxy));
		}, aabb);
	},

	QueryShape : function (callback, shape, transform) {
		if (!transform) {
			transform = new b2Transform();
			transform.SetIdentity();
		}

		var broadPhase = this.m_contactManager.m_broadPhase,
			aabb = new b2AABB();

		shape.ComputeAABB(aabb, transform);

		broadPhase.Query(function (proxy) {
			var fixture = broadPhase.GetUserData(proxy);

			if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) {
				return callback(fixture);
			}

			return true;
		}, aabb);
	},

	QueryPoint : function (callback, p) {
		var broadPhase = this.m_contactManager.m_broadPhase,
			aabb = new b2AABB();

		aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
		aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);

		broadPhase.Query(function (proxy) {
			var fixture = broadPhase.GetUserData(proxy);
			if (fixture.TestPoint(p)) {
				return callback(fixture);
			}
			return true;
		}, aabb);
	},

	RayCast : function (callback, point1, point2) {
		var broadPhase = this.m_contactManager.m_broadPhase,
			output = new b2RayCastOutput(),
			input = new b2RayCastInput(point1, point2);

		broadPhase.RayCast(function (input, proxy) {
			var fixture = broadPhase.GetUserData(proxy),
				hit = fixture.RayCast(output, input),
				fraction, point;

			if (hit) {
				fraction = output.fraction;
				point = new b2Vec2(
					(1 - fraction) * point1.x + fraction * point2.x,
					(1 - fraction) * point1.y + fraction * point2.y
				);
				return callback(fixture, point, output.normal, fraction);
			}

			return input.maxFraction;
		}, input);
	},

	RayCastOne : function (point1, point2) {
		var result;

		this.RayCast(function RayCastOneWrapper(fixture, point, normal, fraction) {
			result = fixture;
			return fraction || 0;
		}, point1, point2);

		return result;
	},

	RayCastAll : function (point1, point2) {
		var result = [];

		this.RayCast(function (fixture, point, normal, fraction) {
			result[result.length] = fixture;
			return 1;
		}, point1, point2);

		return result;
	},

	GetBodyList : function () {
		return this.m_bodyList;
	},

	GetJointList : function () {
		return this.m_jointList;
	},

	GetContactList : function () {
		return this.m_contactList;
	},

	IsLocked : function () {
		return (this.m_flags & b2World.e_locked) > 0;
	},

	Solve : function (step) {
		var b = this.m_bodyList,
			c = this.m_contactList,
			j = this.m_jointList,
			controller = this.m_controllerList,
			island = this.m_island,
			stackSize = this.m_bodyCount,
			stack = this.s_stack,
			seed,
			stackCount,
			other,
			ce,
			jn,
			i;

		while (controller) {
			controller.Step(step);
			controller = controller.m_next;
		}

		island.Initialize(
			this.m_bodyCount,
			this.m_contactCount,
			this.m_jointCount,
			null,
			this.m_contactManager.m_contactListener,
			this.m_contactSolver
		);

		while (b) {
			b.m_flags &= ~b2Body.e_islandFlag;
			b = b.m_next;
		}

		while (c) {
			c.m_flags &= ~b2Contact.e_islandFlag;
			c = c.m_next;
		}

		while (j) {
			j.m_islandFlag = false;
			j = j.m_next;
		}

		for (seed = this.m_bodyList; seed; seed = seed.m_next) {
			if (seed.m_flags & b2Body.e_islandFlag ||
				!seed.IsAwake() ||
				!seed.IsActive() ||
				seed.GetType() === b2Body.b2_staticBody) {
				continue;
			}

			island.Clear();

			stackCount = 0;
			stack[stackCount++] = seed;

			seed.m_flags |= b2Body.e_islandFlag;

			while (stackCount > 0) {
				b = stack[--stackCount];
				b.SetAwake(true);

				island.AddBody(b);

				if (b.GetType() === b2Body.b2_staticBody) {
					continue;
				}

				for (ce = b.m_contactList; ce; ce = ce.next) {
					if (ce.contact.m_flags & b2Contact.e_islandFlag ||
						ce.contact.IsSensor() ||
						!ce.contact.IsEnabled() ||
						!ce.contact.IsTouching()) {
						continue;
					}

					other = ce.other;

					island.AddContact(ce.contact);
					ce.contact.m_flags |= b2Contact.e_islandFlag;

					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}
					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}

				for (jn = b.m_jointList; jn; jn = jn.next) {
					other = jn.other;

					if (jn.joint.m_islandFlag || !other.IsActive()) {
						continue;
					}

					island.AddJoint(jn.joint);
					jn.joint.m_islandFlag = true;

					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					stack[stackCount++] = other;
					other.m_flags |= b2Body.e_islandFlag;
				}
			}

			island.Solve(step, this.m_gravity, this.m_allowSleep);

			for (i = 0; i < island.m_bodyCount; i++) {
				b = island.m_bodies[i];
				if (b.GetType() === b2Body.b2_staticBody) {
					b.m_flags &= ~b2Body.e_islandFlag;
				}
			}
		}

		for (i = 0; i < stack.length; i++) {
			if (!stack[i]) {
				break;
			}
			stack[i] = null;
		}

		b = this.m_bodyList;

		while (b) {
			if (!b.IsAwake() || !b.IsActive() || b.GetType() === b2Body.b2_staticBody) {
				continue;
			}

			b.SynchronizeFixtures();

			b = b.m_next;
		}

		this.m_contactManager.FindNewContacts();
	},

	SolveTOI : function (step) {
		var b = this.m_bodyList,
			c = this.m_contactList,
			j = this.m_jointList,
			fA, fB,
			bA, bB,
			cEdge,
			island = this.m_island,
			queue = b2World.s_queue,
			minContact,
			minTOI,
			toi,
			t0,
			seed,
			queueStart,
			queueSize,
			other,
			jEdge,
			subStep,
			i;

		island.Initialize(
			this.m_bodyCount,
			b2Settings.b2_maxTOIContactsPerIsland,
			b2Settings.b2_maxTOIJointsPerIsland,
			null,
			this.m_contactManager.m_contactListener,
			this.m_contactSolver
		);

		while (b) {
			b.m_flags &= ~b2Body.e_islandFlag;
			b.m_sweep.t0 = 0;
			b = b.m_next;
		}

		while (c) {
			c.m_flags &= ~ (b2Contact.e_toiFlag | b2Contact.e_islandFlag);
			c = c.m_next;
		}

		while (j) {
			j.m_islandFlag = false;
			j = j.m_next;
		}

		while (true) {
			minContact = null;
			minTOI = 1;

			for (c = this.m_contactList; c; c = c.m_next) {
				if (c.IsSensor() || !c.IsEnabled() || !c.IsContinuous()) {
					continue;
				}

				if (c.m_flags & b2Contact.e_toiFlag) {
					toi = c.m_toi;
				} else {
					fA = c.m_fixtureA;
					fB = c.m_fixtureB;
					bA = fA.m_body;
					bB = fB.m_body;

					if ((bA.GetType() !== b2Body.b2_dynamicBody || !bA.IsAwake()) &&
						(bB.GetType() !== b2Body.b2_dynamicBody || !bB.IsAwake())) {
						continue;
					}

					if (bA.m_sweep.t0 < bB.m_sweep.t0) {
						t0 = bB.m_sweep.t0;
						bA.m_sweep.Advance(t0);
					} else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
						t0 = bA.m_sweep.t0;
						bB.m_sweep.Advance(t0);
					}

					toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);

					b2Settings.b2Assert(toi >= 0 && toi <= 1);

					if (toi > 0 && toi < 1) {
						toi = (1 - toi) * t0 + toi;
						if (toi > 1) {
							toi = 1;
						}
					}

					c.m_toi = toi;
					c.m_flags |= b2Contact.e_toiFlag;
				}

				if (toi > Number.MIN_VALUE && toi < minTOI) {
					minContact = c;
					minTOI = toi;
				}
			}

			if (!minContact || 1 - (100 * Number.MIN_VALUE) < minTOI) {
				break;
			}

			fA = minContact.m_fixtureA;
			fB = minContact.m_fixtureB;
			bA = fA.m_body;
			bB = fB.m_body;

			b2World.s_backupA.Set(bA.m_sweep);
			b2World.s_backupB.Set(bB.m_sweep);

			bA.Advance(minTOI);
			bB.Advance(minTOI);

			minContact.Update(this.m_contactManager.m_contactListener);
			minContact.m_flags &= ~b2Contact.e_toiFlag;

			if (minContact.IsSensor() || !minContact.IsEnabled()) {
				bA.m_sweep.Set(b2World.s_backupA);
				bB.m_sweep.Set(b2World.s_backupB);
				bA.SynchronizeTransform();
				bB.SynchronizeTransform();
				continue;
			}

			if (!minContact.IsTouching()) {
				continue;
			}

			seed = bA;

			if (seed.GetType() !== b2Body.b2_dynamicBody) {
				seed = bB;
			}

			island.Clear();

			queueStart = queueSize = 0;

			queue[queueSize++] = seed;

			seed.m_flags |= b2Body.e_islandFlag;

			while (queueSize > 0) {
				queueSize--;

				b = queue[queueStart++];
				b.SetAwake(true);

				island.AddBody(b);

				if (b.GetType() !== b2Body.b2_dynamicBody) {
					continue;
				}

				for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
					if (island.m_contactCount === island.m_contactCapacity) {
						break;
					}

					if (cEdge.contact.m_flags & b2Contact.e_islandFlag ||
						cEdge.contact.IsSensor() ||
						!cEdge.contact.IsEnabled() ||
						!cEdge.contact.IsTouching()) {
						continue;
					}

					island.AddContact(cEdge.contact);

					cEdge.contact.m_flags |= b2Contact.e_islandFlag;

					other = cEdge.other;

					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					if (other.GetType() !== b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}

					queue[queueStart + queueSize] = other;
					queueSize++;

					other.m_flags |= b2Body.e_islandFlag;
				}

				for (jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
					other = jEdge.other;

					if (island.m_jointCount === island.m_jointCapacity ||
						jEdge.joint.m_islandFlag ||
						!other.IsActive()) {
						continue;
					}

					island.AddJoint(jEdge.joint);
					jEdge.joint.m_islandFlag = true;

					if (other.m_flags & b2Body.e_islandFlag) {
						continue;
					}

					if (other.GetType() !== b2Body.b2_staticBody) {
						other.Advance(minTOI);
						other.SetAwake(true);
					}

					queue[queueStart + queueSize] = other;
					queueSize++;

					other.m_flags |= b2Body.e_islandFlag;
				}
			}

			subStep = b2World.s_timestep;
			subStep.warmStarting = false;
			subStep.dt = (1 - minTOI) * step.dt;
			subStep.inv_dt = 1 / subStep.dt;
			subStep.dtRatio = 0;
			subStep.velocityIterations = step.velocityIterations;
			subStep.positionIterations = step.positionIterations;

			island.SolveTOI(subStep);

			for (i = 0; i < island.m_bodyCount; i++) {
				b = island.m_bodies[i];
				b.m_flags &= ~b2Body.e_islandFlag;

				if (!b.IsAwake() ||
					b.GetType() !== b2Body.b2_dynamicBody) {
					continue;
				}

				b.SynchronizeFixtures();

				for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
					cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
				}
			}

			for (i = 0; i < island.m_contactCount; i++) {
				c = island.m_contacts[i];
				c.m_flags &= ~ (b2Contact.e_toiFlag | b2Contact.e_islandFlag);
			}

			for (i = 0; i < island.m_jointCount; i++) {
				j = island.m_joints[i];
				j.m_islandFlag = false;
			}

			this.m_contactManager.FindNewContacts();
		}
	},

	DrawJoint : function (joint) {
		var b1 = joint.GetBodyA(),
			b2 = joint.GetBodyB(),
			xf1 = b1.m_xf,
			xf2 = b2.m_xf,
			x1 = xf1.position,
			x2 = xf2.position,
			p1 = joint.GetAnchorA(),
			p2 = joint.GetAnchorB(),
			color = b2World.s_jointColor,
			pulley,
			s1, s2,
			debugDraw = this.m_debugDraw;

		switch (joint.m_type) {
		case b2Joint.e_distanceJoint:
		case b2Joint.e_mouseJoint:
			debugDraw.DrawSegment(p1, p2, color);
			break;
		case b2Joint.e_pulleyJoint:
			s1 = joint.GetGroundAnchorA();
			s2 = joint.GetGroundAnchorB();
			debugDraw.DrawSegment(s1, p1, color);
			debugDraw.DrawSegment(s2, p2, color);
			debugDraw.DrawSegment(s1, s2, color);
			break;
		default:
			if (b1 !== this.m_groundBody) {
				debugDraw.DrawSegment(x1, p1, color);
			}
			if (b2 !== this.m_groundBody) {
				debugDraw.DrawSegment(x2, p2, color);
			}
			debugDraw.DrawSegment(p1, p2, color);
		}
	},

	DrawShape : function (shape, xf, color) {
		var i,
			localVertices,
			vertices,
			debugDraw = this.m_debugDraw;

		switch (shape.m_type) {
		case b2Shape.e_circleShape:
			debugDraw.DrawSolidCircle(
				b2Math.MulX(xf, shape.m_p),
				shape.m_radius,
				xf.R.col1,
				color
			);
			break;
		case b2Shape.e_polygonShape:
			localVertices = shape.GetVertices();
			vertices = [];
			for (i = 0; i < localVertices.length; i++) {
				vertices[i] = b2Math.MulX(xf, localVertices[i]);
			}
			debugDraw.DrawSolidPolygon(vertices, localVertices.length, color);
			break;
		case b2Shape.e_edgeShape:
			debugDraw.DrawSegment(b2Math.MulX(xf, shape.GetVertex1()), b2Math.MulX(xf, shape.GetVertex2()), color);
			break;
		}
	}
};

whenReady(function () {
	b2World.s_timestep2 = new b2TimeStep();
	b2World.s_xf = new b2Transform();
	b2World.s_backupA = new b2Sweep();
	b2World.s_backupB = new b2Sweep();
	b2World.s_timestep = new b2TimeStep();
	b2World.s_queue = [];
	b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
});
