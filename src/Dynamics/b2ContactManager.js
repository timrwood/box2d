function b2ContactManager() {
	this.m_world = null;
	this.m_contactCount = 0;
	this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
	this.m_contactListener = b2ContactListener.b2_defaultListener;
	this.m_contactFactory = new b2ContactFactory(this.m_allocator);
	this.m_broadPhase = new b2DynamicTreeBroadPhase();
}

b2ContactManager.prototype = {
	AddPair : function (fixtureA, fixtureB) {
		if (!fixtureA instanceof b2Fixture || !fixtureB instanceof b2Fixture) {
			return;
		}

		var bodyA = fixtureA.GetBody(),
			bodyB = fixtureB.GetBody(),
			edge = bodyB.GetContactList(),
			fA, fB,
			c;

		if (bodyA === bodyB) {
			return;
		}

		edge = bodyB.GetContactList();

		while (edge) {
			if (edge.other === bodyA) {
				fA = edge.contact.GetFixtureA();
				fB = edge.contact.GetFixtureB();
				if (fA === fixtureA && fB === fixtureB) {
					return;
				}
				if (fA === fixtureB && fB === fixtureA) {
					return;
				}
			}
			edge = edge.next;
		}

		if (!bodyB.ShouldCollide(bodyA)) {
			return;
		}

		if (!this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)) {
			return;
		}

		c = this.m_contactFactory.Create(fixtureA, fixtureB);

		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();

		bodyA = fixtureA.m_body;
		bodyB = fixtureB.m_body;

		c.m_prev = null;
		c.m_next = this.m_world.m_contactList;

		if (this.m_world.m_contactList) {
			this.m_world.m_contactList.m_prev = c;
		}

		this.m_world.m_contactList = c;

		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;
		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;

		if (bodyA.m_contactList) {
			bodyA.m_contactList.prev = c.m_nodeA;
		}

		bodyA.m_contactList = c.m_nodeA;
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;
		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;

		if (bodyB.m_contactList) {
			bodyB.m_contactList.prev = c.m_nodeB;
		}

		bodyB.m_contactList = c.m_nodeB;
		this.m_world.m_contactCount++;

		return;
	},

	FindNewContacts : function () {
		this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
	},

	Destroy : function (c) {
		var fixtureA = c.GetFixtureA(),
			fixtureB = c.GetFixtureB(),
			bodyA = fixtureA.GetBody(),
			bodyB = fixtureB.GetBody();
		if (c.IsTouching()) {
			this.m_contactListener.EndContact(c);
		}
		if (c.m_prev) {
			c.m_prev.m_next = c.m_next;
		}
		if (c.m_next) {
			c.m_next.m_prev = c.m_prev;
		}
		if (c === this.m_world.m_contactList) {
			this.m_world.m_contactList = c.m_next;
		}
		if (c.m_nodeA.prev) {
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}
		if (c.m_nodeA.next) {
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}
		if (c.m_nodeA === bodyA.m_contactList) {
			bodyA.m_contactList = c.m_nodeA.next;
		}
		if (c.m_nodeB.prev) {
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}
		if (c.m_nodeB.next) {
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}
		if (c.m_nodeB === bodyB.m_contactList) {
			bodyB.m_contactList = c.m_nodeB.next;
		}
		this.m_contactFactory.Destroy(c);
		this.m_contactCount--;
	},

	Collide : function () {
		var c = this.m_world.m_contactList,
			fixtureA, fixtureB,
			bodyA, bodyB,
			overlap,
			cNuke;

		while (c) {
			fixtureA = c.GetFixtureA();
			fixtureB = c.GetFixtureB();
			bodyA = fixtureA.GetBody();
			bodyB = fixtureB.GetBody();

			if (!bodyA.IsAwake() && !bodyB.IsAwake()) {
				c = c.GetNext();
				continue;
			}

			if (c.m_flags & b2Contact.e_filterFlag) {
				if (!bodyB.ShouldCollide(bodyA)) {
					cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				if (!this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)) {
					cNuke = c;
					c = cNuke.GetNext();
					this.Destroy(cNuke);
					continue;
				}

				c.m_flags &= ~b2Contact.e_filterFlag;
			}

			overlap = this.m_broadPhase.TestOverlap(fixtureA.m_proxy, fixtureB.m_proxy);
			if (!overlap) {
				cNuke = c;
				c = cNuke.GetNext();
				this.Destroy(cNuke);
				continue;
			}

			c.Update(this.m_contactListener);
			c = c.GetNext();
		}
	}
};