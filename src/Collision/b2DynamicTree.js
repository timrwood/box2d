function b2DynamicTree() {
	this.m_root = null;
	this.m_freeList = null;
	this.m_path = 0;
	this.m_insertionCount = 0;
}

Box2D.b2DynamicTree = b2DynamicTree;

b2DynamicTree.prototype = {
	CreateProxy : function (aabb, userData) {
		var node = this.AllocateNode(),
			extendX = b2Settings.b2_aabbExtension,
			extendY = b2Settings.b2_aabbExtension;

		node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
		node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
		node.aabb.upperBound.x = aabb.upperBound.x + extendX;
		node.aabb.upperBound.y = aabb.upperBound.y + extendY;
		node.userData = userData;
		this.InsertLeaf(node);
		return node;
	},

	DestroyProxy : function (proxy) {
		this.RemoveLeaf(proxy);
		this.FreeNode(proxy);
	},

	MoveProxy : function (proxy, aabb, displacement) {
		var extendX, extendY;
		b2Settings.b2Assert(proxy.IsLeaf());
		if (proxy.aabb.Contains(aabb)) {
			return false;
		}
		this.RemoveLeaf(proxy);

		extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
		extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));

		proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
		proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
		proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
		proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;

		this.InsertLeaf(proxy);
		return true;
	},

	Rebalance : function (iterations) {
		iterations = iterations || 0;
		var i, node, bit;
		if (!this.m_root) {
			return;
		}
		for (i = 0; i < iterations; i++) {
			node = this.m_root;
			bit = 0;
			while (!node.IsLeaf()) {
				node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
				bit = (bit + 1) & 31;
			}
			this.m_path++;
			this.RemoveLeaf(node);
			this.InsertLeaf(node);
		}
	},

	GetFatAABB : function (proxy) {
		return proxy.aabb;
	},

	GetUserData : function (proxy) {
		return proxy.userData;
	},

	Query : function (callback, aabb) {
		if (!this.m_root) {
			return;
		}
		var stack = [],
			count = 0,
			node, proceed;

		stack[count++] = this.m_root;

		while (count > 0) {
			node = stack[--count];
			if (node.aabb.TestOverlap(aabb)) {
				if (node.IsLeaf()) {
					proceed = callback(node);
					if (!proceed) {
						return;
					}
				} else {
					stack[count++] = node.child1;
					stack[count++] = node.child2;
				}
			}
		}
	},

	RayCast : function (callback, input) {
		if (!this.m_root) {
			return;
		}
		var p1 = input.p1,
			p2 = input.p2,
			r = b2Math.SubtractVV(p1, p2),
			v, abs_v,
			maxFraction = input.maxFraction,
			segmentAABB = new b2AABB(),
			tX = 0,
			tY = 0,
			stack = [],
			count = 0,
			node,
			c, h,
			separation,
			subInput;

		r.Normalize();
		v = b2Math.CrossFV(1, r);
		abs_v = b2Math.AbsV(v);

		tX = p1.x + maxFraction * (p2.x - p1.x);
		tY = p1.y + maxFraction * (p2.y - p1.y);
		segmentAABB.lowerBound.x = Math.min(p1.x, tX);
		segmentAABB.lowerBound.y = Math.min(p1.y, tY);
		segmentAABB.upperBound.x = Math.max(p1.x, tX);
		segmentAABB.upperBound.y = Math.max(p1.y, tY);

		stack[count++] = this.m_root;

		while (count > 0) {
			node = stack[--count];
			if (!node.aabb.TestOverlap(segmentAABB)) {
				continue;
			}

			c = node.aabb.GetCenter(); // TODO: Only make one?
			h = node.aabb.GetExtents(); // TODO: Only make one?
			separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;

			if (separation > 0) {
				continue;
			}
			if (node.IsLeaf()) {
				subInput = new b2RayCastInput(); // TODO: Only make one?
				subInput.p1 = input.p1;
				subInput.p2 = input.p2;
				subInput.maxFraction = input.maxFraction;
				maxFraction = callback(subInput, node);
				if (maxFraction === 0) {
					return;
				}
				if (maxFraction > 0) {
					tX = p1.x + maxFraction * (p2.x - p1.x);
					tY = p1.y + maxFraction * (p2.y - p1.y);
					segmentAABB.lowerBound.x = Math.min(p1.x, tX);
					segmentAABB.lowerBound.y = Math.min(p1.y, tY);
					segmentAABB.upperBound.x = Math.max(p1.x, tX);
					segmentAABB.upperBound.y = Math.max(p1.y, tY);
				}
			} else {
				stack[count++] = node.child1;
				stack[count++] = node.child2;
			}
		}
	},

	AllocateNode : function () {
		var node;
		if (this.m_freeList) {
			node = this.m_freeList;
			this.m_freeList = node.parent;
			node.parent = null;
			node.child1 = null;
			node.child2 = null;
			return node;
		}
		return new b2DynamicTreeNode();
	},

	FreeNode : function (node) {
		node.parent = this.m_freeList;
		this.m_freeList = node;
	},

	InsertLeaf : function (leaf) {
		var center = b2DynamicTree.t_vec2a,
			sibling,
			child1, child2,
			norm1, norm2,
			node1, node2;

		this.m_insertionCount++;

		if (!this.m_root) {
			this.m_root = leaf;
			this.m_root.parent = null;
			return;
		}

		center = leaf.aabb.GetCenter(center);
		sibling = this.m_root;

		while (!sibling.IsLeaf()) {
			child1 = sibling.child1;
			child2 = sibling.child2;
			norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
			norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
			if (norm1 < norm2) {
				sibling = child1;
			} else {
				sibling = child2;
			}
		}

		node1 = sibling.parent;
		node2 = this.AllocateNode();
		node2.parent = node1;
		node2.userData = null;
		node2.aabb.Combine(leaf.aabb, sibling.aabb);

		if (node1) {
			if (sibling.parent.child1 === sibling) {
				node1.child1 = node2;
			} else {
				node1.child2 = node2;
			}
			node2.child1 = sibling;
			node2.child2 = leaf;
			sibling.parent = node2;
			leaf.parent = node2;
			do {
				if (node1.aabb.Contains(node2.aabb)) {
					break;
				}
				node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
				node2 = node1;
				node1 = node1.parent;
			} while (node1);
		} else {
			node2.child1 = sibling;
			node2.child2 = leaf;
			sibling.parent = node2;
			leaf.parent = node2;
			this.m_root = node2;
		}
	},

	RemoveLeaf : function (leaf) {
		var node1, node2,
			sibling,
			oldAABB;

		if (leaf === this.m_root) {
			this.m_root = null;
			return;
		}

		node2 = leaf.parent;
		node1 = node2.parent;

		if (node2.child1 === leaf) {
			sibling = node2.child2;
		} else {
			sibling = node2.child1;
		}

		if (node1) {
			if (node1.child1 === node2) {
				node1.child1 = sibling;
			} else {
				node1.child2 = sibling;
			}
			sibling.parent = node1;
			this.FreeNode(node2);
			while (node1) {
				oldAABB = node1.aabb;
				node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb); // TODO: Check for b2AABB reuse?
				if (oldAABB.Contains(node1.aabb)) {
					break;
				}
				node1 = node1.parent;
			}
		} else {
			this.m_root = sibling;
			sibling.parent = null;
			this.FreeNode(node2);
		}
	}
};


function b2DynamicTreeNode() {
	this.aabb = new b2AABB();
}

b2DynamicTreeNode.prototype = {
	IsLeaf : function () {
		return !this.child1;
	}
};

function b2DynamicTreePair() {}


whenReady(function () {
	b2DynamicTree.t_vec2a = new b2Vec2();
});
