function b2DynamicTreeBroadPhase() {
	this.m_tree = new b2DynamicTree();
	this.m_moveBuffer = [];
	this.m_pairBuffer = [];
	this.m_pairCount = 0;
}

Box2D.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase;

b2DynamicTreeBroadPhase.prototype = {

	CreateProxy : function (aabb, userData) {
		var proxy = this.m_tree.CreateProxy(aabb, userData);
		this.m_proxyCount++;
		this.BufferMove(proxy);
		return proxy;
	},

	DestroyProxy : function (proxy) {
		this.UnBufferMove(proxy);
		this.m_proxyCount--;
		this.m_tree.DestroyProxy(proxy);
	},

	MoveProxy : function (proxy, aabb, displacement) {
		if (this.m_tree.MoveProxy(proxy, aabb, displacement)) {
			this.BufferMove(proxy);
		}
	},

	TestOverlap : function (proxyA, proxyB) {
		return this.m_tree.GetFatAABB(proxyA).TestOverlap(this.m_tree.GetFatAABB(proxyB));
	},

	GetUserData : function (proxy) {
		return this.m_tree.GetUserData(proxy);
	},

	GetFatAABB : function (proxy) {
		return this.m_tree.GetFatAABB(proxy);
	},

	GetProxyCount : function () {
		return this.m_proxyCount;
	},

	__Query : function () {

	},

	UpdatePairs : function (callback) {
		var __this = this,
			m_pairBuffer = this.m_pairBuffer,
			m_moveBuffer = this.m_moveBuffer,
			m_tree = this.m_tree,
			i,
			pair,
			fatAABB,
			userDataA,
			userDataB,
			queryProxy,
			primaryPair;

		this.m_pairCount = 0;

		/*jshint loopfunc: true */
		for (i = 0; i < m_moveBuffer.length; i++) {
			queryProxy = m_moveBuffer[i];
			fatAABB = m_tree.GetFatAABB(queryProxy);
			m_tree.Query(function (proxy) {
				var pair;
				if (proxy === queryProxy) {
					return true;
				}
				if (__this.m_pairCount === m_pairBuffer.length) {
					m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
				}
				pair = m_pairBuffer[__this.m_pairCount];
				pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
				pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
				__this.m_pairCount++;
				return true;
			}, fatAABB);
		}

		m_moveBuffer.length = 0;

		for (i = 0; i < this.m_pairCount;) {
			primaryPair = m_pairBuffer[i];
			userDataA = m_tree.GetUserData(primaryPair.proxyA);
			userDataB = m_tree.GetUserData(primaryPair.proxyB);
			callback(userDataA, userDataB);
			i++;
			while (i < this.m_pairCount) {
				pair = m_pairBuffer[i];
				if (pair.proxyA !== primaryPair.proxyA || pair.proxyB !== primaryPair.proxyB) {
					break;
				}
				i++;
			}
		}
	},

	Query : function (callback, aabb) {
		this.m_tree.Query(callback, aabb);
	},

	RayCast : function (callback, input) {
		this.m_tree.RayCast(callback, input);
	},

	Validate : function () {},

	Rebalance : function (iterations) {
		this.m_tree.Rebalance(iterations || 0);
	},

	BufferMove : function (proxy) {
		this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
	},

	UnBufferMove : function (proxy) {
		this.m_moveBuffer.splice(this.m_moveBuffer.indexOf(proxy), 1);
	},

	ComparePairs : function (pair1, pair2) {
		return 0;
	}
};
