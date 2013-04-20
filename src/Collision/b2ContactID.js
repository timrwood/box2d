function b2ContactID () {
	this.features = new Features();
}

b2ContactID.prototype = {
	b2ContactID : function () {
		this.features._m_id = this;
	},

	Set : function (id) {
		this.key = id._key;
	},

	Copy : function () {
		var id = new b2ContactID();
		id.key = this.key;
		return id;
	}
};


defineProperty(b2ContactID.prototype, 'key', {
	enumerable: false,
	configurable: true,
	get: function () {
		return this._key;
	}
});

defineProperty(b2ContactID.prototype, 'key', {
	enumerable: false,
	configurable: true,
	set: function (value) {
		this._key = value || 0;
		this.features._referenceEdge = this._key & 0x000000ff;
		this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
		this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
		this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
	}
});