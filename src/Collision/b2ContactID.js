function b2ContactID() {
	this.features = new Features();
	this.features._m_id = this;
}

Box2D.b2ContactID = b2ContactID;

b2ContactID.prototype = {
	Set : function (id) {
		this.key = id._key;
	},

	Copy : function () {
		var id = new b2ContactID();
		id.key = this.key;
		return id;
	}
};


defineProperty(b2ContactID.prototype, 'key',
	function () {
		return this._key;
	},
	function (value) {
		this._key = value || 0;
		this.features._referenceEdge = this._key & 0x000000ff;
		this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
		this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
		this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
	}
);
