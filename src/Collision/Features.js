function Features() {

}

Features.prototype = {

};

defineProperty(Features.prototype, 'referenceEdge',
	function () {
		return this._referenceEdge;
	},
	function (value) {
		if (value === undefined) value = 0;
		this._referenceEdge = value;
		this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
	}
);

defineProperty(Features.prototype, 'incidentEdge',
	function () {
		return this._incidentEdge;
	},
	function (value) {
		if (value === undefined) value = 0;
		this._incidentEdge = value;
		this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
	}
);

defineProperty(Features.prototype, 'incidentVertex',
	function () {
		return this._incidentVertex;
	},
	function (value) {
		if (value === undefined) value = 0;
		this._incidentVertex = value;
		this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
	}
);

defineProperty(Features.prototype, 'flip',
	function () {
		return this._flip;
	},
	function (value) {
		if (value === undefined) value = 0;
		this._flip = value;
		this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
	}
);
