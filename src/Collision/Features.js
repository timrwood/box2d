function Features() {

}

Features.prototype = {

};

defineProperty(Features.prototype, 'referenceEdge', {
	enumerable: false,
	configurable: true,
	get: function () {
		return this._referenceEdge;
	}
});
defineProperty(Features.prototype, 'referenceEdge', {
	enumerable: false,
	configurable: true,
	set: function (value) {
		if (value === undefined) value = 0;
		this._referenceEdge = value;
		this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
	}
});
defineProperty(Features.prototype, 'incidentEdge', {
	enumerable: false,
	configurable: true,
	get: function () {
		return this._incidentEdge;
	}
});
defineProperty(Features.prototype, 'incidentEdge', {
	enumerable: false,
	configurable: true,
	set: function (value) {
		if (value === undefined) value = 0;
		this._incidentEdge = value;
		this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
	}
});
defineProperty(Features.prototype, 'incidentVertex', {
	enumerable: false,
	configurable: true,
	get: function () {
		return this._incidentVertex;
	}
});
defineProperty(Features.prototype, 'incidentVertex', {
	enumerable: false,
	configurable: true,
	set: function (value) {
		if (value === undefined) value = 0;
		this._incidentVertex = value;
		this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
	}
});
defineProperty(Features.prototype, 'flip', {
	enumerable: false,
	configurable: true,
	get: function () {
		return this._flip;
	}
});
defineProperty(Features.prototype, 'flip', {
	enumerable: false,
	configurable: true,
	set: function (value) {
		if (value === undefined) value = 0;
		this._flip = value;
		this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
	}
});