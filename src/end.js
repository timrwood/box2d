ready();

this.Box2D = Box2D;

/*global module:false */
if (typeof module !== "undefined") {
	module.exports = Box2D;
}

/*global define:false */
if (typeof define === "function" && define.amd) {
	define("Box2D", [], function () {
		return Box2D;
	});
}
