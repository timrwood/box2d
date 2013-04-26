function ClipVertex() {
	this.v = new b2Vec2();
	this.id = new b2ContactID();
}

Box2D.ClipVertex = ClipVertex;

ClipVertex.prototype = {
	Set : function (other) {
		this.v.SetV(other.v);
		this.id.Set(other.id);
	}
};
