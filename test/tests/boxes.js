tests.push(function(world){
	var fixDef = new Box2D.b2FixtureDef(),
		bodyDef = new Box2D.b2BodyDef(),
		i;

	fixDef.density = 1;
	fixDef.friction = 0.5;
	fixDef.restitution = 0.2;

	bodyDef.type = Box2D.b2Body.b2_dynamicBody;

	for (i = 0; i < 20; i++) {
		fixDef.shape = Box2D.b2PolygonShape.AsBox(
			0.1 + Math.random(),
			0.1 + Math.random()
		);

		bodyDef.position.x = Math.random() * 20;
		bodyDef.position.y = Math.random() * 10;
		bodyDef.angle = Math.random() * Math.PI;
		world.CreateBody(bodyDef).CreateFixture(fixDef);
	}
});
