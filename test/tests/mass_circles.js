tests.push(function(world){
	var fixDef = new Box2D.b2FixtureDef(),
		bodyDef = new Box2D.b2BodyDef(),
		i;

	fixDef.friction = 0.5;

	bodyDef.type = Box2D.b2Body.b2_dynamicBody;

	for (i = 0; i < 200; i++) {
		fixDef.shape = new Box2D.b2CircleShape(0.2);
		bodyDef.position.x = Math.random() * 20;
		bodyDef.position.y = Math.random() * 10;
		world.CreateBody(bodyDef).CreateFixture(fixDef);
	}
});
