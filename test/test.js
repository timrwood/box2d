(function(){
var b2Vec2 = Box2D.b2Vec2,
	b2AABB = Box2D.b2AABB,
	b2BodyDef = Box2D.b2BodyDef,
	b2Body = Box2D.b2Body,
	b2FixtureDef = Box2D.b2FixtureDef,
	b2Fixture = Box2D.b2Fixture,
	b2World = Box2D.b2World,
	b2MassData = Box2D.b2MassData,
	b2PolygonShape = Box2D.b2PolygonShape,
	b2CircleShape = Box2D.b2CircleShape,
	b2DebugDraw = Box2D.b2DebugDraw,
	b2DebugDrawCanvas = Box2D.b2DebugDrawCanvas,
	b2MouseJointDef = Box2D.b2MouseJointDef,

	world = new b2World(new b2Vec2(0, 10), true),
	debugDraw = new b2DebugDrawCanvas(),
	canvas = document.getElementById("canvas"),
	ctx = canvas.getContext('2d'),

	tests = [],
	testIndex = -1,

	mouseJoint;

debugDraw.SetCanvas(canvas);
debugDraw.SetDrawScale(30);
debugDraw.SetFillAlpha(0.5);
world.SetDebugDraw(debugDraw);

function makeBounds() {
	var fixDef = new b2FixtureDef(),
		bodyDef = new b2BodyDef();

	bodyDef.type = b2Body.b2_staticBody;

	fixDef.shape = new b2PolygonShape();

	fixDef.shape.SetAsBox(10, 2);
	bodyDef.position.Set(10, 11.8);
	world.CreateBody(bodyDef).CreateFixture(fixDef);
	bodyDef.position.Set(10, -1.8);
	world.CreateBody(bodyDef).CreateFixture(fixDef);

	fixDef.shape.SetAsBox(2, 5);
	bodyDef.position.Set(-1.8, 5);
	world.CreateBody(bodyDef).CreateFixture(fixDef);
	bodyDef.position.Set(21.8, 5);
	world.CreateBody(bodyDef).CreateFixture(fixDef);
}

function clearAll() {
	var body = world.GetBodyList(),
		joint = world.GetJointList();

	while (body) {
		world.DestroyBody(body);
		body = world.GetBodyList();
	}

	while (joint) {
		world.DestroyJoint(joint);
		joint = world.GetJointList();
	}
}

$(window).resize(function(){
	canvas.width = $(window).width();
	canvas.height = canvas.width / 2;
	debugDraw.SetDrawScale(canvas.width / 20);
	restartCurrent();
}).resize();

function update() {
	world.Step(1 / 60, 10, 10);
	world.DrawDebugData();
	world.ClearForces();
	setTimeout(update, 1000 / 60);
}
update();

function restartCurrent() {
	clearAll();
	makeBounds();
	if (tests[testIndex]) {
		tests[testIndex](world);
	}
}

function next() {
	testIndex = (testIndex + 1) % (tests.length || 1);
	restartCurrent();
}

$(document).on('mousedown', function (e) {
	var scale = debugDraw.GetDrawScale(),
		x = e.pageX / scale,
		y = e.pageY / scale,
		vec2 = new b2Vec2(x, y),
		aabb = new b2AABB(),
		body,
		jointDef;

	console.log(scale);

	aabb.lowerBound.Set(x - 0.001, y - 0.001);
	aabb.upperBound.Set(x + 0.001, y + 0.001);
	world.QueryAABB(function (fixture) {
		if (fixture.GetBody().GetType() !== b2Body.b2_staticBody &&
			fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), vec2)) {
			body = fixture.GetBody();
			return false;
		}

		return true;
	}, aabb);

	if (body) {
		jointDef = new b2MouseJointDef();
		jointDef.bodyA = world.GetGroundBody();
		jointDef.bodyB = body;
		jointDef.target.Set(x, y);
		jointDef.collideConnected = true;
		jointDef.maxForce = 3000 * body.GetMass();
		mouseJoint = world.CreateJoint(jointDef);
		body.SetAwake(true);
	}
});

$(document).on('mousemove', function (e) {
	if (!mouseJoint) {
		return;
	}
	var scale = debugDraw.GetDrawScale();
	mouseJoint.SetTarget(new b2Vec2(e.pageX / scale, e.pageY / scale));
});

$(document).on('mouseup', function () {
	if (mouseJoint) {
		world.DestroyJoint(mouseJoint);
		mouseJoint = null;
	}
});

document.getElementById('next').addEventListener("click", next, true);

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

tests.push(function(world, width, height){
	var fixDef = new Box2D.b2FixtureDef(),
		bodyDef = new Box2D.b2BodyDef(),
		i;

	fixDef.density = 1;
	fixDef.friction = 0.5;
	fixDef.restitution = 0.2;

	bodyDef.type = Box2D.b2Body.b2_dynamicBody;

	for (i = 0; i < 20; i++) {
		fixDef.shape = new Box2D.b2CircleShape(0.1 + Math.random());
		bodyDef.position.x = Math.random() * 20;
		bodyDef.position.y = Math.random() * 10;
		bodyDef.angle = Math.random() * Math.PI;
		world.CreateBody(bodyDef).CreateFixture(fixDef);
	}
});

tests.push(function(world){
	var fixDef = new Box2D.b2FixtureDef(),
		bodyDef = new Box2D.b2BodyDef(),
		i;

	fixDef.friction = 0.5;
	fixDef.restitution = 0.9;

	bodyDef.type = Box2D.b2Body.b2_dynamicBody;

	for (i = 0; i < 200; i++) {
		fixDef.shape = new Box2D.b2CircleShape(0.2);
		bodyDef.position.x = Math.random() * 20;
		bodyDef.position.y = Math.random() * 10;
		world.CreateBody(bodyDef).CreateFixture(fixDef);
	}
});

next();

}());