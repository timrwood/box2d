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

function update() {
	world.Step(1 / 60, 10, 10);
	world.DrawDebugData();
	world.ClearForces();
	setTimeout(update, 1000 / 60);
}
update();

function next() {
	clearAll();
	makeBounds();
	testIndex = (testIndex + 1) % (tests.length || 1);
	if (tests[testIndex]) {
		tests[testIndex](world);
	}
}
next();

document.addEventListener("mousedown", function (e) {
	var x = e.pageX / 30,
		y = e.pageY / 30,
		vec2 = new b2Vec2(x, y),
		aabb = new b2AABB(),
		body,
		jointDef;

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
}, true);

document.addEventListener("mousemove", function (e) {
	if (!mouseJoint) {
		return;
	}
	mouseJoint.SetTarget(new b2Vec2(e.pageX / 30, e.pageY / 30));
}, true);

document.addEventListener("mouseup", function () {
	if (mouseJoint) {
		world.DestroyJoint(mouseJoint);
		mouseJoint = null;
	}
}, true);

document.getElementById('next').addEventListener("click", next, true);
