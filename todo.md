# Todos

Fix Number.MAX_VALUE and Number.POSITIVE_INFINITY

Find b2Bound.value setter and try to get rid of that bitwise for a !

Add `defineProperty` to the start.js file. See b2ContactID, Features, b2Color for its use. Consider switching to proper getter/setter?

Object.defineProperty is not supported in IE7, FF 3.6, Safari 4, Opera 11.50


```javascript
if(!(Object.prototype.defineProperty instanceof Function)
		&& Object.prototype.__defineGetter__ instanceof Function
		&& Object.prototype.__defineSetter__ instanceof Function)
	{
		Object.defineProperty=function(obj, p, cfg) {
			if(cfg.get instanceof Function)
				obj.__defineGetter__(p, cfg.get);
			if(cfg.set instanceof Function)
				obj.__defineSetter__(p, cfg.set);
		}
	}
```

Remove b2Distance.b2_gjkCalls and b2Distance.b2_gjkIters and b2Distance.b2_gjkMaxIters ??

Replace b2Math.Max and b2Math.Min and b2Math.Abs

See if we can remove b2PolygonShape.prototype.SetAsVector and b2PolygonShape.AsVector

Find a better solution for b2ContactManager.FindNewContacts (function.bind) ?

Move number values from constructor to prototype on b2Fixture ?

Move b2World joint and controller methods outside b2World.js to better modularize the library?

Make debugDraw colors configurable

Add b2ContactEdge null objects to the prototype ?

Clean up b2ContactFactory.Create to remove b2CircleContact.Create / b2CircleContact.Destroy methods

Remove this.m_nodeA and this.m_nodeB from b2Contact ?

Find out what b2EdgeAndCircleContact.prototype.b2CollideEdgeAndCircle should be

Make sure the while loop in b2Controller.prototype.RemoveBody was translated correctly

Double check `if (this.useDensity) {` in b2BuoyancyController.Step

Remove b2Joint.Destroy ?

Remove whenReady calls for direct object creation?

Assign classes to Box2D object



# Changes

Deleted b2Point as it was unused

Deleted b2ContactRegister as it was no longer needed

Deleted b2NullContact


# Replace

= function
: function

\}\n	b2\w+\.prototype\.
},

	|

,
,

var |
	|
