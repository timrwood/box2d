# Todos

Fix Number.MAX_VALUE and Number.POSITIVE_INFINITY

Find b2Bound.value setter and try to get rid of that bitwise for a !

Add `defineProperty` to the start.js file. See b2ContactID for its use. Consider switching to proper getter/setter?

```javascript
if(!(Object.prototype.defineProperty instanceof Function)
		&& Object.prototype.__defineGetter__ instanceof Function
		&& Object.prototype.__defineSetter__ instanceof Function)
	{
		Object.defineProperty = function(obj, p, cfg) {
			if(cfg.get instanceof Function)
				obj.__defineGetter__(p, cfg.get);
			if(cfg.set instanceof Function)
				obj.__defineSetter__(p, cfg.set);
		}
	}
```

Remove b2Distance.b2_gjkCalls and b2Distance.b2_gjkIters and b2Distance.b2_gjkMaxIters ??




# Changes

Deleted b2Point as it was unused