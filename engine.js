"use strict";

const pi = Math.PI;
const canvas = document.getElementById("canv");
const ctx = canv.getContext("2d");
canv.width = window.innerWidth;
canv.height = window.innerHeight;

var phys = {
	canvas: canvas,
	ctx: ctx,
	World: {
		gravity: {
			x: 0,
			y: 0,
		},
		bodies: [],
		lights: [],
		timescale: 1,
	},
	Collision: {
		pairs: [],
		simplifiedPairs: [],
		canCollide: function(body1, body2) {
			let filter1 = body1.collisionFilter;
			let filter2 = body2.collisionFilter;
			if (filter1.func) {
				return filter1.func(body2);
			}
			if (filter2.func) {
				return filter2.func(body1);
			}

			if (filter1.category === 0 || filter2.category === 0) return true;
			return (filter1.mask & filter2.category) !== 0;
		},
		test: function() {
			// Trigger event
			phys.Collision.trigger("beforeUpdate");

			//
			// Get collisions
			//
			const { normalize } = phys.Vector;
			const timescale = phys.World.timescale;
			const canCollide = phys.Collision.canCollide;
		
			// Go through each shape
			let allBodies = World.bodies;
			for (let i = allBodies.length - 1; i >= 0; i--) {
				const body1 = allBodies[i];

				if (body1.removed) {
					phys.Broadphase.remove(body1);
					allBodies.splice(i, 1);
					continue;
				}

				if (!body1.isStatic) {
					if (Math.abs(body1.velocity.x) < 0.01) body1.velocity.x = 0;
					if (Math.abs(body1.velocity.y) < 0.01) body1.velocity.y = 0;
					if (Math.abs(body1.angularVelocity) < 0.005) body1.angularVelocity = 0;

					if (!body1.ignoreGravity) {
						body1.velocity.x += phys.World.gravity.x;
						body1.velocity.y += phys.World.gravity.y;
					}

					if (body1.velocity.x !== 0) {
						body1.velocity.x *= 1 - body1.frictionAir * timescale;
					}
					if (body1.velocity.y !== 0) {
						body1.velocity.y *= 1 - body1.frictionAir * timescale;
					}
					if (body1.angularVelocity !== 0) {
						body1.angularVelocity *= 1 - body1.frictionAir * timescale;
						body1.setAngle(body1.angle + body1.angularVelocity);
					}

					body1.position.x += body1.velocity.x * timescale;
					body1.position.y += body1.velocity.y * timescale;
					phys.Broadphase.remove(body1);
					phys.Broadphase.add(body1);
				}
				else {
					body1.velocity.x = 0;
					body1.velocity.y = 0;
				}

				if ((body1.position.x !== body1.last.position.x || body1.position.y !== body1.last.position.y || body1.angle !== body1.last.angle) && body1.hasCollisions !== false) { // Sleeping
					body1.last.position = { ...body1.position };
					body1.last.angle = body1.angle;
		
					// - Get pairs
					for (let j = 0; j < body1.buckets.length; j++) { // ~ Look through each bucket body is part of
						let bucket = phys.Broadphase.grid[body1.buckets[j].x][body1.buckets[j].y];

						for (let i = 0; i < bucket.length; i++) { // ~ Look through all bodies in that bucket
							const body2 = bucket[i];

							if (!body2.isLine && body2.id !== body1.id) { // Check if it isn't itself
								if (!body2.removed && body2.hasCollisions !== false && canCollide(body1, body2) && !body2.checkedPairs.includes(body1.id)) { // Check if not already in pairs list
									body1.checkedPairs.push(body2.id);

									// Do AABB collision test
									const bounds1 = body1.bounds;
									const bounds2 = body2.bounds;
									const AABBCollision = !(bounds1.min.x + body1.position.x > bounds2.max.x + body2.position.x ||
															bounds1.max.x + body1.position.x < bounds2.min.x + body2.position.x ||
															bounds1.min.y + body1.position.y > bounds2.max.y + body2.position.y ||
															bounds1.max.y + body1.position.y < bounds2.min.y + body2.position.y);
								
									if (AABBCollision) {
										let penetration = Infinity; // Penetration amount
										let edge1 = []; // 2 points represent the collision edge for body1
										let edge2 = []; // 2 points represent the collision edge for body2
										let collisionPoint;
	
										function checkCollisions(body1, body2, isBody1) {
											let curPenetration = Infinity;
											let collision = true;
											let vertices = body1.vertices;
											let len = vertices.length;
	
	
											function project(body, normal) {
												let bounds = {
													min: { dp:  Infinity,  x: 0, y: 0 },
													max: { dp: -Infinity, x: 0, y: 0 },
												}
												for (let i = body.vertices.length; i--;) {
													const cV = body.vertices[i];
													const vertice = { x: cV.x + body.position.x, y: cV.y + body.position.y };
													const dp = normal.x * vertice.x + normal.y * vertice.y;
	
													if (dp > bounds.max.dp) {
														bounds.max.dp = dp;
														bounds.max.x = vertice.x;
														bounds.max.y = vertice.y;
													}
													if (dp < bounds.min.dp) {
														bounds.min.dp = dp;
														bounds.min.x = vertice.x;
														bounds.min.y = vertice.y;
													}
												}
	
												return bounds;
											}
	
											for (let verticeNum = len; verticeNum--;) {
												let vertice = vertices[verticeNum];
												let nextVertice = vertices[(verticeNum + 1) % len];
												let vector = { x: nextVertice.x - vertice.x, y: nextVertice.y - vertice.y };
												let normal = normalize({ x: -vector.y, y: vector.x });
	
	
												// Get min / max of both bodies
												let bounds1 = project(body1, normal);
												let bounds2 = project(body2, normal);
												
												if (bounds1.min.dp > bounds2.max.dp || bounds1.max.dp < bounds2.min.dp) {
													collision = false;
													break;
												}
												else {
													let pen;
													if (bounds1.min.dp > bounds2.max.dp) {
														pen = bounds1.min.dp - bounds2.max.dp;
													}
													else {
														pen = bounds1.max.dp - bounds2.min.dp;
													}
													if (pen < penetration) {
														penetration = pen;
													}
													if (pen < curPenetration) {
														curPenetration = pen;
	
														if (isBody1) {
															edge1[0] = vertice;
															edge1[1] = nextVertice;
														}
														else {
															edge2[0] = vertice;
															edge2[1] = nextVertice;
														}
													}
												}
											}
	
											return collision;
										}
									
										let test1 = checkCollisions(body1, body2, true);
										if (!test1) continue;
										let test2 = checkCollisions(body2, body1);
									
										if (test1 && test2) {
											if (body1.collisions === undefined) body1.collisions = [];
											if (body2.collisions === undefined) body2.collisions = [];
										
											if (!body1.collisions.includes(body2.id)) {
												body1.collisions.push(body2.id);
											}
											if (!body2.collisions.includes(body1.id)) {
												body2.collisions.push(body1.id);
											}
	
	
											//
											// Get supports + solve
											//
											let normalVector = normalize({ x: -edge1[1].y + edge1[0].y, y: edge1[1].x - edge1[0].x });
	
											// Solve physics
											let collision = {
												penetration: penetration,
												point: collisionPoint,
												normal: normalVector,
	
												edge1: edge1,
												edge2: edge2,
	
												body1: body1,
												body2: body2,
											};
											phys.Collision.solve(collision);
	
											if (!body1.lastCollisions.includes(body2.id)) {
												body1.trigger("collisionStart", body2, collision);
											}
											if (!body1.lastCollisions.includes(body1.id)) {
												body1.trigger("collisionStart", body1, collision);
											}
											body1.trigger("collision", body2, collision);
											body2.trigger("collision", body1, collision);
										}
									}
								}
							}
						}
					}

					body1.checkedPairs.length = 0;
				}

				body1.lastCollisions = [ ...body1.collisions ];
				body1.collisions.length = 0;
			}

			
			phys.Collision.trigger("afterUpdate");
		},
		solve: function(collision) { // Solves just for body1
			const { dotProduct:dot, mult, sub, add, normalize } = phys.Vector;
			const timescale = phys.World.timescale;

			const { penetration, body1, body2, edge1, edge2, point } = collision;
			let normal;
			if (body1.isSensor || body2.isSensor) return;

			if (body2.isStatic) {
				normal = normalize({ x: -(edge2[0].y - edge2[1].y), y: edge2[0].x - edge2[1].x });
			}
			else {
				normal = collision.normal;
			}
			
			const { velocity:vel1, mass:mass1, restitution:restitution1 } = body1;
			const { velocity:vel2, mass:mass2, restitution:restitution2 } = body2;
			const relativeVel = { x: vel2.x - vel1.x, y: vel2.y - vel1.y };
			const totalMass = mass1 + mass2;

			let velShare1 = (mass2 / totalMass) || 0;
			let velShare2 = (mass1 / totalMass) || 0;

			if (mass1 === Infinity || body1.isStatic) {
				velShare2 = 0.7;
			}
			else if (mass2 === Infinity || body2.isStatic) {
				velShare1 = 0.7;
			}

			// let torque1 = 1 - dotProduct(normalize({ x: body2.position.x - point.x, y: body2.position.y - point.y }), normal);
			// body1.applyAngularForce(torque1 / 10);

			let dotProd = dot(relativeVel, normal);

			if (dotProd < 0) {
				let r1 = mult(normal, dotProd * 2 * velShare1);
	
				body1.applyForce(mult(r1, velShare1 * (1 + restitution1)));
				if (!body2.isStatic) body2.applyForce(mult(r1, -velShare2 * (1 + restitution2)));
			}

			if (!body2.isStatic) {
				body1.setPosition({ x: body1.position.x + normal.x * penetration *-1, y: body1.position.y + normal.y * penetration *-1 });
				body2.setPosition({ x: body2.position.x + normal.x * penetration * 1, y: body2.position.y + normal.y * penetration * 1 });
			}
			else if (!body1.isStatic) {
				body1.setPosition({ x: body1.position.x - normal.x * penetration, y: body1.position.y - normal.y * penetration });
			}

			// body1.applyForce(mult(normal, -penetration * 10));
			// body2.applyForce(mult(normal,  penetration * 10));
			
			/*
			if (body2.isStatic) {
				body1.setPosition({ x: body1.position.x + -normal.x * penetration / 100, y: body1.position.y + -normal.y * penetration / 100 });
			}/* */
		},

		
		// Events
		events: {
			beforeUpdate: [],
			afterUpdate: [],
		},
		on: function(event, callback) {
			if (!Render.events[event]) Render.events[event] = [];

			if (Render.events[event]) {
				Render.events[event].push(callback);
			}
			else {
				console.warn(event + " is not a valid event");
			}
		},
		off: function(event, callback) {
			event = Render.events[event];
			event.splice(event.indexOf(callback), 1);
		},
		trigger: function(event) {
			// Trigger each event
			if (Render.events[event] && Render.events[event].length > 0) {
				Render.events[event].forEach(callback => {
					callback();
				});
			}
		},
	},
	Common: {
		// Basic stuff I should really be putting in a vector class
		addPt: function(point1, point2) {
			if (typeof point2 === "number") point2 = { x: point2, y: point2 };
		
			point1 = { ...point1 };
			point2 = { ...point2 };
		
			point1.x += point2.x;
			point1.y += point2.y;
		
			return point1;
		},
		subPt: function(point1, point2) {
			if (typeof point2 === "number") point2 = { x: point2, y: point2 };
		
			point1 = { ...point1 };
			point2 = { ...point2 };
		
			point1.x -= point2.x;
			point1.y -= point2.y;
		
			return point1;
		},
		multPt: function(point1, point2) {
			point1 = { ...point1 };
		
			if (typeof point1 === "number") {
				point1 = { ...point1 };
				point2.x *= point1;
				point2.y *= point1;

				return point2;
			}
			if (typeof point2 === "object") {
				point2 = { ...point2 };
				point1.x *= point2.x;
				point1.y *= point2.y;
			}
			else {
				point1.x *= point2;
				point1.y *= point2;
			}
		
			return point1;
		},
		powerPt: function(point1, point2) {
			point1 = { ...point1 };
		
			if (typeof point2 === "object") {
				point2 = { ...point2 };
				point1.x = Math.pow(point1.x, point2.x);
				point1.y = Math.pow(point1.y, point2.y);
			}
			else {
				point1.x = Math.pow(point1.x, point2);
				point1.y = Math.pow(point1.y, point2);
			}
		
			return point1;
		},
		divPt: function(point1, point2) {
			point1 = { ...point1 };
		
			if (typeof point2 === "object") {
				point2 = { ...point2 };
				point1.x /= point2.x;
				point1.y /= point2.y;
			}
			else {
				point1.x /= point2;
				point1.y /= point2;
			}
		
			return point1;
		},
		roundPt: function(point) {
			return { x: Math.round(point.x), y: Math.round(point.y) };
		},
		roundToPlace: function(number, place) {
			return Math.round(number * (10**place)) / (10**place);
		},
		floorPt: function(point) {
			return { x: Math.floor(point.x), y: Math.floor(point.y) };
		},
		ceilPt: function(point) {
			return { x: Math.ceil(point.x), y: Math.ceil(point.y) };
		},
		absPt: function(point) {
			return { x: Math.abs(point.x), y: Math.abs(point.y) };
		},
		normalizeAngle: function(angle) {
			return (angle - Math.floor(angle / (pi*2)) * pi*2) % (pi*2);
		},
		angleDifference: function(angle1, angle2) {
			let diff = (angle1 - angle2 + pi) % (pi*2) - pi;
			
			if (Math.abs(Math.abs(diff) - Math.PI) < Math.abs(diff)) {
				diff = Math.abs(Math.abs(diff) - Math.PI) * Math.sign(diff);
			}
			return diff;
		},
		midPt: function(vertices) {
			let total = { x: 0, y: 0 };

			for (let i = 0; i < vertices.length; i++) {
				total.x += vertices[i].x;
				total.y += vertices[i].y;
			}

			total.x /= vertices.length;
			total.y /= vertices.length;

			return total;
		},
		normalizeVertices: function(vertices, position) {
			const { subPt, midPt } = phys.Common;
			const midpt = midPt(vertices);

			vertices = vertices.map(vertice => {
				return subPt(vertice, midpt);
			});

			return [vertices, midpt];
		},
		avg: function(x1, x2, weight = 0.5) {
			return x1*weight + x2*(1-weight);
		},
		avgPt: function(point1, point2, weight = 0.5) {
			return {
				x: point1.x*weight + point2.x*(1-weight),
				y: point1.y*weight + point2.y*(1-weight)
			};
		},
		minPt: function(point1, point2) {
			if (this.hypot(point1.x, point1.y) < this.hypot(point2.x, point2.y)) {
				return point1;
			}
			else {
				return point2;
			}
		},
		maxPt: function(point1, point2) {
			if (this.hypot(point1.x, point1.y) < this.hypot(point2.x, point2.y)) {
				return point2;
			}
			else {
				return point1;
			}
		},

		// Trig
		hypot: function(x1, x2) {
			if (typeof x1 === "object" && typeof x2 === "object") {
				return Math.sqrt((x1.x - x2.x) ** 2 + (x1.y - x2.y) ** 2);
			}
			else if (typeof x1 === "object") {
				return Math.sqrt(x1.x ** 2 + x1.y ** 2);
			}
			else {
				return Math.sqrt(x1 ** 2 + x2 ** 2);
			}
		},
		angle: function(point1, point2) {
			if (!point2) {
				return Math.atan2(point1.x, point1.y);
			}
			return Math.atan2(point1.x - point2.x, point1.y - point2.y);
		},

		// Other
		rotatePt: function(point, angle) {
			const sin = Math.sin(angle);
			const cos = Math.cos(angle);

			return {
				x: point.x*cos - point.y*sin,
				y: point.x*sin + point.y*cos
			};
		},

		mergeObj: function(defaults, obj2 = {}) { // Essentially a clone function, except it merges defaults and obj2 together so that properties in obj2 are prioritized over defaults
			let finalObj = { ...obj2 };

			Object.keys(defaults).forEach(key => {
				if (typeof obj2[key] === "object" && !Array.isArray(obj2[key])) {
					finalObj[key] = phys.Common.mergeObj(defaults[key], obj2[key]);
				}
				else if (typeof defaults[key] === "object" && !Array.isArray(defaults[key])) {
					finalObj[key] = phys.Common.mergeObj(defaults[key]);
				}
				else {
					finalObj[key] = obj2[key] ?? defaults[key];
				}
			});

			return finalObj;
		},
		updateBodyThis: function(obj) { // Don't worry about it...
			Object.values(obj).forEach(val => {
				if (typeof val === "function") {
					val.bind(obj);
				}
			});
		}
	},
	Vector: {
		dotProduct: function(vector1, vector2) {
			return vector1.x*vector2.x + vector1.y*vector2.y;
		},
		crossProduct: function(vector1, vector2) {
			return vector1.x*vector2.x - vector1.y*vector2.y;
		},
		normalize: function(vector) {
			if (vector.x !== 0 || vector.y !== 0) {
				let size = Math.sqrt(vector.x * vector.x + vector.y * vector.y);
				return { x: vector.x / size, y: vector.y / size };
			}
			else {
				return vector;
			}
		},

		
		add: function(point1, point2) {
			if (typeof point2 === "number") point2 = { x: point2, y: point2 };
		
			point1 = { ...point1 };
			point2 = { ...point2 };
		
			point1.x += point2.x;
			point1.y += point2.y;
		
			return point1;
		},
		sub: function(point1, point2) {
			if (typeof point2 === "number") point2 = { x: point2, y: point2 };
		
			point1 = { ...point1 };
			point2 = { ...point2 };
		
			point1.x -= point2.x;
			point1.y -= point2.y;
		
			return point1;
		},
		mult: function(point1, point2) {
			point1 = { ...point1 };
		
			if (typeof point1 === "number") {
				point1 = { ...point1 };
				point2.x *= point1;
				point2.y *= point1;

				return point2;
			}
			if (typeof point2 === "object") {
				point2 = { ...point2 };
				point1.x *= point2.x;
				point1.y *= point2.y;
			}
			else {
				point1.x *= point2;
				point1.y *= point2;
			}
		
			return point1;
		},
	},
	Broadphase: {
		grid: [],
		bucketSize: 800,
		add: function(body) {
			if (body.hasCollisions) {
				const { Broadphase } = phys;
				const { bucketSize } = Broadphase;
				const position = body.position;

				function addToBucket(position) {
					if (Broadphase.grid[position.x] === undefined) {
						Broadphase.grid[position.x] = [];
					}
					if (Broadphase.grid[position.x][position.y] === undefined) {
						Broadphase.grid[position.x][position.y] = [];
					}
					
					body.buckets.push(position);
					let bucket = Broadphase.grid[position.x][position.y];
					bucket[bucket.length] = body;
				}
				function getBucketPosition(vPosition) {
					return { x: Math.floor((vPosition.x + position.x) / bucketSize), y: Math.floor((vPosition.y + position.y) / bucketSize) };
				}

				let minBucket = getBucketPosition(body.bounds.min);
				let maxBucket = getBucketPosition(body.bounds.max);

				// Fill rectangle between min / max buckets
				for (let w = 0; w <= maxBucket.x - minBucket.x; w++) {
					for (let h = 0; h <= maxBucket.y - minBucket.y; h++) {
						addToBucket({ x: w + minBucket.x, y: h + minBucket.y });
					}
				}
			}
		},
		remove: function(body) {
			if (body.buckets.length > 0) {
				for (let i = body.buckets.length; i--;) {
					let bucketPos = body.buckets[i];
					let bucket = phys.Broadphase.grid[bucketPos.x][bucketPos.y];
					let index = bucket.indexOf(body);

					if (index >= 0) {
						bucket.splice(index, 1);
						body.buckets.length--;
					}
				}
			}
		}
	},
	Render: (() => {
		let Render = function() {
			const { canvas, ctx, World } = phys;
		
			ctx.clearRect(0, 0, canvas.width, canvas.height);

			const camera = phys.Render.camera;
			const { position:cameraPosition, fov:FoV } = camera;
			const canvWidth = canvas.width;
			const canvHeight = canvas.height;
			const boundSize = (canvWidth + canvHeight) / 2;

			camera.translation = { x: -cameraPosition.x * boundSize/FoV + canvWidth/2, y: -cameraPosition.y * boundSize/FoV + canvHeight/2 };
			camera.scale = boundSize / FoV;

			Render.trigger("beforeSave");

			ctx.save();
			ctx.translate(camera.translation.x, camera.translation.y);
			ctx.scale(camera.scale, camera.scale);
			
			Render.trigger("beforeRender");

			if (!phys.Render.postRenderLights) phys.Render.lights();


			for (let i = 0; i < World.bodies.length; i++) {
				const body = World.bodies[i];
				const { position, angle, vertices, render } = body;
				const { visible, background, tempBackground, border, borderWidth, borderType, lineDash, bloom } = render;

				if (visible) {
					ctx.lineWidth = borderWidth;
					ctx.strokeStyle = border;
					ctx.fillStyle = background;
					ctx.lineJoin = borderType;

					if (bloom) {
						ctx.shadowColor = border;
						ctx.shadowBlur = bloom;
					}
		
					if (tempBackground !== undefined) {
						ctx.fillStyle = tempBackground;
		
						if (body.collisions.length === 0) {
							delete render.tempBackground;
						}
					};
		
					if (lineDash) {
						ctx.setLineDash(lineDash);
					}
					else {
						ctx.setLineDash([]);
					}
		
					ctx.beginPath();

					if (body.type === "circle") { // circleRender
						ctx.arc(position.x, position.y, body.radius, 0, Math.PI*2);
					}
					else {
						ctx.moveTo(vertices[0].x + position.x, vertices[0].y + position.y);
	
						for (let index = 0; index < vertices.length; index++) {
							if (index > 0) {
								let vertice = vertices[index];
								ctx.lineTo(vertice.x + position.x, vertice.y + position.y);
							}
						}
			
						ctx.closePath();
					}
					if (ctx.fillStyle && ctx.fillStyle !== "transparent") ctx.fill();
					if (ctx.strokeStyle && ctx.strokeStyle !== "transparent") ctx.stroke();
					
					if (bloom) {
						ctx.shadowColor = "rgba(0, 0, 0, 0)";
						ctx.shadowBlur = bloom;
					}
				}
			}

			if (phys.Render.postRenderLights) phys.Render.lights();

			if (Render.showBroadphase === true) {
				Render.broadphase();
			}
			if (Render.showBoundingBox === true) {
				Render.boundingBox();
			}


			Render.trigger("afterRender");

			ctx.restore();

			Render.trigger("afterRestore");
		}
		Render.lights = function() {
			const allLights = phys.World.lights;
			const allBodies = phys.World.bodies;
			const dot = phys.Vector.dotProduct;
			const normalize = phys.Vector.normalize;

			let path = new Path2D();
			ctx.beginPath();
			
			for (let i = allLights.length - 1; i >= 0; i--) {
				let light = allLights[i];
				let { position, size, background } = light;

				if (size <= 0) continue;

				if (background !== "transparent") {
					// Render light
					ctx.beginPath();
					ctx.fillStyle = background;
					ctx.arc(position.x, position.y, size, 0, Math.PI * 2);
					ctx.fill();
				}


				// Render shadows
				for (let i = 0; i < allBodies.length; i++) {
					const body = allBodies[i];
					const { vertices, position:bodyPos, ignoreLights } = body;

					if (ignoreLights) continue;

					let minAngle = Infinity;
					let maxAngle = -Infinity;
					let min = Infinity;
					let max = -Infinity;
					let minPt;
					let maxPt;

					let testVec = normalize({ x: bodyPos.y - position.y, y: position.x - bodyPos.x });

					if (body.type === "circle") {
						minPt = { x: bodyPos.x - body.radius * testVec.x, y: bodyPos.y - body.radius * testVec.y };
						maxPt = { x: bodyPos.x + body.radius * testVec.x, y: bodyPos.y + body.radius * testVec.y };
					}
					else {
						for (let j = 0; j < vertices.length; j++) {
							const vertex = vertices[j];
							const dp = dot(testVec, vertex);
	
							if (dp < min) {
								min = dp;
								minPt = { x: vertex.x + bodyPos.x, y: vertex.y + bodyPos.y };
							}
							if (dp > max) {
								max = dp;
								maxPt = { x: vertex.x + bodyPos.x, y: vertex.y + bodyPos.y };
							}
						}
					}

					minAngle = (Math.atan2(position.y - minPt.y, position.x - minPt.x) + Math.PI*3) % (Math.PI*2);
					maxAngle = (Math.atan2(position.y - maxPt.y, position.x - maxPt.x) + Math.PI*3) % (Math.PI*2);
					let shadowLenA = phys.Render.camera.fov*1.5;
					let shadowLenB = shadowLenA;

					path.moveTo(minPt.x, minPt.y);
					path.lineTo(maxPt.x, maxPt.y);
					path.lineTo(maxPt.x + Math.cos(maxAngle) * shadowLenA, maxPt.y + Math.sin(maxAngle) * shadowLenA);
					path.lineTo(minPt.x + Math.cos(minAngle) * shadowLenB, minPt.y + Math.sin(minAngle) * shadowLenB);
					path.closePath();
				}
			}
			// let opacity = Math.ceil(40 / (allLights.length ** 0.01)).toString(16);
			// if (opacity.length === 1) opacity += "0";
			ctx.fillStyle = "#00000040";
			ctx.fill(path, "nonzero");
		}
		Render.postRenderLights = false;

		// Camera
		Render.camera = {
			position: { x: 0, y: 0 },
			fov: 2000,
			translation: { x: 0, y: 0 },
			scale: 1,
		}

		// Broadphase
		Render.showBroadphase = false;
		Render.showBoundingBox = false;
		Render.broadphase = function() {
			const Broadphase = phys.Broadphase;
			const { grid, bucketSize } = Broadphase;
			
			Object.keys(grid).forEach(x => {
				Object.keys(grid[x]).forEach(y => {
					let len = grid[x][y].length;
					if (len > 0) {
						ctx.strokeStyle = "#FFAF33" + (len * 51).toString(16);
						ctx.strokeRect(bucketSize * x, bucketSize * y, bucketSize, bucketSize);
					}
				});
			});
		}
		Render.boundingBox = function() {
			let allBodies = World.bodies;

			ctx.strokeStyle = "#ffffff80";
			ctx.lineWidth = 2;

			for (let i = 0; i < allBodies.length; i++) {
				let body = allBodies[i];
				let bounds = body.bounds;
				let width = bounds.max.x - bounds.min.x;
				let height = bounds.max.y - bounds.min.y;

				ctx.beginPath();
				ctx.strokeRect(body.position.x + bounds.min.x, body.position.y + bounds.min.y, width, height);
			}
		}

		// Events
		Render.events = {
			beforeRender: [],
			afterRender: [],
			afterRestore: [],
			beforeSave: [],
		}
		Render.on = function(event, callback) {
			if (Render.events[event]) {
				Render.events[event].push(callback);
			}
			else {
				console.warn(event + " is not a valid event");
			}
		}
		Render.off = function(event, callback) {
			event = Render.events[event];
			event.splice(event.indexOf(callback), 1);
		}
		Render.trigger = function(event) {
			// Trigger each event
			Render.events[event].forEach(callback => {
				callback();
			});
		}
		
		return Render;
	})(),
	Bodies: {
		bodies: 0,
		get uniqueId() {
			this.bodies++;
			return this.bodies;
		},
		get defaults() {
			return {
				velocity: { x: 0, y: 0 },
				buckets: [],
				angle: 0,
				frictionAir: 0.1,
				isStatic: false,
				isSensor: false,
				hasCollisions: true,
				restitution: 0.4,
				angularVelocity: 0,
				torque: 1,
				mass: 1,
				checkedPairs: [],
				collisions: [],
				lastCollisions: [],
				bounds: {
					min: { x: 0, y: 0 },
					max: { x: 1, y: 1 }
				},
				render: {
					background: "transparent",
					border: "#EC2E2E",
					borderWidth: 3,
					borderType: "round",
					lineDash: false,
					visible: true,
				},
				collisionFilter: {
					category: 0,
					mask: 0,
				},

				delete: function() {
					let World = phys.World;
					if (World.bodies.includes(this)) {
						this.trigger("delete");
						this.removed = true;
					}

					return this;
				},
				add: function() {
					if (!World.bodies.includes(this)) {
						this.removed = false;
						World.bodies.push(this);
					}

					return this;
				},
				setAngle: function(angle) {
					if (angle !== this.angle) {
						const rotatePt = phys.Common.rotatePt;
						const vertices = this.vertices;

						for (let i = vertices.length; i-- > 0;) {
							vertices[i] = rotatePt(vertices[i], this.angle - angle);
						}

						this.angle = angle;
						this.updateBounds();
					}

					return this;
				},
				setPosition: function(position) {
					if (position.x !== this.position.x || position.y !== this.position.y) {
						this.position.x = position.x;
						this.position.y = position.y;
					}
				},
				updateBounds: function() {
					const vertices = this.vertices;
					let minX = Infinity, minY = Infinity;
					let maxX = -Infinity, maxY = -Infinity;

					for (let i = 0; i < vertices.length; i++) {
						let v = vertices[i];

						if (v.x < minX) minX = v.x;
						if (v.y < minY) minY = v.y;
						if (v.x > maxX) maxX = v.x;
						if (v.y > maxY) maxY = v.y;
					}

					this.bounds = {
						min: { x: minX, y: minY },
						max: { x: maxX, y: maxY },
					};

					phys.Broadphase.remove(this);
					phys.Broadphase.add(this);

					return this;
				},
				applyForce: function(force) {
					this.velocity.x += force.x;
					this.velocity.y += force.y;

					return this;
				},
				applyAngularForce: function(force) {
					this.angularVelocity += force;
					
					return this;
				},

				events: {
					collision: [],
					collisionStart: [],
					delete: [],
				},
				on: function(event, callback) {
					if (!this.events[event]) {
						this.events[event] = [];
					}
					this.events[event].push(callback);
				},
				off: function(event, callback) {
					event = this.events[event];
					event.splice(event.indexOf(callback), 1);
				},
				trigger: function(event, arg1, arg2) {
					// Get event's arguments
					/*
					let args = Array.from(arguments);
					// Remove event name from arguments
					args.shift();*/
					
					// Trigger each event
					this.events[event].forEach(callback => {
						// callback(...args);
						callback(arg1, arg2);
					});

					return this;
				}
			};
		},
		rectangle: function(width, height, x, y, options = {}) {
			let body = phys.Common.mergeObj(phys.Bodies.defaults, options);
			
			body.type = "rectangle";
			body.id = this.uniqueId;
			body.position = { x: x, y: y };
			body.last = {
				angle: 0,
				position: { x: x, y: y },
				velocity: { x: 0, y: 0 },
			};

			body.width = width;
			body.height = height;
			body.vertices = [{ x: -width/2, y: -height/2 }, { x: width/2, y: -height/2 }, { x: width/2, y: height/2 }, { x: -width/2, y: height/2 }];

			phys.Common.updateBodyThis(body);
			body.updateBounds();
			
			if (options.angle) {
				body.angle = 0;
				body.setAngle(options.absangle);
			}

			phys.World.bodies.push(body);

			return body;
		},
		circle: function(radius, x, y, options={}) {
			let body = phys.Common.mergeObj(phys.Bodies.defaults, options);
			
			body.type = "circle";
			body.id = this.uniqueId;
			body.position = { x: x, y: y };
			body.last = {
				angle: 0,
				position: { x: x, y: y },
				velocity: { x: 0, y: 0 },
			};

			body.radius = options.radius ?? radius;

			let vertices = [];
			if (vertices.length === 0) {
				let numVerts = options.maxVertices ?? Math.max(3, Math.round(body.radius / 10));
				let a = Math.PI * 2 / numVerts;
				for (let i = numVerts; i > 0; i--) {
					vertices.push({ x: Math.cos(a * i) * radius, y: Math.sin(a * i) * radius });
				}
			}
			body.vertices = vertices;

			phys.Common.updateBodyThis(body);
			body.updateBounds();

			if (options.angle) {
				body.angle = 0;
				body.setAngle(options.absangle);
			}

			phys.World.bodies.push(body);

			return body;
		},
		polygon: function(vertices, x, y, options={}) {
			vertices = phys.Common.normalizeVertices(vertices);
			vertices = vertices[0];

			let body = phys.Common.mergeObj(phys.Bodies.defaults, options);
			
			body.type = "polygon";
			body.id = this.uniqueId;
			body.position = { x: x, y: y };
			body.last = {
				angle: 0,
				position: { x: x, y: y },
				velocity: { x: 0, y: 0 },
			};

			body.vertices = vertices;

			phys.Common.updateBodyThis(body);
			body.updateBounds();
			
			if (options.angle) {
				body.angle = 0;
				body.setAngle(options.absangle);
			}

			phys.World.bodies.push(body);

			return body;
		}
	},
	Lights: {
		create: function(x, y, options={}) {
			let light = {
				position: { x: x, y: y },
				size: options.size ?? 100,
				background: options.background ?? "transparent",
			}
			phys.World.lights.push(light);

			return light;
		}
	},
}

phys.Render.path = function(path, options={}) {
	const { background, border, borderWidth, borderType, lineDash } = options;

	ctx.lineWidth = borderWidth;
	ctx.strokeStyle = border;
	ctx.fillStyle = background;
	ctx.lineJoin = borderType;

	if (lineDash) {
		ctx.setLineDash(lineDash);
	}
	else {
		ctx.setLineDash([]);
	}

	ctx.beginPath();
	ctx.moveTo(path[0].x, path[0].y);

	path.forEach((point, index) => {
		if (index > 0) {
			ctx.lineTo(point.x, point.y);
		}
	});

	ctx.fill();
	ctx.stroke();
}


// Dev
let toDeg = function(deg) {
	return Number(deg) / pi * 180;
}