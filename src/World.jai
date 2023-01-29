// KodaWorld :: struct {
//     polygons: CircleAllocator(KodaBody(KodaPolygon));
//     rectangles: CircleAllocator(KodaBody(KodaRectangle));

//     static_bodies: BoundingTree(*KodaBody(KodaShape)) = ---;
//     body_ids: CircleAllocator(*KodaBody(KodaShape));

//     gravity: Vector3 = Vector2.{0, 0.01};

//     contact_listener: #type (body: *KodaBody(KodaShape), normal: Vector23, extras: *u8);
//     contact_listener_extras: *u8;

//     create :: () -> KodaWorld {
//         world: KodaWorld;
//         world.static_bodies = BoundingTree(*KodaBody(KodaShape)).create();
//         return world;
//     }

//     free :: (world: KodaWorld) {
//         CircleAllocator(KodaBody(KodaPolygon)).free(world.polygons);
//         CircleAllocator(KodaBody(KodaRectangle)).free(world.rectangles);
//     }
// }

// create_body :: (world: *KodaWorld, shape: KodaPolygon) -> *KodaBody(KodaPolygon) {
//     #if DEBUG {
//         view := get_polygon_view(*shape);
//         if !is_convex(view) {
//             print("Tried to create a polygon body from a concave polygon. %\n", view);
//             assert(false);
//         }
//     }

//     body := claim(*world.polygons);
//     body.shape = shape;
//     body.id = claim_index(*world.body_ids, xx body);

//     return body;
// }

// create_body :: (world: *KodaWorld, shape: KodaRectangle) -> *KodaBody(KodaRectangle) {
//     body := claim(*world.rectangles);
//     body.shape = shape;
//     body.id = claim_index(*world.body_ids, xx body);

//     return body;
// }

// destroy_body :: (world: *KodaWorld, body: *KodaBody(KodaPolygon)) {
//     free(*world.body_ids, body.id);
//     free(*world.polygons, body);
// }

// destroy_body :: (world: *KodaWorld, body: *KodaBody(KodaRectangle)) {
//     free(*world.body_ids, body.id);
//     free(*world.rectangles, body);
// }

// set_contact_listener :: (world: *KodaWorld, listener: (body: *KodaBody(KodaShape), normal: Vector2, extras: *u8), extras: *u8) {
//     world.contact_listener = listener;
//     world.contact_listener_extras = extras;
// }

// query_static_bodies :: (world: *KodaWorld, aabb: AABB) -> [..] *KodaBody(KodaShape) {
//     bodies: [..] *KodaBody(KodaShape);
//     query_static_bodies(world, aabb, *bodies);

//     return bodies;
// }

// query_static_bodies :: (world: *KodaWorld, aabb: AABB, bodies: *[..] *KodaBody(KodaShape)) {
//     query(world.static_bodies, aabb, (body: *KodaBody(KodaShape), extras: *[..] *KodaBody(KodaShape)) -> bool {
//         array_add(extras, body);

//         return true;
//     }, bodies);
// }

// update :: (world: *KodaWorld, delta: float) {
//     // clear all previous collision normals from the last tick
//     clear_collisions(world);

//     // check for dirty bodies and update their aabb and update their tree position
//     clear_dirty(world, delta);

//     // check for static bodies whos tree_index is -1 and add them to the tree
//     initialize_static(world);

//     // move kinematic bodies
//     update_kinematic(world, delta);
// }

// clear_collisions :: (world: *KodaWorld) {
//     for i: 0..world.polygons.count - 1 {
//         body := world.polygons[i];
//         if !body {
//             continue;
//         }

//         array_reset_keeping_memory(*body.collision_normals);
//     }

//     for i: 0..world.rectangles.count - 1 {
//         body := world.rectangles[i];
//         if !body {
//             continue;
//         }

//         array_reset_keeping_memory(*body.collision_normals);
//     }
// }

// clear_dirty :: (world: *KodaWorld, delta: float) {
//     for i: 0..world.polygons.count - 1 {
//         body := world.polygons[i];
//         if !body {
//             continue;
//         }

//         clear_dirty(world, body, delta);
//     }

//     for i: 0..world.rectangles.count - 1 {
//         body := world.rectangles[i];
//         if !body {
//             continue;
//         }

//         clear_dirty(world, body, delta);
//     }
// }

// clear_dirty :: (world: *KodaWorld, body: *KodaBody($T), delta: float) {
//     if is_dirty(<< body) {
//         clear_dirty(body);
//     }

//     body.aabb = calculate_world_aabb(<< body);
//     if body.type == KodaBodyType.Static {
//         if body.tree_proxy != -1 {
//             // velocity * delta is an approximation of how I think the displacement vector is supposed to work for the tree
//             move_proxy(*world.static_bodies, body.tree_proxy, body.aabb, body.velocity * delta);
//         }
//     }
// }

// initialize_static :: (world: *KodaWorld) {
//     for i: 0..world.polygons.count - 1 {
//         body := world.polygons[i];
//         if !body {
//             continue;
//         }

//         initialize_static(world, body);
//     }

//     for i: 0..world.rectangles.count - 1 {
//         body := world.rectangles[i];
//         if !body {
//             continue;
//         }

//         initialize_static(world, body);
//     }
// }

// initialize_static :: (world: *KodaWorld, body: *KodaBody($T)) {
//     if body.tree_proxy != -1 {
//         return;
//     }

//     if body.type != KodaBodyType.Static {
//         return;
//     }

//     // add the body to the tree
//     body.tree_proxy = create_proxy(*world.static_bodies, body.aabb, xx body);

//     // check for other adjacent bodies and mark external edges
// }

// update_kinematic :: (world: *KodaWorld, delta: float) {
//     for i: 0..world.rectangles.count - 1 {
//         body := world.rectangles[i];
//         if !body {
//             continue;
//         }
//         if body.type != KodaBodyType.Kinematic {
//             continue;
//         }

//         // move the body, no velocity bounding for this method
//         // TODO I can optimize this
//         body_falling_speed := length(project(world.gravity, body.velocity)) * 0.04;
//         if dot(body.velocity, world.gravity) <= 0 {
//             body_falling_speed = 0;
//         }
//         gravity_speed := length(world.gravity);
//         gravity := max(gravity_speed - body_falling_speed * body_falling_speed, 0) * normalize(world.gravity);
//         body.velocity += gravity * delta;
//         body.position += body.velocity * delta;

//         edges: [..] EdgeRestriction;
//         collided: [..] u64;
//         // array_add(*edges, create_edge_restriction(normalize(-body.velocity)));

//         update_kinematic(world, body, *edges, *collided);
//     }
    
//     for i: 0..world.polygons.count - 1 {
//         body := world.polygons[i];
//         if !body {
//             continue;
//         }
//         if body.type != KodaBodyType.Kinematic {
//             continue;
//         }

//         // move the body, no velocity bounding for this method
//         // TODO I can optimize this
//         body_falling_speed := length(project(world.gravity, body.velocity));
//         gravity_speed := length(world.gravity);
//         gravity := max(gravity_speed - body_falling_speed * body_falling_speed, 0) * world.gravity;
//         body.velocity += gravity * delta;
//         body.position += body.velocity * delta;

//         edges: [..] EdgeRestriction;
//         collided: [..] u64;
//         // array_add(*edges, create_edge_restriction(normalize(-body.velocity)));

//         update_kinematic(world, body, *edges, *collided);
//     }
// }

// update_kinematic :: (world: *KodaWorld, body: *KodaBody(KodaRectangle), edges: *[..] EdgeRestriction, collided: *[..] u64) {
//     // TODO dont use pointers as identifiers in the future

//     // calculate all overlapped polygons
//     body_aabb := calculate_world_aabb(<< body);
//     // TODO could update this by skipping collided in the query callback
//     static_bodies: [..] *KodaBody(KodaShape) = query_static_bodies(world, body_aabb);
    
//     // determine the polygon with the most overlap
//     largest_resolve_vector: Vector2;
//     // this can be different because when you stair step up the normal is still the edge
//     largest_resolve_normal: Vector2;
//     largest_resolve_body: *KodaBody(KodaShape);
//     largest_resolve_polygon: BodyVertices;
//     // we prioritize steps first, so keep track
//     largest_resolve_is_step := false;

//     body_vertices := get_world_vertices(<< body);

//     // TODO don't collide against already checked polygons
//     for static_body: static_bodies {
//         if array_find(<< collided, cast(u64) static_body) {
//             continue;
//         }

//         static_vertices: BodyVertices = ---;
//         if static_body.shape.type == KodaShapeType.Rectangle {
//             cast_body: *KodaBody(KodaRectangle) = xx static_body;
//             static_vertices = get_world_vertices(<< cast_body);
//         } else if static_body.shape.type == KodaShapeType.Polygon {
//             cast_body: *KodaBody(KodaPolygon) = xx static_body;
//             static_vertices = get_world_vertices(<< cast_body);
//         } else {
//             assert(false);
//         }

//         step_resolve_vector, step_normal, step_collision := overlap(view(*static_vertices), view(*body_vertices), .{0, -1});
//         valid_step := step_collision;
//         valid_step = valid_step && step_resolve_vector.y >= -16;
//         valid_step = valid_step && is_ground(step_normal);
//         valid_step = valid_step && !check_collision(world, body_vertices + step_resolve_vector, cast(u64) static_body);
//         if valid_step {
//             // if this is a step, but we've already found a non-step, ignore it
//             if !largest_resolve_is_step {
//                 largest_resolve_is_step = true;
//                 largest_resolve_vector = .{0, 0};
//             }

//             resolve_length_squared := dot(step_resolve_vector);
//             if resolve_length_squared > dot(largest_resolve_vector) {
//                 largest_resolve_vector = step_resolve_vector;
//                 largest_resolve_normal = step_normal;
//                 largest_resolve_body = static_body;
//                 largest_resolve_polygon = static_vertices;
//             }
//         } else {
//             resolve_vector, collision := overlap(view(*static_vertices), view(*body_vertices));
//             if !collision {
//                 continue;
//             }

//             // if we've already found a valid step then don't do this
//             if largest_resolve_is_step {
//                 continue;
//             }

//             resolve_length_squared := dot(resolve_vector);
//             if resolve_length_squared > dot(largest_resolve_vector) {
//                 largest_resolve_vector = resolve_vector;
//                 largest_resolve_normal = normalize(resolve_vector);
//                 largest_resolve_body = static_body;
//                 largest_resolve_polygon = static_vertices;
//             }
//         }
//     }

//     // this is your escape
//     if dot(largest_resolve_vector) < EPSILON {
//         return;
//     }

//     array_add(collided, cast(u64) largest_resolve_body);

//     // determine the normal of the collision edge
//     array_add(edges, create_edge_restriction(largest_resolve_normal));

//     if dot(normalize(body.velocity), largest_resolve_normal) <= 0 {
//         if world.contact_listener {
//             world.contact_listener(xx body, largest_resolve_normal, world.contact_listener_extras);
//         }
//         array_add(*body.collision_normals, largest_resolve_normal);
//     }

//     // calculate the union of all the collision edge restrictions
//     // so merge all overlapping collision edge normals
//     merge_edges(edges);

//     // if there is no remaining allowed angle, stop moving here
//     // but also this should never happen, you would be stuck insie a shape
//     // if there are no edges it means we're in a closed environment with nowhere to go, so stop movement
//     if edges.count == 0 {
//         print("Edge count 0. Movement completely restricted. I expect you're stuck in a wall.\n");
//         // body.position -= body.velocity * delta;
//         body.velocity = .{0, 0};
//         return;
//     }

//     // if the normal and the vector are different you stepped up
//     stepped := largest_resolve_vector != largest_resolve_normal;

//     // TODO if this isnt a ground angle don't just project the velocity but inverse project the velocity
//     // if it is a ground angle then don't resolve correctly, resolve upwards?

//     // calculate the angle to exit the polygon
//     // correct the exit angle based on the list of restrictive edges
//     // project the exit vector onto the new correcter exit angle, offset your character and repeat
//     restricted_resolve_vector := restrict_movement(largest_resolve_vector, << edges);
//     if length(restricted_resolve_vector) == 0 {
//         print("BAD\n");
//         // body.velocity = .{};
//     } else {
//         normalized_resolve_vector := normalize(restricted_resolve_vector);
//         if dot(normalized_resolve_vector, normalize(body.velocity)) <= 0 {
//             rotated_resolve_vector := rotate(normalized_resolve_vector, PI / 2);
//             body.velocity = project(rotated_resolve_vector, body.velocity);
//         }
//     }
//     body.position += restricted_resolve_vector;

//     update_kinematic(world, body, edges, collided);
// }

// update_kinematic :: (world: *KodaWorld, body: *KodaBody(KodaPolygon), edges: *[..] EdgeRestriction, collided: *[..] u64) {
//     print("kinematic polygons not supported yet\n");
//     assert(false);
// }

// check_collision :: (world: *KodaWorld, vertices: BodyVertices, ignore_id: u64) -> bool { 
//     aabb: AABB = ---;
//     aabb.lower = vertices[0];
//     aabb.upper = vertices[0];

//     for i: 1..vertices.count - 1 {
//         aabb.lower = min(aabb.lower, vertices[i]);
//         aabb.upper = min(aabb.upper, vertices[i]);
//     }

//     static_bodies: [..] *KodaBody(KodaShape) = query_static_bodies(world, aabb);
//     for static_body: static_bodies {
//         if cast(u64) static_body == ignore_id {
//             continue;
//         }

//         static_vertices: BodyVertices = ---;
//         if static_body.shape.type == KodaShapeType.Rectangle {
//             cast_body: *KodaBody(KodaRectangle) = xx static_body;
//             static_vertices = get_world_vertices(<< cast_body);
//         } else if static_body.shape.type == KodaShapeType.Polygon {
//             cast_body: *KodaBody(KodaPolygon) = xx static_body;
//             static_vertices = get_world_vertices(<< cast_body);
//         } else {
//             assert(false);
//         }

//         resolve_vector, collision := overlap(view(*static_vertices), view(*vertices));
//         if collision {
//             return true;
//         }
//     }

//     return false;
// }

// #scope_file

// // the fan made by rotating ccw to most_ccw and rotating cw to most_cw is the allowed area
// EdgeRestriction :: struct {
//     most_ccw: Vector2;
//     most_cw: Vector2;
// }

// create_edge_restriction :: (normal: Vector2) -> EdgeRestriction {
//     restriction: EdgeRestriction = ---;
//     restriction.most_ccw = rotate(normal, PI / 2.0);
//     restriction.most_cw = rotate(normal, -PI / 2.0);
//     return restriction;
// }

// restrict_movement :: (vector: Vector2, edges: [] EdgeRestriction) -> Vector2 {
//     direction := normalize(vector);

//     for edge: edges {
//         // if the direction is inside the restricted areas but in the same direction as an edge, project it
//         if is_right(edge.most_cw, direction) && is_forward(edge.most_cw, direction) {
//             // project vector onto most_cw
//             // inverse_project(edge.most_cw, vector);
//             return project(edge.most_cw, vector);
//         } else if is_left(edge.most_ccw, direction) && is_forward(edge.most_ccw, direction) {
//             // project vector onto most_ccw
//             // inverse_project(edge.most_ccw, vector);
//             return project(edge.most_ccw, vector);
//         } else if is_right(edge.most_cw, direction) || is_left(edge.most_ccw, direction) {
//             // we've already tested the forwardness of the vector, so if we're moving into the restricted area just stop moving because we've hit a corner
//             return .{0, 0};
//         }
//     }

//     // found no edges that are collisions so continue
//     return vector;
// }

// merge_edges :: (edges: *[..] EdgeRestriction) {
//     calculate_edge_overlap :: (edge1: EdgeRestriction, edge2: EdgeRestriction) -> EdgeRestriction, bool, closed: bool {
//         // if edge2ccw is to the left of edge1ccw, it overlaps, so calculate new cw edge
//         restriction: EdgeRestriction = ---;

//         if is_left(edge1.most_ccw, edge2.most_ccw) {
//             if is_left(edge1.most_ccw, edge2.most_cw) {
//                 return restriction, true, true;
//             } else if is_left(edge1.most_cw, edge2.most_cw) {
//                 restriction.most_cw = edge2.most_cw;
//                 restriction.most_ccw = edge1.most_ccw;
//             } else {
//                 restriction.most_cw = edge1.most_cw;
//                 restriction.most_ccw = edge1.most_ccw;
//             }

//             return restriction, true, false;
//         } else if is_right(edge1.most_cw, edge2.most_ccw) {
//             if is_left(edge1.most_ccw, edge2.most_cw) {
//                 return restriction, true, true;
//             } else if is_right(edge1.most_ccw, edge2.most_cw) {
//                 restriction.most_cw = edge2.most_cw;
//                 restriction.most_ccw = edge1.most_ccw;
//             } else {
//                 // the only option that remains is they perfectly align
//                 restriction.most_cw = edge1.most_ccw;
//                 restriction.most_ccw = edge1.most_ccw;
//             }

//             return restriction, true, false;
//         } else if is_left(edge1.most_ccw, edge2.most_cw) {
//             if is_right(edge1.most_cw, edge2.most_ccw) {
//                 return restriction, true, true;
//             } else if is_left(edge1.most_cw, edge2.most_ccw) {
//                 restriction.most_cw = edge1.most_cw;
//                 restriction.most_ccw = edge2.most_ccw;
//             } else {
//                 // theyre perfectly aligned
//                 restriction.most_cw = edge1.most_cw;
//                 restriction.most_ccw = edge1.most_cw;
//             }

//             return restriction, true, false;
//         } else if is_right(edge1.most_cw, edge2.most_cw) {
//             if is_right(edge1.most_cw, edge2.most_ccw) {
//                 return restriction, true, true;
//             } else if is_right(edge1.most_ccw, edge2.most_ccw) {
//                 restriction.most_cw = edge1.most_cw;
//                 restriction.most_ccw = edge2.most_ccw;
//             } else {
//                 restriction.most_cw = edge1.most_cw;
//                 restriction.most_ccw = edge1.most_ccw;
//             }

//             return restriction, true, false;
//         }

//         return restriction, false, false;
//     }

//     a := 0;
//     while a < edges.count {
//         defer a += 1;

//         edge1 := (<< edges)[a];

//         b := a + 1;
//         while b < edges.count {
//             defer b += 1;

//             edge2 := (<< edges)[b];

//             merged, overlap, closed := calculate_edge_overlap(edge1, edge2);
//             if closed {
//                 // if we eventually get to a point where the available movement area entirely closes then exit out of this
//                 edges.count = 0;
//                 print("Movement completely restricted. I expect you're stuck in a wall.\n");
//                 return;
//             } else if overlap {
//                 array_ordered_remove_by_index(edges, b);
//                 array_ordered_remove_by_index(edges, a);

//                 array_add(edges, merged);

//                 // restart the loop so we can check everything against this entry
//                 a = 0;
//                 b = 0;
//                 break;
//             }
//         }
//     }
// }

// is_ground :: (normal: Vector2) -> bool {
//     return dot(normal, .{0, -1}) > 0.75;
// }

// #import "Basic";
// #import "JaiMath";

// // TODO there has to be a better way to declare this than declaring it in every module
// DEBUG :: true;
// EPSILON :: 0.000001;