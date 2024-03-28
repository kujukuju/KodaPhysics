
koda_physics_init :: () {
    physics_initialized = true;
    tree_bodies = create_bounding_tree(*KodaBody);
}

step_physics :: (dt: float) {
    for *allocated_bodies {
        // TODO if the body is alive
        it.velocity += it.gravity_strength * gravity * dt;
    }

    for *allocated_bodies {
        if is_dirty(it) {
            clear_dirty(it);

            aabb := get_world_aabb(it);
            move_proxy(*tree_bodies, it.proxy, aabb);
        }
    }

    clear_active_simplexes(*gjk_mesh_tracker);
    clear_active_simplexes(*gjk_hull_tracker);

    potential_collisions: [..] *KodaBody;
    potential_collisions.allocator = temp;
    penetrations: [..] Penetration;
    penetrations.allocator = temp;

    for *a: allocated_bodies {
        step_physics(dt, a, *potential_collisions, *penetrations);
    }

    remove_inactive_simplexes(*gjk_mesh_tracker);
    remove_inactive_simplexes(*gjk_hull_tracker);
}

step_physics :: (dt: float, a: *KodaBody) {
    potential_collisions: [..] *KodaBody;
    potential_collisions.allocator = temp;
    penetrations: [..] Penetration;
    penetrations.allocator = temp;

    step_physics(dt, a, *potential_collisions, *penetrations);
}

step_physics :: (dt: float, a: *KodaBody, potential_collisions: *[..] *KodaBody, penetrations: *[..] Penetration) {
    if a.type != .Dynamic || a.sensor {
        return;
    }

    // no rotation so no need to update
    if length_squared(a.velocity) == 0 {
        return;
    }

    {
        // TODO just test code for now, if youre moving faster than half your aabb you could potentially clip
        a_aabb := get_world_aabb(a);
        a_aabb.lower -= 0.01;
        a_aabb.upper += 0.01;
        aabb_dim := a_aabb.upper - a_aabb.lower;
        if length(a.velocity) >= min(aabb_dim.x, min(aabb_dim.y, aabb_dim.z)) / 2.0 {
            print("Clip warning! Needs to be a convex hull or treat the body as a bullet! % %\n", length(a.velocity), min(aabb_dim.x, min(aabb_dim.y, aabb_dim.z)) / 2.0);
        }
    }

    a.collision_normals.count = 0;
    a.position += a.velocity * dt;

    penetrations.count = 0;
    
    // go through 1 loop to solve for all stair steps if this body can stair step
    if a.ground_enabled && a.step_height > 0 {
        a_aabb := get_world_aabb(a);

        potential_collisions.count = 0;
        for tree_body: query(*tree_bodies, a_aabb) {
            if tree_body.type == .Static && !tree_body.sensor {
                array_add(potential_collisions, tree_body);
            }
        }

        largest_resolve: Vector3;
        largest_normal: Vector3;
        for b: potential_collisions.* {
            b_aabb := get_world_aabb(b);

            // TODO I could move a_aabb into b local space instead of both into world space and the check would be a lot better
            if !intersects(a_aabb, b_aabb) {
                continue;
            }

            for index_b: 0..b.shapes.count - 1 {
                shape_b := *b.shapes[index_b];
                b_shape_aabb := transform_aabb(get_transform(b), shape_b.aabb);

                for index_a: 0..a.shapes.count - 1 {
                    shape_a := *a.shapes[index_a];
                    a_shape_aabb := transform_aabb(get_transform(a), shape_a.aabb);

                    if !intersects(a_shape_aabb, b_shape_aabb) {
                        continue;
                    }

                    if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                        resolve, normal := resolve_collision_convex_hull_triangle_mesh_ground(a, b, shape_a, shape_b, index_a, index_b);
                        if length_squared(resolve) > length_squared(largest_resolve) {
                            largest_resolve = resolve;
                            largest_normal = normal;
                        }
                    } else if shape_a.type == .ConvexHull && shape_b.type == .ConvexHull {
                        resolve, normal := resolve_collision_convex_hull_convex_hull_ground(a, b, shape_a, shape_b, index_a, index_b);
                        if length_squared(resolve) > length_squared(largest_resolve) {
                            largest_resolve = resolve;
                            largest_normal = normal;
                        }
                    } else {
                        assert(false, "Unhandled collision combination.");
                    }
                }
            }
        }

        if length_squared(largest_resolve) > 0 {
            penetration := array_add(penetrations);
            penetration.normal = largest_normal;
            penetration.d = 0.0;

            a.position += largest_resolve;
            if dot(a.velocity, largest_normal) < 0 {
                a.velocity.y = project_direction(largest_normal, a.velocity).y;
                array_add(*a.collision_normals, largest_normal);
            }
        }
    }

    // TODO if you jump from a tall height you can hit the ground and get shot in a random direction due to the ground resolve not happening first
    /*
    after ground velocity is {-0.0075, -0.01936, -0}
    velocity accumulated {0, 0, 0}
    after resolve velocity is {-0.0075, -0.01936, -0}
    after ground velocity is {-0.0075, -0.02, -0}
    found penetration {{-0.002567, 0.999997, 0.000493}, 0.156175}
    found penetration {{0.001923, -0, -0.999998}, 0.137372}
    velocity accumulated {-0.000137, 0.156175, -0.137295}
    after resolve velocity is {-0.00751, -0.008722, -0.009914}
    offset based on ground {0, 0.146375, 0} 7ff6_ed41_c400
    after ground velocity is {-0.004622, -0.000009, -0.005907}
    found penetration {{-0.002567, 0.999997, 0.0005}, 0}
    found penetration {{-0.002567, 0.999997, 0.000493}, 0.000039}
    velocity accumulated {-0, 0.000039, 0}
    after resolve velocity is {-0.004622, -0.000009, -0.005907}
    */

    a_aabb := get_world_aabb(a);

    // now we need to add all penetration vectors into the list
    potential_collisions.count = 0;
    for body: query(*tree_bodies, a_aabb) {
        if body.type == .Static && !body.sensor {
            array_add(potential_collisions, body);
        }
    }

    for b: potential_collisions.* {
        b_aabb := get_world_aabb(b);
        
        // TODO I could move a_aabb into b local space instead of both into world space and the check would be a lot better
        if !intersects(a_aabb, b_aabb) {
            continue;
        }

        for index_b: 0..b.shapes.count - 1 {
            shape_b := *b.shapes[index_b];
            b_shape_aabb := transform_aabb(get_transform(b), shape_b.aabb);

            for index_a: 0..a.shapes.count - 1 {
                shape_a := *a.shapes[index_a];
                a_shape_aabb := transform_aabb(get_transform(a), shape_a.aabb);

                if !intersects(a_shape_aabb, b_shape_aabb) {
                    continue;
                }

                if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                    resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, index_a, index_b, penetrations);
                } else if shape_a.type == .ConvexHull && shape_b.type == .ConvexHull {
                    resolve_collision_convex_hull_convex_hull(a, b, shape_a, shape_b, index_a, index_b, penetrations);
                } else {
                    assert(false, "Unhandled collision combination.");
                }
            }
        }
    }

    // now we solve all the penetrations
    // we could use least squares matrix method
    // but since we want an exact solution if possible
    // and we probably don't have many collisions we can use gaussian
    // and expand the dimension of the resolution vector with empty data
    // vx * p1x + vy * p1y + vz * p1z = d1
    // vx * p2x + vy * p2y + vz * p2z = d2

    ITERATION_COUNT :: 32;

    accumulated_vector: Vector3;

    iter := 0;

    for i: 0..ITERATION_COUNT - 1 {
        found_penetration := false;

        // using the current accumulation vector is a better idea in practice because it applies them all at the same time
        // and if we divide by the penetration count it shouldnt ever go beyond a constraint
        // and theoretically this would handle no solution collisions best but its about twice as slow, which isnt very bad
        // but I can't notice any improvement

        // current_accumulated_vector: Vector3;

        for penetration: penetrations.* {
            accumulated_magnitude := dot(accumulated_vector, penetration.normal);
            remaining_magnitude := (penetration.d - accumulated_magnitude) / penetrations.count;

            if abs(remaining_magnitude) > 0.001 {
                accumulated_vector += remaining_magnitude * penetration.normal;
                found_penetration = true;
            }
        }

        // accumulated_vector += current_accumulated_vector;

        if !found_penetration {
            break;
        }

        iter += 1;
    }

    velocity_accumulated: Vector3;
    for penetration: penetrations.* {
        velocity_accumulated += penetration.normal * penetration.d;
    }

    if length_squared(accumulated_vector) > 0 {
        a.position += accumulated_vector;
        if dot(velocity_accumulated, a.velocity) < 0 {
            normalized_vector := normalize(velocity_accumulated);
            a.velocity = project_direction(normalized_vector, a.velocity);
            array_add(*a.collision_normals, normalized_vector);
        }
    }
}

get_line_intersection :: (line: Line3, $sensor: bool = false, $static_only: bool = true) -> Vector3, Vector3, bool {
    aabb: AABB3 = .{min(line.p1, line.p2), max(line.p1, line.p2)};

    search_direction := normalize(line.p1 - line.p2);
    best_point: Vector3 = line.p2;
    best_normal: Vector3;

    for tree_body: query(*tree_bodies, aabb) {
        if tree_body.sensor != sensor {
            continue;
        }

        #if static_only {
            if tree_body.type != .Static {
                continue;
            }
        }

        for *shape: tree_body.shapes {
            if shape.type == .TriangleMesh {
                a_transform := get_transform(tree_body);
                inv_a_transform := inverse(a_transform);
                a_space_body_aabb := transform_aabb(inv_a_transform, aabb);

                triangles := *shape.mesh.triangles;

                for triangle: query(triangles, a_space_body_aabb) {
                    world_triangle: Triangle3 = .{
                        transform_point(a_transform, triangle.a),
                        transform_point(a_transform, triangle.b),
                        transform_point(a_transform, triangle.c),
                    };

                    found_point, hit := intersection(world_triangle, line);
                    if hit && dot(found_point - line.p2, search_direction) > dot(best_point - line.p2, search_direction) {
                        best_point = found_point;
                        best_normal = get_normal(world_triangle);
                    }
                }
            } else if shape.type == .ConvexHull {
                // theres probably a better way to do this
                a_transform := get_transform(tree_body);
                for i: 0..shape.hull.indices.count / 3 - 1 {
                    index1 := shape.hull.indices[i * 3];
                    index2 := shape.hull.indices[i * 3 + 1];
                    index3 := shape.hull.indices[i * 3 + 2];

                    world_triangle: Triangle3 = ---;
                    world_triangle.a = transform_point(a_transform, shape.hull.vertices[index1]);
                    world_triangle.b = transform_point(a_transform, shape.hull.vertices[index2]);
                    world_triangle.c = transform_point(a_transform, shape.hull.vertices[index3]);

                    found_point, hit := intersection(world_triangle, line);
                    if hit && dot(found_point - line.p2, search_direction) > dot(best_point - line.p2, search_direction) {
                        best_point = found_point;
                        best_normal = get_normal(world_triangle);
                    }
                }
            }
        }
    }

    return best_point, best_normal, best_point != line.p2;
}

get_body_collisions :: (shape: *HullShape, transform: Matrix4, $sensor: bool = false, $static: bool = false, $dynamic: bool = false) -> [] *KodaBody {
    #run assert(static || dynamic, "You cannot get bodies without specifying static or dynamic bodies.\n");

    aabb := transform_aabb(transform, shape.aabb);

    final_bodies: [..] *KodaBody;
    final_bodies.allocator = temp;

    for tree_body: query(*tree_bodies, aabb) {
        if tree_body.sensor != sensor {
            continue;
        }

        #if !static {
            if tree_body.type == .Static {
                continue;
            }
        }

        #if !dynamic {
            if tree_body.type == .Dynamic {
                continue;
            }
        }

        for *body_shape: tree_body.shapes {
            if body_shape.type == .TriangleMesh {
                a_transform := get_transform(tree_body);
                inv_a_transform := inverse(a_transform);
                a_space_body_aabb := transform_aabb(inv_a_transform, aabb);

                triangles := *body_shape.mesh.triangles;
                found_collision: bool;

                for triangle: query(triangles, a_space_body_aabb) {
                    triangle_vertices: KodaVertices(3) = ---;
                    triangle_vertices.count = 3;
                    triangle_vertices[0] = triangle.a;
                    triangle_vertices[1] = triangle.b;
                    triangle_vertices[2] = triangle.c;

                    collision := solve_gjk(*triangle_vertices, *shape.vertices, transform1 = a_transform, transform2 = transform);
                    if collision {
                        found_collision = true;
                        break;
                    }
                };

                if found_collision {
                    array_add(*final_bodies, tree_body);
                }
            } else if body_shape.type == .ConvexHull {
                a_vertices := *body_shape.hull.vertices;
                a_transform := get_transform(tree_body);

                if solve_gjk(a_vertices, *shape.vertices, a_transform, transform) {
                    array_add(*final_bodies, tree_body);
                }
            }
        }
    }

    return final_bodies;
}

get_gravity :: () -> Vector3 {
    return gravity;
}

set_gravity :: (g: Vector3) {
    gravity = g;
}

is_ground :: (normal: Vector3) -> bool {
    return dot(normal, .{0, 1, 0}) >= 0.70;
}

get_world_bodies :: () -> [] *KodaBody {
    bodies: [..] *KodaBody;
    bodies.allocator = temp;

    for *body: allocated_bodies {
        array_add(*bodies, body);
    }

    return bodies;
}

#scope_module

resolve_collision_convex_hull_triangle_mesh_ground :: (a: *KodaBody, b: *KodaBody, a_shape: *KodaShape, b_shape: *KodaShape, a_shape_index: int, b_shape_index: int) -> Vector3, Vector3 {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // hopefully I'm doing this right, we can only step up a certain distance, so we actually only care about shapes that are within that distance
    body_aabb.upper.y = min(body_aabb.upper.y, body_aabb.lower.y + a.step_height);

    triangle_transform := get_transform(b);
    triangles := *b_shape.mesh.triangles;

    resolve_vector: Vector3;
    resolve_normal: Vector3;

    inv_triangle_transform := inverse(triangle_transform);
    triangle_space_body_aabb := transform_aabb(inv_triangle_transform, body_aabb);

    time := seconds_since_init();
    for triangle, triangle_id: query(triangles, triangle_space_body_aabb) {
        triangle_normal_unscaled := transform_vector(triangle_transform, cross(triangle.b - triangle.a, triangle.c - triangle.a));
        if dot(triangle_normal_unscaled, .{0, 1, 0}) < 0 {
            continue;
        }

        // doing it this way doesn't account for standing on the corner of a triangle, but I think that's handled by epa
        triangle_normal := normalize(triangle_normal_unscaled);
        if !is_ground(triangle_normal) {
            continue;
        }
        
        simplex_key := get_simplex_key(xx a, xx b, xx a_shape_index, xx b_shape_index, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        gjk_directions := get_cached_simplex(*gjk_mesh_tracker, simplex_key);
        collide, simplex, directions := solve_gjk(*triangle_vertices, body_vertices, transform1 = triangle_transform, transform2 = body_transform, cached_directions = gjk_directions);

        if collide {
            add_cached_simplex(*gjk_mesh_tracker, simplex_key, directions);

            gjk_resolve_vector := solve_gjk_dir(*triangle_vertices, body_vertices, .{0, 1, 0}, transform1 = triangle_transform, transform2 = body_transform);
            if gjk_resolve_vector.y <= a.step_height && gjk_resolve_vector.y > resolve_vector.y {
                resolve_vector = gjk_resolve_vector;
                resolve_normal = normalize(triangle_normal_unscaled);
            }
        }
    };

    return resolve_vector, resolve_normal;
}

resolve_collision_convex_hull_convex_hull_ground :: (a: *KodaBody, b: *KodaBody, a_shape: *KodaShape, b_shape: *KodaShape, a_shape_index: int, b_shape_index: int) -> Vector3, Vector3 {
    a_vertices := *a_shape.hull.vertices;
    a_transform := get_transform(a);

    b_vertices := *b_shape.hull.vertices;
    b_transform := get_transform(b);
        
    simplex_key := get_simplex_key(xx a, xx b, xx a_shape_index, xx b_shape_index, 0, 0);

    gjk_directions := get_cached_simplex(*gjk_hull_tracker, simplex_key);
    collide, simplex, directions := solve_gjk(b_vertices, a_vertices, transform1 = b_transform, transform2 = a_transform, cached_directions = gjk_directions);

    final_resolve_vector: Vector3;
    final_resolve_normal: Vector3;
    if collide {
        add_cached_simplex(*gjk_hull_tracker, simplex_key, directions);

        resolve_vector, resolve_normal := solve_gjk_dir(b_vertices, a_vertices, .{0, 1, 0}, transform1 = b_transform, transform2 = a_transform);
        // TODO if I'm worried about stair stepping up edges that aren't ground normals I should only resolve when the simplex count is 3, and then resolve the rest with epa I guess
        if is_ground(resolve_normal) {
            if length_squared(resolve_vector) > 0 {
                if resolve_vector.y <= a.step_height && resolve_vector.y > final_resolve_vector.y {
                    final_resolve_vector = resolve_vector;
                    final_resolve_normal = resolve_normal;
                }
            }
        }
    }

    return final_resolve_vector, final_resolve_normal;
}

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: *KodaShape, b_shape: *KodaShape, a_shape_index: int, b_shape_index: int, penetrations: *[..] Penetration) {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // TODO convert the mesh aabb and stuff into body b space
    triangle_transform := get_transform(b);
    triangles := *b_shape.mesh.triangles;

    inv_triangle_transform := inverse(triangle_transform);
    triangle_space_body_aabb := transform_aabb(inv_triangle_transform, body_aabb);

    for triangle, triangle_id: query(triangles, triangle_space_body_aabb) {
        simplex_key := get_simplex_key(xx a, xx b, xx a_shape_index, xx b_shape_index, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        gjk_directions := get_cached_simplex(*gjk_mesh_tracker, simplex_key);
        collide, simplex, directions := solve_gjk(*triangle_vertices, body_vertices, transform1 = triangle_transform, transform2 = body_transform, cached_directions = gjk_directions);
        
        if collide {
            add_cached_simplex(*gjk_mesh_tracker, simplex_key, directions);

            resolve_vector := solve_epa(*triangle_vertices, body_vertices, simplex, transform1 = triangle_transform, transform2 = body_transform);
            if length_squared(resolve_vector) > 0 {
                penetration := array_add(penetrations);
                penetration.normal = normalize(resolve_vector);
                penetration.d = dot(resolve_vector, penetration.normal);
            }
        }
    };
}

resolve_collision_convex_hull_convex_hull :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int, penetrations: *[..] Penetration) {
    a_vertices := *a_shape.hull.vertices;
    a_transform := get_transform(a);

    b_vertices := *b_shape.hull.vertices;
    b_transform := get_transform(b);
        
    simplex_key := get_simplex_key(xx a, xx b, xx a_shape_index, xx b_shape_index, 0, 0);

    gjk_directions := get_cached_simplex(*gjk_hull_tracker, simplex_key);
    collide, simplex, directions := solve_gjk(b_vertices, a_vertices, transform1 = b_transform, transform2 = a_transform, cached_directions = gjk_directions);

    if collide {
        add_cached_simplex(*gjk_hull_tracker, simplex_key, directions);

        resolve_vector := solve_epa(b_vertices, a_vertices, simplex, transform1 = b_transform, transform2 = a_transform);
        if length_squared(resolve_vector) > 0 {
            penetration := array_add(penetrations);
            penetration.normal = normalize(resolve_vector);
            penetration.d = dot(resolve_vector, penetration.normal);
        }
    }
}

allocated_bodies: BlockAllocator(KodaBody, 256);
tree_bodies: BoundingTree(*KodaBody);
gravity: Vector3;
physics_initialized: bool;
gjk_mesh_tracker: SimplexTracker;
gjk_hull_tracker: SimplexTracker;

#scope_file

CollideHistory :: struct {
    body: *KodaBody;
    shape_index: int;
}

NormalHistory :: struct {
    normal: Vector3;
    d: float;
}

Penetration :: struct {
    normal: Vector3;
    d: float;
}

operator == :: (a: CollideHistory, b: CollideHistory) -> bool {
    return a.body == b.body && a.shape_index == b.shape_index;
}

valid_body :: (history: [..] CollideHistory, body: *KodaBody, shape_index: int) -> bool {
    for entry: history {
        if entry.body == body && entry.shape_index == shape_index {
            return false;
        }
    }

    return true;
}

// get_restricted_vector :: (resolve: Vector3, normals: [..] Vector3) -> Vector3, Vector3 {
//     // before you do this I think you have to solve all stair step collisions
//     // then from this new position get all collisions
//     // you can add together all resolution vectors of all collisions to get the new displacement direction

//     // basically set up each normal as a plane with the origin at 0,0,0
//     // take the resolve vector as a plane set up at 0,0,0
//     // move the resolve plane forward by the resolve vector distance
//     // project the new resolve position onto the nearest feature of the new plane collisions
//     // this would work, but I think there's a better way

//     // only consider planes whos normals are against the resolve vector
//     // build out the list of all planes in some winding order around resolve vector
//     // each adjacent plane should make a line intersection with each other
//     // sort these into the list
//     // project the resolve vector onto each of these restrictions

//     RestrictionType :: enum {
//         Plane;
//         Line;
//     }

//     Restriction :: struct {
//         type: RestrictionType;
//         direction: Vector3;
//         union {

//         } 
//     }

//     restrictions: [..] Restriction;

//     // this is extremely hard to figure out analytically like this
//     // so I think I just need to construct the simplex that contains the resolve vector using the intersection of the planes
//     // then find the nearest feature to the resolve vector, whether it's a line or a plane (or a point?)
//     // I think ray directions are only allowed to go in the normal direction of the planes that are intersecting

//     // you have your new resolve vector
//     // get all the normals that are against the resolve vector
//     // get the intersection shape of all those planes
//     // get the dot product between your resolve vector and the resulting shape

//     // we keep track of currently restricted normals
//     // we only consider new normals that are going against our velocity
//     // if we find a new velocity, if that velocity is going against every previously restricted normal we stop

//     // we shouldn't possibly have to iterate more than n^2 times

//     for limit: 0..normals.count - 1 {
//         for i: 0..normals.count - 1 {

//         }
//     }
// }

#scope_export
