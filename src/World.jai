
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
    defer array_free(potential_collisions);
    penetrations: [..] Penetration;
    defer array_free(penetrations);

    for *a: allocated_bodies {
        if a.type != .Dynamic {
            continue;
        }

        // no rotation so no need to update
        if length_squared(a.velocity) == 0 {
            continue;
        }

        {
            // TODO just test code for now, if youre moving faster than half your aabb you could potentially clip
            a_aabb := get_world_aabb(a);
            a_aabb.lower -= 0.01;
            a_aabb.upper += 0.01;
            aabb_dim := a_aabb.upper - a_aabb.lower;
            if length(a.velocity) >= min(aabb_dim.x, min(aabb_dim.y, aabb_dim.z)) / 2.0 {
                print("Clip warning! Needs to be a convex hull or treat the body as a bullet!\n");
            }
        }

        a.collision_normals.count = 0;
        a.position += a.velocity * dt;

        penetrations.count = 0;
        
        // go through 1 loop to solve for all stair steps if this body can stair step
        if a.ground_enabled && a.step_height > 0 {
            a_aabb := get_world_aabb(a);

            potential_collisions.count = 0;
            query(tree_bodies, a_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            largest_resolve: Vector3;
            largest_normal: Vector3;
            for b: potential_collisions {
                b_aabb := get_world_aabb(b);

                // TODO I could move a_aabb into b local space instead of both into world space and the check would be a lot better
                if !intersects(a_aabb, b_aabb) {
                    continue;
                }

                for index_b: 0..b.shapes.count - 1 {
                    shape_b := b.shapes[index_b];
                    b_shape_aabb := transform_aabb(get_transform(b), shape_b.aabb);

                    for index_a: 0..a.shapes.count - 1 {
                        shape_a := a.shapes[index_a];
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
                penetration := array_add(*penetrations);
                penetration.normal = largest_normal;
                penetration.d = 0.0;

                a.position += largest_resolve;
                if dot(a.velocity, largest_normal) < 0 {
                    a.velocity.y = project_direction(largest_normal, a.velocity).y;
                    array_add(*a.collision_normals, largest_normal);
                }
            }
        }

        a_aabb := get_world_aabb(a);

        // now we need to add all penetration vectors into the list
        potential_collisions.count = 0;
        query(tree_bodies, a_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
            if p.type == .Static {
                array_add(potential_collisions, p);
            }

            return true;
        }, *potential_collisions);

        for b: potential_collisions {
            b_aabb := get_world_aabb(b);
            
            // TODO I could move a_aabb into b local space instead of both into world space and the check would be a lot better
            if !intersects(a_aabb, b_aabb) {
                continue;
            }

            for index_b: 0..b.shapes.count - 1 {
                shape_b := b.shapes[index_b];
                b_shape_aabb := transform_aabb(get_transform(b), shape_b.aabb);

                for index_a: 0..a.shapes.count - 1 {
                    shape_a := a.shapes[index_a];
                    a_shape_aabb := transform_aabb(get_transform(a), shape_a.aabb);

                    if !intersects(a_shape_aabb, b_shape_aabb) {
                        continue;
                    }

                    if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                        resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, index_a, index_b, *penetrations);
                    } else if shape_a.type == .ConvexHull && shape_b.type == .ConvexHull {
                        resolve_collision_convex_hull_convex_hull(a, b, shape_a, shape_b, index_a, index_b, *penetrations);
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

            for penetration: penetrations {
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
        for penetration: penetrations {
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

    remove_inactive_simplexes(*gjk_mesh_tracker);
    remove_inactive_simplexes(*gjk_hull_tracker);
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

#scope_module

resolve_collision_convex_hull_triangle_mesh_ground :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int) -> Vector3, Vector3 {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // hopefully I'm doing this right, we can only step up a certain distance, so we actually only care about shapes that are within that distance
    body_aabb.upper.y = min(body_aabb.upper.y, body_aabb.lower.y + a.step_height);

    triangle_transform := get_transform(b);
    triangles := b_shape.mesh.triangles;

    resolve_vector: Vector3;
    resolve_normal: Vector3;

    QueryInfo :: struct {
        resolve_vector: *Vector3;
        resolve_normal: *Vector3;
        body_vertices: *KodaVertices;
        body_transform: Matrix4;
        triangle_transform: Matrix4;
        step_height: float;
        velocity: Vector3;
        a: u64;
        b: u64;
        a_shape: u64;
        b_shape: u64;
    }
    info: QueryInfo;
    info.resolve_vector = *resolve_vector;
    info.resolve_normal = *resolve_normal;
    info.body_vertices = body_vertices;
    info.body_transform = body_transform;
    info.triangle_transform = triangle_transform;
    info.step_height = a.step_height;
    info.velocity = a.velocity;
    info.a = xx a;
    info.b = xx b;
    info.a_shape = xx a_shape_index;
    info.b_shape = xx b_shape_index;

    inv_triangle_transform := inverse(triangle_transform);
    triangle_space_body_aabb := transform_aabb(inv_triangle_transform, body_aabb);

    time := get_time();
    query(triangles, triangle_space_body_aabb, (triangle: Triangle3, info: QueryInfo, triangle_id: int) -> bool {
        triangle_normal_unscaled := transform_vector(info.triangle_transform, cross(triangle.b - triangle.a, triangle.c - triangle.a));
        if dot(triangle_normal_unscaled, .{0, 1, 0}) < 0 {
            return true;
        }

        // doing it this way doesn't account for standing on the corner of a triangle, but I think that's handled by epa
        triangle_normal := normalize(triangle_normal_unscaled);
        if !is_ground(triangle_normal) {
            return true;
        }
        
        simplex_key := get_simplex_key(info.a, info.b, info.a_shape, info.b_shape, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        gjk_directions := get_cached_simplex(*gjk_mesh_tracker, simplex_key);
        collide, simplex, directions := solve_gjk(*triangle_vertices, info.body_vertices, transform1 = info.triangle_transform, transform2 = info.body_transform, cached_directions = gjk_directions);

        if collide {
            add_cached_simplex(*gjk_mesh_tracker, simplex_key, directions);

            resolve_vector := solve_gjk_dir(*triangle_vertices, info.body_vertices, .{0, 1, 0}, transform1 = info.triangle_transform, transform2 = info.body_transform);
            if resolve_vector.y <= info.step_height && resolve_vector.y > info.resolve_vector.y {
                << info.resolve_vector = resolve_vector;
                << info.resolve_normal = normalize(triangle_normal_unscaled);
            }
        }

        return true;
    }, info);

    return resolve_vector, resolve_normal;
}

resolve_collision_convex_hull_convex_hull_ground :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int) -> Vector3, Vector3 {
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

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int, penetrations: *[..] Penetration) {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // TODO convert the mesh aabb and stuff into body b space
    triangle_transform := get_transform(b);
    triangles := b_shape.mesh.triangles;

    QueryInfo :: struct {
        penetrations: *[..] Penetration;
        body_vertices: *KodaVertices;
        body_transform: Matrix4;
        triangle_transform: Matrix4;
        velocity: Vector3;
        a: u64;
        b: u64;
        a_shape: u64;
        b_shape: u64;
    }
    info: QueryInfo;
    info.penetrations = penetrations;
    info.body_vertices = body_vertices;
    info.body_transform = body_transform;
    info.triangle_transform = triangle_transform;
    info.velocity = a.velocity;
    info.a = xx a;
    info.b = xx b;
    info.a_shape = xx a_shape_index;
    info.b_shape = xx b_shape_index;

    inv_triangle_transform := inverse(triangle_transform);
    traingle_space_body_aabb := transform_aabb(inv_triangle_transform, body_aabb);

    query(triangles, traingle_space_body_aabb, (triangle: Triangle3, info: QueryInfo, triangle_id: int) -> bool {
        simplex_key := get_simplex_key(info.a, info.b, info.a_shape, info.b_shape, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        gjk_directions := get_cached_simplex(*gjk_mesh_tracker, simplex_key);
        collide, simplex, directions := solve_gjk(*triangle_vertices, info.body_vertices, transform1 = info.triangle_transform, transform2 = info.body_transform, cached_directions = gjk_directions);
        
        if collide {
            add_cached_simplex(*gjk_mesh_tracker, simplex_key, directions);

            resolve_vector := solve_epa(*triangle_vertices, info.body_vertices, simplex, transform1 = info.triangle_transform, transform2 = info.body_transform);
            if length_squared(resolve_vector) > 0 {
                penetration := array_add(info.penetrations);
                penetration.normal = normalize(resolve_vector);
                penetration.d = dot(resolve_vector, penetration.normal);
            }
        }

        return true;
    }, info);
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

allocated_bodies: BlockAllocator(KodaBody, 1024);
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
