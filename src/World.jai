
koda_physics_init :: () {
    physics_initialized = true;
    tree_bodies = create_bounding_tree(*KodaBody);
}

step_physics :: (dt: float) {
    start := get_time();
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

    clear_active_simplexes(*gjk_simplex_tracker);
    clear_active_simplexes(*gjk_dir_simplex_tracker);

    dirty_total += get_time() - start;

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
            body_aabb := get_world_aabb(a);
            body_aabb.lower -= 0.01;
            body_aabb.upper += 0.01;
            aabb_dim := body_aabb.upper - body_aabb.lower;
            if length(a.velocity) >= min(aabb_dim.x, min(aabb_dim.y, aabb_dim.z)) / 2.0 {
                print("Clip warning! Needs to be a convex hull or treat the body as a bullet!\n");
            }
        }

        a.collision_normals.count = 0;

        a.position += a.velocity * dt;

        penetrations.count = 0;
        
        // go through 1 loop to solve for all stair steps if this body can stair step
        start = get_time();
        if a.ground_enabled && a.step_height > 0 {
            body_aabb := get_world_aabb(a);

            potential_collisions.count = 0;
            query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            largest_resolve: Vector3;
            largest_normal: Vector3;
            for b: potential_collisions {
                for index_b: 0..b.shapes.count - 1 {
                    shape_b := b.shapes[index_b];

                    for index_a: 0..a.shapes.count - 1 {
                        shape_a := a.shapes[index_a];

                        if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                            start = get_time();
                            resolve, normal := resolve_collision_convex_hull_triangle_mesh_ground(a, b, shape_a, shape_b, index_a, index_b);
                            stair_resolve_total += get_time() - start;
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
                    a.velocity = project_direction(largest_normal, a.velocity);
                    array_add(*a.collision_normals, largest_normal);
                }
            }
        }
        stair_total += get_time() - start;

        body_aabb := get_world_aabb(a);

        // now we need to add all penetration vectors into the list
        start = get_time();
        potential_collisions.count = 0;
        query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
            if p.type == .Static {
                array_add(potential_collisions, p);
            }

            return true;
        }, *potential_collisions);
        aabb_search_total += get_time() - start;

        start = get_time();
        for b: potential_collisions {
            for index_b: 0..b.shapes.count - 1 {
                shape_b := b.shapes[index_b];

                for index_a: 0..a.shapes.count - 1 {
                    shape_a := a.shapes[index_a];

                    if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                        resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, index_a, index_b, *penetrations);
                    } else {
                        assert(false, "Unhandled collision combination.");
                    }
                }
            }
        }
        resolve_total += get_time() - start;

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

        start = get_time();
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
        resolve_math_total += get_time() - start;

        print("Iteration count %\n", iter);

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

    remove_inactive_simplexes(*gjk_simplex_tracker);
    remove_inactive_simplexes(*gjk_dir_simplex_tracker);
}

get_gravity :: () -> Vector3 {
    return gravity;
}

set_gravity :: (g: Vector3) {
    gravity = g;
}

#scope_module

resolve_collision_convex_hull_triangle_mesh_ground :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int) -> Vector3, Vector3 {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // TODO convert the mesh aabb and stuff into body b space
    assert(b.position == .{});
    assert(b.rotation == .{});
    assert(b.scale == .{1, 1, 1});
    triangles := b_shape.mesh.triangles;

    resolve_vector: Vector3;
    resolve_normal: Vector3;

    QueryInfo :: struct {
        resolve_vector: *Vector3;
        resolve_normal: *Vector3;
        body_vertices: *KodaVertices;
        body_transform: Matrix4;
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
    info.step_height = a.step_height;
    info.velocity = a.velocity;
    info.a = xx a;
    info.b = xx b;
    info.a_shape = xx a_shape_index;
    info.b_shape = xx b_shape_index;

    time := get_time();
    triangle_count = 0;
    query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo, triangle_id: int) -> bool {
        triangle_count += 1;
        triangle_normal_unscaled := cross(triangle.b - triangle.a, triangle.c - triangle.a);
        if dot(triangle_normal_unscaled, info.velocity) >= 0 {
            return true;
        }
        
        simplex_key := get_simplex_key(info.a, info.b, info.a_shape, info.b_shape, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        inner_time := get_time();

        // gjk_initial_simplex := get_cached_simplex(*gjk_simplex_tracker, simplex_key);
        gjk_initial_simplex: *Simplex;
        collide, simplex := solve_gjk(triangle_vertices, << info.body_vertices, transform2 = info.body_transform, initial_simplex = gjk_initial_simplex);

        stair_gjk_total += get_time() - inner_time;
        if collide {
            add_cached_simplex(*gjk_simplex_tracker, simplex_key, simplex);

            triangle_normal := normalize(triangle_normal_unscaled);
            if is_ground(triangle_normal) {
                inner_time = get_time();

                // gjk_dir_initial_simplex := get_cached_simplex(*gjk_dir_simplex_tracker, simplex_key);
                gjk_dir_initial_simplex: *Simplex;
                resolve_vector, resolve_simplex := solve_gjk_dir(triangle_vertices, << info.body_vertices, .{0, 1, 0}, transform2 = info.body_transform, initial_simplex = gjk_dir_initial_simplex);
                
                if length_squared(resolve_vector) > 0 {
                    add_cached_simplex(*gjk_dir_simplex_tracker, simplex_key, resolve_simplex);

                    stair_gjk_dir_total += get_time() - inner_time;
                    if resolve_vector.y <= info.step_height && resolve_vector.y > info.resolve_vector.y {
                        << info.resolve_vector = resolve_vector;
                        << info.resolve_normal = triangle_normal;
                    }
                }
            }
            // we don't currently have a way to get the normal direction of the gdk_dir function in order to evaluate whether or not it's ground, so we won't do this
        }

        return true;
    }, info);
    stair_query_total += get_time() - time;
    print("Triangle count %\n", triangle_count);

    return resolve_vector, resolve_normal;
}

triangle_count := 0;

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, a_shape_index: int, b_shape_index: int, penetrations: *[..] Penetration) {
    body_aabb := get_world_aabb(a);
    body_vertices := *a_shape.hull.vertices;
    body_transform := get_transform(a);

    // TODO convert the mesh aabb and stuff into body b space
    assert(b.position == .{});
    assert(b.rotation == .{});
    assert(b.scale == .{1, 1, 1});
    triangles := b_shape.mesh.triangles;

    QueryInfo :: struct {
        penetrations: *[..] Penetration;
        body_vertices: *KodaVertices;
        body_transform: Matrix4;
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
    info.velocity = a.velocity;
    info.a = xx a;
    info.b = xx b;
    info.a_shape = xx a_shape_index;
    info.b_shape = xx b_shape_index;

    query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo, triangle_id: int) -> bool {
        // triangle_normal_unscaled := cross(triangle.b - triangle.a, triangle.c - triangle.a);
        // if dot(triangle_normal_unscaled, info.velocity) >= 0 {
        //     return true;
        // }
        
        simplex_key := get_simplex_key(info.a, info.b, info.a_shape, info.b_shape, 0, xx triangle_id);

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        // gjk_initial_simplex := get_cached_simplex(*gjk_simplex_tracker, simplex_key);
        gjk_initial_simplex: *Simplex;
        collide, simplex := solve_gjk(triangle_vertices, << info.body_vertices, transform2 = info.body_transform, initial_simplex = gjk_initial_simplex);

        if collide {
            resolve_vector := solve_epa(triangle_vertices, << info.body_vertices, simplex, transform2 = info.body_transform);
            if length_squared(resolve_vector) > 0 {
                penetration := array_add(info.penetrations);
                penetration.normal = normalize(resolve_vector);
                penetration.d = dot(resolve_vector, penetration.normal);
            }
        }

        return true;
    }, info);
}

is_ground :: (normal: Vector3) -> bool {
    return dot(normal, .{0, 1, 0}) >= 0.85;
}

allocated_bodies: BlockAllocator(KodaBody, 1024);
// dynamic_bodies: [..] *KodaBody;
tree_bodies: BoundingTree(*KodaBody);
gravity: Vector3;
physics_initialized: bool;
gjk_simplex_tracker: SimplexTracker;
gjk_dir_simplex_tracker: SimplexTracker;

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
