
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

    push_temporary_context();

    Penetration :: struct {
        normal: Vector3;
        d: float;
    }

    potential_collisions: [..] *KodaBody;
    body_history: [..] CollideHistory;
    triangle_history: [..] Triangle3;
    penetrations: [..] Penetration;

    for *a: allocated_bodies {
        if a.type != .Dynamic {
            continue;
        }

        // no rotation so no need to update
        if length_squared(a.velocity) == 0 {
            continue;
        }
        {
            // just test code for now, if youre moving faster than half your aabb you could potentially clip
            body_aabb := get_world_aabb(a);
            aabb_dim := body_aabb.upper - body_aabb.lower;
            if length(a.velocity) >= min(aabb_dim.x, min(aabb_dim.y, aabb_dim.z)) / 2.0 {
                print("Clip warning! Needs to be a convex hull or treat the body as a bullet!\n");
            }
        }

        a.position += a.velocity * dt;

        body_history.count = 0;
        triangle_history.count = 0;
        penetrations.count = 0;
        
        // go through 1 loop to solve for all stair steps if this body can stair step
        if a.ground_enabled && a.step_height > 0 {
            body_aabb := get_world_aabb(a);

            potential_collisions.count = 0;
            query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            largest_step: float;
            largest_triangle: Triangle3;
            largest_normal: Vector3;
            for b: potential_collisions {
                for index_b: 0..b.shapes.count - 1 {
                    shape_b := b.shapes[index_b];

                    for index_a: 0..a.shapes.count - 1 {
                        shape_a := a.shapes[index_a];

                        if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                            resolve, triangle, normal, ground := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, triangle_history);
                            if ground {
                                resolve_length := length(resolve);
                                if resolve_length > largest_step {
                                    largest_step = resolve_length;
                                    largest_triangle = triangle;
                                    largest_normal = normal;
                                }
                            }
                        } else {
                            assert(false, "Unhandled collision combination.");
                        }
                    }
                }
            }

            if largest_step > 0 {
                penetration := array_add(*penetrations);
                penetration.normal = largest_normal;
                penetration.d = 0.0;

                a.position += largest_step * Vector3.{0, 1, 0};
                // TODO really this should be something based on the triangle normal if its a ground triangle
                a.velocity.y = 0.0;
            }
        }

        body_aabb := get_world_aabb(a);

        // now we need to add all penetration vectors into the list
        potential_collisions.count = 0;
        query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
            if p.type == .Static {
                array_add(potential_collisions, p);
            }

            return true;
        }, *potential_collisions);

        for b: potential_collisions {
            for index_b: 0..b.shapes.count - 1 {
                shape_b := b.shapes[index_b];

                for index_a: 0..a.shapes.count - 1 {
                    shape_a := a.shapes[index_a];

                    if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                        resolve, triangle, normal, ground := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, triangle_history);
                        if !ground && length_squared(resolve) > 0 {
                            penetration := array_add(*penetrations);
                            penetration.normal = normalize(resolve);
                            penetration.d = dot(resolve, penetration.normal);
                        }
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

        if penetrations.count > 0 {
            print("penetrations %\n", penetrations);

            matrix: [..] [3] float;
            array_resize(*matrix, penetrations.count);
            rhs: [..] float;
            array_resize(*rhs, penetrations.count);
            for i: 0..penetrations.count - 1 {
                penetration := penetrations[i];
                matrix[i][0] = penetration.normal.x;
                matrix[i][1] = penetration.normal.y;
                matrix[i][2] = penetration.normal.z;
                rhs[i] = penetration.d;
            }

            resolution := solve_least_squares_svd(matrix, rhs);

            // resolution := solve_gauss_seidel(matrix, rhs);
            print("result %\n", resolution);
            
            a.position.x += resolution[0];
            a.position.y += resolution[1];
            a.position.z += resolution[2];
        }

        // largest_step: float;
        // largest_triangle: Triangle3;
        // for b: potential_collisions {
        //     for index_b: 0..b.shapes.count - 1 {
        //         shape_b := b.shapes[index_b];

        //         // TODO dont need this until we support static convex hulls
        //         // if !valid_body(body_history, b, index_b) {
        //         //     continue;
        //         // }

        //         for index_a: 0..a.shapes.count - 1 {
        //             shape_a := a.shapes[index_a];s

        //             if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
        //                 resolve, triangle, ground := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, triangle_history);
        //                 if ground {
        //                     resolve_length := length(resolve);
        //                     if resolve_length > largest_stair {
        //                         largest_step = resolve_length;
        //                         largest_triangle = triangle;
        //                     }
        //                 }
        //             } else {
        //                 assert(false, "Unhandled collision combination.");
        //             }
        //         }
        //     }
        // }

        // if largest_step > 0 {
        //     triangle_normal := normalize(cross(largest_triangle.b - largest_triangle.a, largest_triangle.c - largest_triangle.a));
        //     penetration := array_add(*penetrations);
        //     penetration.normal = triangle_normal;
        //     penetration.d = 0.0;
        // }

        // TODO go through and group all bodies into groups depending on velocity spheres and step them together
        // TODO this also allows you to multithread, I guess? maybe collision steps are also independent in which
        // TODO case you can multithread everything

        // this can be a direct check of 0 without any epsilon because the resolve collision function will return dt if no collision is found
        // had_collision := true;
        // while collision_loop := had_collision {
        //     had_collision = false;

        //     body_aabb := get_world_aabb(a);

        //     potential_collisions.count = 0;
        //     query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
        //         if p.type == .Static {
        //             array_add(potential_collisions, p);
        //         }

        //         return true;
        //     }, *potential_collisions);

        //     largest_resolve: Vector3;
        //     largest_is_triangle: bool;
        //     largest_triangle: Triangle3;
        //     largest_is_ground: bool;
        //     for b: potential_collisions {
        //         for index_b: 0..b.shapes.count - 1 {
        //             shape_b := b.shapes[index_b];

        //             // TODO dont need this until we support static convex hulls
        //             // if !valid_body(body_history, b, index_b) {
        //             //     continue;
        //             // }

        //             for index_a: 0..a.shapes.count - 1 {
        //                 shape_a := a.shapes[index_a];

        //                 if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
        //                     resolve, triangle, ground := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, triangle_history);
        //                     if length_squared(resolve) > 0 {
        //                         should_replace := false;
        //                         should_replace = should_replace || (ground && !largest_is_ground);
        //                         should_replace = should_replace || (ground && largest_is_ground && length_squared(resolve) > length_squared(largest_resolve));
        //                         should_replace = should_replace || (!ground && !largest_is_ground && length_squared(resolve) > length_squared(largest_resolve));
        //                         if should_replace {
        //                             largest_resolve = resolve;
        //                             largest_is_triangle = true;
        //                             largest_triangle = triangle;
        //                             largest_is_ground = ground;
        //                         }
        //                     }
        //                 } else {
        //                     print("Unhandled collision combination. % %\n", shape_a.type, shape_b.type);
        //                     assert(false, "Unhandled collision combination.");
        //                 }
        //             }
        //         }
        //     }

        //     if length_squared(largest_resolve) > 0 {
        //         a.position = a.position + largest_resolve;
        //         resolve_normal := normalize(largest_resolve);
        //         if dot(a.velocity, resolve_normal) < 0 {
        //             a.velocity = project_direction(resolve_normal, a.velocity);
        //         }
        //         triangle_normal := normalize(cross(largest_triangle.b - largest_triangle.a, largest_triangle.c - largest_triangle.a));
        //         if largest_is_triangle {
        //             array_add(*triangle_history, largest_triangle);
        //         }
        //         had_collision = true;
        //     }
        // }
    }
}

get_gravity :: () -> Vector3 {
    return gravity;
}

set_gravity :: (g: Vector3) {
    gravity = g;
}

#scope_module

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, triangle_history: [..] Triangle3) -> Vector3, Triangle3, Vector3, bool {
    body_aabb := get_world_aabb(a);
    body_vertices := get_world_vertices(a, a_shape);

    // TODO convert the mesh aabb and stuff into body b space
    assert(b.position == .{});
    assert(b.rotation == .{});
    assert(b.scale == .{1, 1, 1});
    triangles := b_shape.mesh.triangles;

    triangle: Triangle3;
    // triangle_normal: Vector3;
    // triangle_d: float;
    resolve_vector: Vector3;
    resolve_depth_squared: float;
    resolve_ground: bool;
    resolve_normal: Vector3;
    found_triangle: bool;

    QueryInfo :: struct {
        triangle: *Triangle3;
        // triangle_normal: *Vector3;
        // triangle_d: *float;
        resolve_vector: *Vector3;
        resolve_depth_squared: *float;
        resolve_ground: *bool;
        resolve_normal: *Vector3;
        found_triangle: *bool;
        body_vertices: *KodaVertices;
        step_height: float;
        velocity: Vector3;
        triangle_history: [..] Triangle3;
    }
    info: QueryInfo;
    info.triangle = *triangle;
    // info.triangle_normal = *triangle_normal;
    // info.triangle_d = *triangle_d;
    info.resolve_vector = *resolve_vector;
    info.resolve_depth_squared = *resolve_depth_squared;
    info.resolve_ground = *resolve_ground;
    info.resolve_normal = *resolve_normal;
    info.found_triangle = *found_triangle;
    info.body_vertices = *body_vertices;
    info.step_height = a.step_height;
    info.velocity = a.velocity;
    info.triangle_history = triangle_history;

    if a.ground_enabled {
        query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
            if !valid_triangle(info.triangle_history, triangle) {
                return true;
            }

            triangle_normal_unscaled := cross(triangle.b - triangle.a, triangle.c - triangle.a);
            if dot(triangle_normal_unscaled, info.velocity) >= 0 {
                return true;
            }

            triangle_vertices: KodaVertices(3) = ---;
            triangle_vertices.count = 3;
            triangle_vertices[0] = triangle.a;
            triangle_vertices[1] = triangle.b;
            triangle_vertices[2] = triangle.c;

            collide, simplex := solve_gjk(triangle_vertices, << info.body_vertices);
            if collide {
                triangle_normal := normalize(triangle_normal_unscaled);
                if is_ground(triangle_normal) {
                    resolve_vector := solve_gjk_dir(triangle_vertices, << info.body_vertices, simplex, .{0, 1, 0});
                    if resolve_vector.y <= info.step_height {
                        if (<< info.resolve_ground) {
                            // if we already have a ground resolution we have to be sure this one is better
                            if resolve_vector.y > info.resolve_vector.y {
                                << info.triangle = triangle;
                                // << info.triangle_normal = .{};
                                // << info.triangle_d = 0.0;
                                << info.resolve_vector = resolve_vector;
                                << info.resolve_depth_squared = length_squared(resolve_vector);
                                << info.resolve_ground = true;
                                << info.resolve_normal = triangle_normal;
                                << info.found_triangle = true;
                            }
                        } else {
                            // if we don't already have a ground resolution then use this
                            << info.triangle = triangle;
                            // << info.triangle_normal = .{};
                            // << info.triangle_d = 0.0;
                            << info.resolve_vector = resolve_vector;
                            << info.resolve_depth_squared = length_squared(resolve_vector);
                            << info.resolve_ground = true;
                            << info.resolve_normal = triangle_normal;
                            << info.found_triangle = true;
                        }
                    } else if !(<< info.resolve_ground) {
                        // we don't bother with non-step collisions if we've already found a step
                        // we can only find a larger step at this point
                        resolve_vector = solve_epa(triangle_vertices, << info.body_vertices, simplex);
                        resolve_depth_squared := length_squared(resolve_vector);
                        if resolve_depth_squared > (<< info.resolve_depth_squared) {
                            << info.triangle = triangle;
                            // << info.triangle_normal = .{};
                            // << info.triangle_d = 0.0;
                            << info.resolve_vector = resolve_vector;
                            << info.resolve_depth_squared = resolve_depth_squared;
                            << info.resolve_normal = normalize(resolve_vector);
                            << info.found_triangle = true;
                        }
                    }
                } else {
                    // the normal of this triangle doesn't qualify it to be treated as ground
                    resolve_vector := solve_epa(triangle_vertices, << info.body_vertices, simplex);
                    resolve_depth_squared := length_squared(resolve_vector);
                    if resolve_depth_squared > (<< info.resolve_depth_squared) {
                        << info.triangle = triangle;
                        // << info.triangle_normal = .{};
                        // << info.triangle_d = 0.0;
                        << info.resolve_vector = resolve_vector;
                        << info.resolve_depth_squared = resolve_depth_squared;
                        << info.resolve_normal = normalize(resolve_vector);
                        << info.found_triangle = true;
                    }
                }
            }

            return true;
        }, info);
    } else {
        query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
            if !valid_triangle(info.triangle_history, triangle) {
                return true;
            }

            triangle_vertices: KodaVertices(3) = ---;
            triangle_vertices.count = 3;
            triangle_vertices[0] = triangle.a;
            triangle_vertices[1] = triangle.b;
            triangle_vertices[2] = triangle.c;

            collide, simplex := solve_gjk(triangle_vertices, << info.body_vertices);
            if collide {
                resolve_vector := solve_epa(triangle_vertices, << info.body_vertices, simplex);
                resolve_depth_squared := length_squared(resolve_vector);
                if resolve_depth_squared > (<< info.resolve_depth_squared) {
                    << info.triangle = triangle;
                    // << info.triangle_normal = .{};
                    // << info.triangle_d = 0.0;
                    << info.resolve_vector = resolve_vector;
                    << info.resolve_depth_squared = resolve_depth_squared;
                    << info.resolve_normal = normalize(resolve_vector);
                    << info.found_triangle = true;
                }
            }

            return true;
        }, info);
    }

    if !found_triangle {
        return .{}, .{}, .{}, false;
    }

    return resolve_vector, triangle, resolve_normal, resolve_ground;
}

get_support :: (vertices: KodaVertices($N), direction: Vector3) -> Vector3 {
    largest_dot := dot(vertices[0], direction);
    largest_vertex := vertices[0];
    for i: 1..vertices.count - 1 {
        vertex := vertices[i];

        current_dot := dot(vertex, direction);
        if current_dot > largest_dot {
            largest_dot = current_dot;
            largest_vertex = vertex;
        }
    }

    return largest_vertex;
}

is_ground :: (normal: Vector3) -> bool {
    return dot(normal, .{0, 1, 0}) >= 0.85;
    // return dot(normal, .{0, 1, 0}) >= 0.75;
}

allocated_bodies: BlockAllocator(KodaBody, 1024);
// dynamic_bodies: [..] *KodaBody;
tree_bodies: BoundingTree(*KodaBody);
gravity: Vector3;
physics_initialized: bool;

#scope_file

CollideHistory :: struct {
    body: *KodaBody;
    shape_index: int;
}

NormalHistory :: struct {
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

valid_triangle :: (history: [..] Triangle3, triangle: Triangle3) -> bool {
    for entry: history {
        if entry == triangle {
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
