
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

    potential_collisions: [..] *KodaBody;
    collide_history: [..] CollideHistory;

    for *a: allocated_bodies {
        if a.type != .Dynamic {
            continue;
        }

        // no rotation so no need to update
        if length_squared(a.velocity) == 0 {
            continue;
        }

        a.position += a.velocity * dt;

        collide_history.count = 0;

        // TODO go through and group all bodies into groups depending on velocity spheres and step them together
        // TODO this also allows you to multithread, I guess? maybe collision steps are also independent in which
        // TODO case you can multithread everything

        // this can be a direct check of 0 without any epsilon because the resolve collision function will return dt if no collision is found
        had_collision := true;
        while collision_loop := had_collision {
            had_collision = false;

            body_aabb := get_world_aabb(a);

            potential_collisions.count = 0;
            query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            // largest_position: Vector3;
            // largest_velocity: Vector3;
            // largest_normal: Vector3;
            for b: potential_collisions {
                for index_b: 0..b.shapes.count - 1 {
                    shape_b := b.shapes[index_b];

                    current_entry := find_history(*collide_history, b, index_b);
                    if shape_b.type != .TriangleMesh && current_entry {
                        continue;
                    }

                    // at this point if there is a current entry then we know it's a triangle mesh

                    for index_a: 0..a.shapes.count - 1 {
                        shape_a := a.shapes[index_a];

                        if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                            triangle_history: [..] Triangle3;
                            if current_entry {
                                triangle_history = current_entry.triangle_history;
                            }
                            position, velocity, normal, triangle := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, triangle_history);
                            if length_squared(normal) > 0 {
                                if !current_entry {
                                    current_entry = array_add(*collide_history);
                                    current_entry.body = b;
                                    current_entry.shape_index = index_b;
                                }

                                array_add(*current_entry.triangle_history, triangle);

                                a.position = position;
                                a.velocity = velocity;
                                had_collision = true;
                                continue collision_loop;
                            }
                        } else {
                            print("Unhandled collision combination. % %\n", shape_a.type, shape_b.type);
                            assert(false, "Unhandled collision combination.");
                        }
                    }
                }
            }
        }
    }
}

get_gravity :: () -> Vector3 {
    return gravity;
}

set_gravity :: (g: Vector3) {
    gravity = g;
}

#scope_module

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, triangle_history: [..] Triangle3) -> Vector3, Vector3, Vector3, Triangle3 {
    body_aabb := get_world_aabb(a);
    body_vertices := get_world_vertices(a, a_shape);

    // TODO convert the mesh aabb and stuff into body b space
    assert(b.position == .{});
    assert(b.rotation == .{});
    assert(b.scale == .{1, 1, 1});
    triangles := b_shape.mesh.triangles;

    triangle: Triangle3;
    simplex: Simplex;
    found_triangle: bool;

    QueryInfo :: struct {
        triangle: *Triangle3;
        simplex: *Simplex;
        maximum_depth: float;
        found_triangle: *bool;
        body_vertices: *KodaVertices;
        triangle_history: [..] Triangle3;
    }
    info: QueryInfo;
    info.triangle = *triangle;
    info.simplex = *simplex;
    info.found_triangle = *found_triangle;
    info.body_vertices = *body_vertices;
    info.triangle_history = triangle_history;

    query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
        if find_triangle(info.triangle_history, triangle) {
            return true;
        }

        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        collide, simplex := solve_gjk(triangle_vertices, << info.body_vertices);
        if collide {
            << info.triangle = triangle;
            << info.simplex = simplex;
            << info.found_triangle = true;

            return false;
        }

        return true;
    }, info);

    if !found_triangle {
        return .{}, .{}, .{}, .{};
    }

    triangle_vertices: KodaVertices(3) = ---;
    triangle_vertices.count = 3;
    triangle_vertices[0] = triangle.a;
    triangle_vertices[1] = triangle.b;
    triangle_vertices[2] = triangle.c;

    triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
    resolve_vector: Vector3;
    if is_ground(triangle_normal) {
        resolve_vector = solve_gjk_dir(triangle_vertices, body_vertices, simplex, .{0, 1, 0});
        if resolve_vector.y > 0.5 {
            resolve_vector = solve_epa(triangle_vertices, body_vertices, simplex);
        }
        // print("ground resolve %\n", resolve_vector);
    } else {
        resolve_vector = solve_epa(triangle_vertices, body_vertices, simplex);
    }
    // resolve_vector, resolve_point := solve_dtr(triangle, body_vertices);
    if length_squared(resolve_vector) == 0 {
        return .{}, .{}, .{}, .{};
    }

    solved_position := a.position + resolve_vector;
    solved_velocity := project_direction(normalize(resolve_vector), a.velocity);

    return solved_position, solved_velocity, triangle_normal, triangle;
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
    return dot(normal, .{0, 1, 0}) >= 0.75;
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
    triangle_history: [..] Triangle3;
}

operator == :: (a: CollideHistory, b: CollideHistory) -> bool {
    return a.body == b.body && a.shape_index == b.shape_index;
}

find_history :: (history: *[..] CollideHistory, body: *KodaBody, shape_index: int) -> *CollideHistory {
    for *entry: << history {
        if entry.body == body && entry.shape_index == shape_index {
            return entry;
        }
    }

    return null;
}

find_triangle :: (history: [..] Triangle3, triangle: Triangle3) -> bool {
    for entry: history {
        if entry == triangle {
            return true;
        }
    }

    return false;
}

#scope_export
