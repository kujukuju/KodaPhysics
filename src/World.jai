
koda_physics_init :: () {
    physics_initialized = true;
    tree_bodies = create_bounding_tree(*KodaBody);
}

step_physics :: (dt: float) {
    for *allocated_bodies {
        // TODO if the body is alive
        it.velocity += gravity * dt;
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

    for *allocated_bodies {
        if it.type != .Dynamic {
            continue;
        }

        // no rotation so no need to update
        if length_squared(it.velocity) == 0 {
            continue;
        }

        it.position += it.velocity * dt;

        collide_history.count = 0;

        // TODO go through and group all bodies into groups depending on velocity spheres and step them together
        // TODO this also allows you to multithread, I guess? maybe collision steps are also independent in which
        // TODO case you can multithread everything

        // this can be a direct check of 0 without any epsilon because the resolve collision function will return dt if no collision is found
        had_collision := true;
        while had_collision {
            had_collision = false;

            print("collide loop\n");

            body_aabb := get_world_aabb(it);

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
            for p: potential_collisions {
                current_position, current_velocity, current_normal, current_shape_index := resolve_collision(it, p, collide_history);
                if length_squared(current_normal) > 0 {
                    collide_entry := array_add(*collide_history);
                    collide_entry.body = p;
                    collide_entry.shape_index = current_shape_index;

                    it.position = current_position;
                    it.velocity = current_velocity;
                    had_collision = true;
                    // TODO check for infinite loops with current_normal
                    break;
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

resolve_collision :: (a: *KodaBody, b: *KodaBody, collide_history: [..] CollideHistory) -> Vector3, Vector3, Vector3, int {
    assert(a.type == .Dynamic, "Right now only dynamic on static collisions are allowed.");
    assert(b.type == .Static, "Right now only dynamic on static collisions are allowed.");

    // TODO could speed this up, because for each triangle plane check I only need the closest support point

    // largest_position: Vector3;
    // largest_velocity: Vector3;
    // largest_normal: Vector3;

    for index_b: 0..b.shapes.count - 1 {
        shape_b := b.shapes[index_b];

        current_entry: CollideHistory;
        current_entry.body = b;
        current_entry.shape_index = index_b;

        found := false;
        for entry: collide_history {
            if entry == current_entry {
                found = true;
                break;
            }
        }

        if found {
            continue;
        }

        for index_a: 0..a.shapes.count - 1 {
            shape_a := a.shapes[index_a];

            if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                position, velocity, normal := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b);
                if length_squared(normal) > 0 {
                    return position, velocity, normal, index_b;
                }
                // if remaining_delta >= maximum_remaining_delta {
                //     maximum_position = position;
                //     maximum_velocity = velocity;
                //     if length_squared(normal) > 0 {
                //         maximum_normal = normal;
                //     }
                //     maximum_remaining_delta = remaining_delta;
                // }
            } else {
                print("Unhandled collision combination. % %\n", shape_a.type, shape_b.type);
                assert(false, "Unhandled collision combination.");
            }
        }
    }

    return .{}, .{}, .{}, 0;
}

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape) -> Vector3, Vector3, Vector3 {
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
        found_triangle: *bool;
        body_vertices: *KodaVertices;
    }
    info: QueryInfo;
    info.triangle = *triangle;
    info.simplex = *simplex;
    info.found_triangle = *found_triangle;
    info.body_vertices = *body_vertices;

    query(triangles, body_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
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
        return .{}, .{}, .{};
    }

    triangle_vertices: KodaVertices(3) = ---;
    triangle_vertices.count = 3;
    triangle_vertices[0] = triangle.a;
    triangle_vertices[1] = triangle.b;
    triangle_vertices[2] = triangle.c;

    triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
    resolve_vector := -solve_epa(triangle_vertices, body_vertices, simplex);
    // resolve_vector, resolve_point := solve_dtr(triangle, body_vertices);
    if length_squared(resolve_vector) == 0 {
        return .{}, .{}, .{};
    }

    print("resolve %\n", resolve_vector);

    solved_position := a.position + resolve_vector;
    solved_velocity := project_direction(triangle_normal, a.velocity);

    return solved_position, solved_velocity, triangle_normal;


    // max_position: Vector3;
    // max_velocity: Vector3;
    // max_normal: Vector3;
    // max_delta := dt;

    // triangles := b_shape.mesh.triangles;

    // // TODO convert the mesh aabb and stuff into body b space
    // assert(b.position == .{});
    // assert(b.rotation == .{});
    // assert(b.scale == .{1, 1, 1});

    // position := a.position;
    // velocity := a.velocity;
    // a.position += velocity * dt;
    // body_aabb := get_world_aabb(a);
    // body_vertices := get_world_vertices(a, a_shape);

    // body_start_aabb := get_world_aabb(a);
    // body_path_aabb := merge(body_start_aabb, body_end_aabb);

    // potential_triangles: [..] Triangle3;
    // defer array_free(potential_triangles);

    // QueryInfo :: struct {
    //     potential_triangles: *[..] Triangle3;
    //     body_vertices: *KodaVertices;
    //     velocity_line: Vector3;
    // }
    // info: QueryInfo;
    // info.potential_triangles = *potential_triangles;
    // info.body_vertices = *body_vertices;
    // info.velocity_line = velocity * dt;

    // query(triangles, body_path_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
    //     triangle_vertices: KodaVertices(3) = ---;
    //     triangle_vertices.count = 3;
    //     triangle_vertices[0] = triangle.a;
    //     triangle_vertices[1] = triangle.b;
    //     triangle_vertices[2] = triangle.c;

    //     if solve_gjk_simple(triangle_vertices, << info.body_vertices, velocity2 = info.velocity_line) {
    //         array_add(info.potential_triangles, triangle);
    //     }

    //     return true;
    // }, info);

    // remaining_dt := dt;
    // remaining_position := position;
    // while remaining_dt > 0 {
    //     min_distance_squared := length(velocity * remaining_dt);
    //     min_triangle_index := -1;
    //     min_simplex: ComplexSimplex;
    //     min_triangle_point: Vector3;
    //     min_triangle_normal: Vector3;
    //     min_body_point: Vector3;

    //     for i: 0..potential_triangles.count - 1 {
    //         triangle := potential_triangles[i];
    //         triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
    //         if dot(triangle_normal, velocity) >= 0 {
    //             continue;
    //         }

    //         triangle_vertices: KodaVertices(3);
    //         triangle_vertices.count = 3;
    //         triangle_vertices[0] = triangle.a;
    //         triangle_vertices[1] = triangle.b;
    //         triangle_vertices[2] = triangle.c;

    //         start_collide := solve_gjk(triangle_vertices, body_vertices);
    //         if start_collide {
    //             print("Issue!! The starting body is overlapping the triangle.\n");

    //             // this code might be better because itll only exit the actual overlapping part of the mesh and the triangle which could even be the edge of the triangle
    //             // for i: 1..simplex.count - 1 {
    //             //     simplex_point: Vector3;
    //             //     body_point: Vector3;
    //             //     if i == {
    //             //         case 0;
    //             //             simplex_point = simplex.a;
    //             //             body_point = simplex.a2;
    //             //         case 1;
    //             //             simplex_point = simplex.b;
    //             //             body_point = simplex.b2;
    //             //         case 2;
    //             //             simplex_point = simplex.c;
    //             //             body_point = simplex.c2;
    //             //         case 3;
    //             //             simplex_point = simplex.d;
    //             //             body_point = simplex.d2;
    //             //     }

    //             //     d2 := dot(simplex_point, simplex_point);
    //             //     if d2 < min_simplex_distance {
    //             //         min_simplex_distance = d2;
    //             //         min_body_point = body_point;
    //             //     }
    //             // }

    //             triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
    //             triangle_plane := create_plane3(triangle.a, triangle_normal);
    //             nearest_body_point := get_support(body_vertices, -triangle_normal);

    //             projected_point := nearest_point(triangle_plane, nearest_body_point);
    //             corrected_position := projected_point + (position - nearest_body_point);
    //             corrected_velocity := project_direction(triangle_normal, velocity);
    //             // TODO correct the delta time, I guess
    //             // TODO could also check all potential triangles for the deepest overlap, and solve for that instead, but they shouldnt ever overlap to start with
    //             return corrected_position, corrected_velocity, triangle_normal, remaining_dt;
    //         }

    //         // view1: [] Vector3;
    //         // view1.data = triangle_vertices.fixed.data;
    //         // view1.count = triangle_vertices.count;
    //         // view2: [] Vector3;
    //         // view2.data = body_vertices.fixed.data;
    //         // view2.count = body_vertices.count;

    //         // #import "JaiSerializer";
    //         // #import "File";
    //         // serializer :: Serializer(int, false);
    //         // bytes1: [..] u8;
    //         // bytes2: [..] u8;
    //         // // bytes3: [..] u8;
    //         // serializer.write(*bytes1, view1);
    //         // serializer.write(*bytes2, view2);
    //         // // serializer.write(*bytes3, info.velocity_line);
    //         // string1: string;
    //         // string1.data = bytes1.data;
    //         // string1.count = bytes1.count;
    //         // string2: string;
    //         // string2.data = bytes2.data;
    //         // string2.count = bytes2.count;
    //         // // string3: string;
    //         // // string3.data = bytes3.data;
    //         // // string3.count = bytes3.count;
    //         // write_entire_file("triangle_vertices.data", string1);
    //         // write_entire_file("body_vertices.data", string2);
            
    //         start_collide0, simplex, point1, point2 := solve_gjk(triangle_vertices, body_vertices, resolve_completely = true);

    //         closest_d2 := distance_squared(point1, point2);
    //         if closest_d2 < min_distance_squared {
    //             min_distance_squared = closest_d2;
    //             min_triangle_index = i;
    //             min_simplex = simplex;
    //             min_triangle_point = point1;
    //             min_triangle_normal = triangle_normal;
    //             min_body_point = point2;
    //         }
    //     }

    //     if min_triangle_index == -1 {
    //         return position + velocity * dt, velocity, .{}, 0.0;
    //     }

    //     min_triangle := potential_triangles[min_triangle_index];
    //     array_unordered_remove_by_index(*potential_triangles, min_triangle_index);

    //     // vertex vertex treats vertex a as a plane
    //     // segment vertex, I guess, treats the line as a plane facing the vertex
    //     // plane vertex treats the plane as a planes
    //     // but this can all be accomplished by creating a plane between the two nearest points
    //     simplex1, simplex2 := resolve_simplex(min_simplex);
    //     normal := normalize(min_body_point - min_triangle_point);
    //     triangle_plane := create_plane3(min_triangle_point, normal);

    //     ray: Ray3;
    //     ray.p = min_body_point;
    //     ray.dir = normalize(velocity);

    //     intersection_point, intersection_hit, intersection_t := intersection(triangle_plane, ray);
    //     if intersection_hit {
    //         travel_percent := intersection_t / length(velocity * remaining_dt);
    //         remaining_dt = max(remaining_dt - travel_percent * remaining_dt, 0);
    //         remaining_position = intersection_point + (position - min_body_point);
    //     }

    //     // if we're now colliding with the triangle in question, stop here
    //     a.position = remaining_position;
    //     body_vertices = get_world_vertices(a, a_shape);
    //     a.position = position;

    //     triangle_vertices: KodaVertices(3);
    //     triangle_vertices.count = 3;
    //     triangle_vertices[0] = min_triangle.a;
    //     triangle_vertices[1] = min_triangle.b;
    //     triangle_vertices[2] = min_triangle.c;
    //     // collide := solve_gjk_simple(triangle_vertices, body_vertices);
    //     collide, simplex, point1, point2 := solve_gjk(triangle_vertices, body_vertices, resolve_completely = true);
    //     if collide || distance_squared(point1, point2) < 0.000001 {
    //         corrected_velocity := project_direction(min_triangle_normal, velocity);
    //         return remaining_position, corrected_velocity, min_triangle_normal, remaining_dt;
    //     }
    // }

    // return position + velocity * dt, velocity, .{}, 0.0;
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

operator == :: (a: CollideHistory, b: CollideHistory) -> bool {
    return a.body == b.body && a.shape_index == b.shape_index;
}

#scope_export
