
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

    for *allocated_bodies {
        if it.type != .Dynamic {
            continue;
        }

        remaining_delta := dt;

        // TODO go through and group all bodies into groups depending on velocity spheres and step them together
        // TODO this also allows you to multithread, I guess? maybe collision steps are also independent in which
        // TODO case you can multithread everything

        // this can be a direct check of 0 without any epsilon because the resolve collision function will return dt if no collision is found
        while remaining_delta > 0 {
            position := it.position;
            it.position += it.velocity * remaining_delta;

            body_aabb := get_world_aabb(it);
            it.position = position;

            potential_collisions: [..] *KodaBody;
            query(tree_bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            minimum_delta := remaining_delta;
            minimum_normal: Vector3;

            for p: potential_collisions {
                delta, normal := resolve_collision(it, p, it.velocity, remaining_delta);
                if delta < minimum_delta {
                    minimum_delta = delta;
                    minimum_normal = normal;
                }
            }

            it.position += it.velocity * minimum_delta;
            remaining_delta -= minimum_delta;

            if dot(minimum_normal, minimum_normal) > 0 {
                // limit the allowed movement to the normal and adjust velocity
                it.velocity = project_direction(minimum_normal, it.velocity);
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

resolve_collision :: (a: *KodaBody, b: *KodaBody, velocity: Vector3, dt: float) -> Vector3, Vector3, Vector3, float {
    assert(a.type == .Dynamic, "Right now only dynamic on static collisions are allowed.");
    assert(b.type == .Static, "Right now only dynamic on static collisions are allowed.");

    minimum_delta := dt;
    minimum_normal: Vector3;

    // TODO could speed this up, because for each triangle plane check I only need the closest support point

    for shape_a: a.shapes {
        for shape_b: b.shapes {
            if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                position, velocity, normal, delta := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, dt);
                print("found potential at delta % / %\n", delta, dt);
                if delta < minimum_delta {
                    minimum_delta = delta;
                    minimum_normal = normal;
                }
            } else {
                print("Unhandled collision combination. % %\n", shape_a.type, shape_b.type);
                assert(false, "Unhandled collision combination.");
            }
        }
    }

    return minimum_delta, minimum_normal;
}

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, dt: float) -> Vector3, Vector3, Vector3, float {
    max_position: Vector3;
    max_velocity: Vector3;
    max_normal: Vector3;
    max_delta := dt;

    triangles := b_shape.mesh.triangles;

    // TODO convert the mesh aabb and stuff into body b space
    assert(b.position == .{});
    assert(b.rotation == .{});
    assert(b.scale == .{1, 1, 1});

    position := a.position;
    velocity := a.velocity;
    a.position += velocity * dt;
    body_end_aabb := get_world_aabb(a);
    a.position = position;

    body_start_aabb := get_world_aabb(a);
    body_path_aabb := merge(body_start_aabb, body_end_aabb);
    start_vertices := get_world_vertices(a, a_shape);

    potential_triangles: [..] Triangle3;
    defer array_free(potential_triangles);

    QueryInfo :: struct {
        potential_triangles: *[..] Triangle3;
        start_vertices: *KodaVertices;
        velocity: Vector3;
    }
    info: QueryInfo;
    info.potential_triangles = *potential_triangles;
    info.end_vertices = *a_end_vertices;
    info.velocity = velocity * dt;

    query(triangles, body_path_aabb, (triangle: Triangle3, info: QueryInfo) -> bool {
        triangle_vertices: KodaVertices(3) = ---;
        triangle_vertices.count = 3;
        triangle_vertices[0] = triangle.a;
        triangle_vertices[1] = triangle.b;
        triangle_vertices[2] = triangle.c;

        if solve_gjk(triangle_vertices, info.start_vertices, velocity2 = info.velocity) {
            array_add(info.potential_triangles, triangle);
        }

        return true;
    }, info);

    remaining_dt := dt;
    while remaining_dt > 0 {
        min_distance_squared := length(velocity * remaining_dt);
        min_triangle_index := -1;
        min_simplex: ComplexSimplex;
        min_triangle_point: Vector3;
        min_body_point: Vector3;

        for i: 0..potential_triangles.count - 1 {
            triangle := potential_triangles[i];
            triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
            if dot(triangle_normal, velocity) >= 0 {
                continue;
            }

            triangle_vertices: KodaVertices(3);
            triangle_vertices.count = 3;
            triangle_vertices[0] = triangle.a;
            triangle_vertices[1] = triangle.b;
            triangle_vertices[2] = triangle.c;
            start_collide, simplex, point1, point2 := solve_gjk(triangle_vertices, a_start_vertices, resolve_completely = true);
            if start_collide {
                print("Issue!! The starting body is overlapping the triangle.\n");
                continue;
            }

            distance_squared := distance_squared(point2, point2);
            if distance_squared < min_distance_squared {
                min_distance_squared = distance_squared;
                min_triangle_index = i;
                min_simplex = simplex;
                min_triangle_point = point1;
                min_body_point = point2;
            }
        }

        if min_triangle_index >= 0 {
            min_triangle := potential_triangles[min_triangle_index];
            array_unordered_remove_by_index(*potential_triangles, min_triangle_index);

            // vertex vertex treats vertex a as a plane
            // segment vertex, I guess, treats the line as a plane facing the vertex
            // plane vertex treats the plane as a planes
            // but this can all be accomplished by creating a plane between the two nearest points
            simplex1, simplex2 := resolve_simplex(min_simplex);
            normal := normalize(min_body_point - min_triangle_point);
            triangle_plane := create_plane3(min_triangle_point, normal);

            
        }



        simplex1, simplex2 := resolve_simplex(simplex);

        velocity_ray.p = get_support(a_start_vertices, -triangle_normal);
        velocity_ray.dir = velocity_dir;
        triangle_plane := create_plane3(triangle.a, triangle_normal);

        intersection, hit, t := intersection(triangle_plane, velocity_ray);
        if hit {
            if t < velocity_length {
                potential_remaining_delta := dt - t / velocity_length * dt;
                if current_delta > reamining_delta {
                    // testing a large epsilon value here
                    remaining_position = intersection + (a.position - velocity_ray.p) + triangle_normal * 0.0001;
                    remaining_velocity = project_direction(triangle_normal, it.velocity);
                    remaining_normal = triangle_normal;
                    reamining_delta = potential_remaining_delta;
                }
            }
        }
    }

    velocity_dir := normalize(velocity);

    velocity_ray: Ray3 = ---;
    velocity_ray.p = position;
    velocity_ray.dir = velocity_dir;
    velocity_length := length(velocity);

    transform := get_transform(a);

    if remaining_delta == 0 {
        return .{}, .{}, .{}, dt;
    }

    return remaining_position, remaining_velocity, remaining_normal, reamining_delta;
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
