
koda_physics_init :: () {
    tree_bodies = create_bounding_tree(*KodaBody);
}

step_physics :: (dt: float) {
    push_temporary_context();

    for body: allocated_bodies {
        if body.type != .Dynamic {
            continue;
        }

        remaining_delta := dt;

        // TODO go through and group all bodies into groups depending on velocity spheres and step them together
        // TODO this also allows you to multithread, I guess? maybe collision steps are also independent in which
        // TODO case you can multithread everything

        // this can be a direct check of 0 without any epsilon because the resolve collision function will return dt if no collision is found
        while remaining_delta > 0 {
            position := body.position;
            body.position += body.velocity * remaining_delta;

            body_aabb := get_world_aabb(body);
            body.position = position;

            potential_collisions: [..] *KodaBody;
            query(bodies, body_aabb, (p: *KodaBody, potential_collisions: *[..] *KodaBody) -> bool {
                if p.type == .Static {
                    array_add(*potential_collisions, p);
                }

                return true;
            }, *potential_collisions);

            for p: potential_collisions {
                minimum_delta, minimum_normal := resolve_collision(body, p, body.velocity, remaining_delta);

                body.position += body.velocity * minimum_delta;
                body.velocity = project_direction(minimum_normal, body.velocity);

                // limit the allowed movement to the normal and adjust velocity

                remaining_delta -= minimum_delta;
            }
        }

        

        // body_vertices := get_world_vertices(body);

        // for p: potential_collisions {
        //     p_vertices := get_world_vertices(p);

        //     if solve_gjk_simple(p_vertices, body_vertices) {

        //     }
        // }
    }
}

resolve_collision :: (a: *KodaBody, b: *KodaBody, velocity: Vector3, dt: float) -> float, Vector3 {
    assert(a.type == .Dynamic, "Right now only dynamic on static collisions are allowed.");
    assert(b.type == .Static, "Right now only dynamic on static collisions are allowed.");

    minimum_delta := dt;
    minimum_normal: Vector3;

    // TODO could speed this up, because for each triangle plane check I only need the closest support point

    for shape_a: a.shapes {
        for shape_b: b.shapes {
            if shape_a.type == .ConvexHull && shape_b.type == .TriangleMesh {
                delta, normal := resolve_collision_convex_hull_triangle_mesh(a, b, shape_a, shape_b, velocity, dt);
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

resolve_collision_convex_hull_triangle_mesh :: (a: *KodaBody, b: *KodaBody, a_shape: KodaShape, b_shape: KodaShape, velocity: Vector3, dt: float) -> float, Vector3 {
    current_delta := FLOAT32_MAX;

    minimum_delta := dt;
    minimum_normal: Vector3;
    minimum_triangle: Triangle3;

    triangles := b.mesh.triangles;

    while minimum_delta < current_delta {
        current_delta = minimum_delta;

        position := a.position;
        a.position += velocity * dt;
        
        body_aabb := get_world_aabb(a);
        a.position = position;

        potential_triangles: [..] Triangle3;
        defer array_free(potential_triangles);

        query(triangles, body_aabb, (triangle: Triangle3, potential_triangles: *[..] Triangle3) -> bool {
            array_add(potential_triangles, triangle);
        }, *potential_triangles);

        a_vertices := get_world_vertices(a, a_shape);

        velocity_ray: Ray3 = ---;
        velocity_ray.p = position;
        velocity_ray.dir = normalize(velocity);
        velocity_length := length(velocity);

        transform := get_transform(a);
        for triangle: potential_triangles {
            // triangle_vertices: KodaVertices(3);
            // triangle_vertices.count = 3;
            // triangle_vertices[0] = Vector4.{triangle.a.x, triangle.a.y, triangle.a.z, 1,0};

            // triangle_a := (transform * Vector4.{triangle.a.x, triangle.a.y, triangle.a.z, 1.0}).xyz;
            // triangle_b := (transform * Vector4.{triangle.b.x, triangle.b.y, triangle.b.z, 1.0}).xyz;
            // triangle_c := (transform * Vector4.{triangle.z.x, triangle.z.y, triangle.z.z, 1.0}).xyz;
            // triangle_a := transform_point(transform, triangle.a);
            // triangle_b := transform_point(transform, triangle.b);
            // triangle_c := transform_point(transform, triangle.c);

            triangle_normal := normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));
            if dot(triangle_normal, velocity) >= 0 {
                continue
            }

            triangle_vertices: KodaVertices(3);
            triangle_vertices.count = 3;
            triangle_vertices[0] = triangle.a;
            triangle_vertices[1] = triangle.b;
            triangle_vertices[2] = triangle.c;
            if solve_gjk(triangle_vertices, a_vertices, resolve_completely = false) {
                velocity_ray.p = get_support(a_vertices, -triangle_normal);
                triangle_plane := create_plane3(triangle.a, triangle_normal);

                intersection, hit, t := intersection(triangle_plane, velocity_ray);
                if hit {
                    // have to check hit because t could be negative
                    delta := t / velocity_length * dt;
                    if delta < minimum_delta {
                        minimum_delta = delta;
                        minimum_normal = triangle_normal;
                        minimum_triangle = triangle;
                    }
                }
            }
        }
    }

    return minimum_delta, minimum_normal;
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
