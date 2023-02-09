// directional triangle resolution

// returns the resolve vector, and the resolved collision point
solve_dtr :: (triangle: Triangle3, mesh: KodaVertices, direction: Vector3 = .{0, 0, 0}) -> Vector3, Vector3 {
    abc_down := cross(triangle.c - triangle.a, triangle.b - triangle.a);
    if abs(length_squared(abc_down)) < EPSILON {
        return .{}, .{};
    }

    resolve_dir: Vector3;
    if length_squared(direction) == 0 {
        resolve_dir = -abc_down;
    } else {
        resolve_dir = direction;
    }

    ab_inner := cross(triangle.b - triangle.a, -resolve_dir);
    bc_inner := cross(triangle.c - triangle.b, -resolve_dir);
    ca_inner := cross(triangle.a - triangle.c, -resolve_dir);

    largest_resolve_vertex: Vector3;
    a_dot := dot(triangle.a, resolve_dir);
    b_dot := dot(triangle.b, resolve_dir);
    c_dot := dot(triangle.c, resolve_dir);
    if a_dot >= b_dot && a_dot >= c_dot {
        largest_resolve_vertex = triangle.a;
    } else if b_dot >= a_dot && b_dot >= c_dot {
        largest_resolve_vertex = triangle.b;
    } else {
        largest_resolve_vertex = triangle.c;
    }

    largest_dot: float;
    largest_vertex: Vector3;

    for i: 0..mesh.count - 1 {
        v := mesh[i];

        inside_volume := dot(v - triangle.a, abc_down) > 0;
        inside_volume = inside_volume && dot(ab_inner, v - triangle.a) > 0;
        inside_volume = inside_volume && dot(bc_inner, v - triangle.b) > 0;
        inside_volume = inside_volume && dot(ca_inner, v - triangle.c) > 0;

        if inside_volume {
            current_dot := dot(v - largest_resolve_vertex, -resolve_dir);
            if current_dot > largest_dot {
                largest_dot = current_dot;
                largest_vertex = v;
            }
        }
    }

    if largest_dot > 0 {
        triangle_plane := create_plane3(triangle.a, normalize(abc_down));
        ray := create_ray3(largest_vertex, normalize(resolve_dir));

        point := intersection(triangle_plane, ray, allow_negative = true);
        return point - largest_vertex, point;
    }

    return .{}, .{};
}

#scope_file

EPSILON :: 0.000001;

#scope_export
