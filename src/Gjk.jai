
solve_gjk :: (a: KodaVertices, b: KodaVertices, dir: Vector3) -> bool {
    simplex: KodaVertices(4);
    return solve_gjk(a, b, dir, *simplex);
}

solve_gjk :: (a: KodaVertices, b: KodaVertices, dir: Vector3, simplex: *KodaVertices(4)) -> bool {
    direction := dir;
    support := get_support(a, b, direction);

    simplex.count = 1;
    simplex.fixed[0] = support;

    direction = -support;

    // I think this gets hung on non-convex shapes
    while true {
        support = get_support(a, b, direction);
        
        if dot(support, direction) <= 0 {
            return false;
        }

        add_front(simplex, support);

        if next_simplex(simplex, *direction) {
            return true;
        }
    }

    return false;
}

solve_gjk :: (a: KodaVertices, b: Vector3, dir: Vector3) -> bool {
    simplex: KodaVertices(4);
    return solve_gjk(a, b, dir, *simplex);
}

solve_gjk :: (a: KodaVertices, b: Vector3, dir: Vector3, simplex: *KodaVertices(4)) -> bool {
    direction := dir;
    support := get_support(a, b, direction);

    simplex.count = 1;
    simplex.fixed[0] = support;

    direction = -support;

    // I think this gets hung on non-convex shapes
    while true {
        support = get_support(a, b, direction);
        
        if dot(support, direction) <= 0 {
            return false;
        }

        add_front(simplex, support);

        if next_simplex(simplex, *direction) {
            return true;
        }
    }

    return false;
}

#scope_file

get_furthest_point :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {
    if vertices.count == 0 {
        return .{};
    }

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

get_support :: (a: KodaVertices, b: KodaVertices, direction: Vector3) -> Vector3 {
    return get_furthest_point(a, direction) - get_furthest_point(b, -direction);
}

get_support :: (a: KodaVertices, b: Vector3, direction: Vector3) -> Vector3 {
    return get_furthest_point(a, direction) - b;
}

next_simplex :: (simplex: *KodaVertices(4), direction: *Vector3) -> bool {
    if simplex.count == {
        case 2;
            return next_simplex_line(simplex, direction);

        case 3;
            return next_simplex_triangle(simplex, direction);

        case 4;
            return next_simplex_tetrahedron(simplex, direction);
    }

    return false;
}

next_simplex_line :: (simplex: *KodaVertices(4), direction: *Vector3) -> bool {
    a := simplex.fixed[0];
    b := simplex.fixed[1];

    ab := b - a;
    ao := -a;

    if same_direction(ab, ao) {
        << direction = cross(cross(ab, ao), ab);
    } else {
        simplex.fixed[0] = a;
        simplex.count = 1;
        << direction = ao;
    }

    return false;
}

next_simplex_triangle :: (simplex: *KodaVertices(4), direction: *Vector3) -> bool {
    a := simplex.fixed[0];
    b := simplex.fixed[1];
    c := simplex.fixed[2];

    ab := b - a;
    ac := c - a;
    ao := -a;

    abc := cross(ab, ac);

    if same_direction(cross(abc, ac), ao) {
        if same_direction(ac, ao) {
            simplex.fixed[0] = a;
            simplex.fixed[1] = c;
            simplex.count = 2;
            << direction = cross(cross(ac, ao), ac);
        } else {
            simplex.fixed[0] = a;
            simplex.fixed[1] = b;
            simplex.count = 2;
            return next_simplex_line(simplex, direction);
        }
    } else {
        if same_direction(cross(ab, abc), ao) {
            simplex.fixed[0] = a;
            simplex.fixed[1] = b;
            simplex.count = 2;
            return next_simplex_line(simplex, direction);
        } else {
            if same_direction(abc, ao) {
                << direction = abc;
            } else {
                simplex.fixed[0] = a;
                simplex.fixed[1] = c;
                simplex.fixed[2] = b;
                simplex.count = 3;
                << direction = -abc;
            }
        }
    }

    return false;
}

next_simplex_tetrahedron :: (simplex: *KodaVertices(4), direction: *Vector3) -> bool {
    a := simplex.fixed[0];
    b := simplex.fixed[1];
    c := simplex.fixed[2];
    d := simplex.fixed[3];

    ab := b - a;
    ac := c - a;
    ad := d - a;
    ao := -a;

    abc := cross(ab, ac);
    acd := cross(ac, ad);
    adb := cross(ad, ab);

    if same_direction(abc, ao) {
        simplex.fixed[0] = a;
        simplex.fixed[1] = b;
        simplex.fixed[2] = c;
        simplex.count = 3;
        return next_simplex_triangle(simplex, direction);
    } else if same_direction(acd, ao) {
        simplex.fixed[0] = a;
        simplex.fixed[1] = c;
        simplex.fixed[2] = d;
        simplex.count = 3;
        return next_simplex_triangle(simplex, direction);
    } else if same_direction(adb, ao) {
        simplex.fixed[0] = a;
        simplex.fixed[1] = d;
        simplex.fixed[2] = b;
        simplex.count = 3;
        return next_simplex_triangle(simplex, direction);
    }

    return true;
}

same_direction :: (direction: Vector3, ao: Vector3) -> bool {
    return dot(direction, ao) > 0;
}

add_front :: (simplex: *KodaVertices(4), v: Vector3) {
    simplex.fixed[3] = simplex.fixed[2];
    simplex.fixed[2] = simplex.fixed[1];
    simplex.fixed[1] = simplex.fixed[0];
    simplex.fixed[0] = v;
    simplex.count = min(simplex.count + 1, 4);
}