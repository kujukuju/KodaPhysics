
solve_epa :: (a: KodaVertices, b: KodaVertices, simplex: Simplex) -> bool, Vector3 {
    MAX_ITERATIONS :: 64;
    MAX_FACES :: 64;
    MAX_LOOSE_EDGES :: 32;
    TOLERANCE :: 0.0001;

    faces: [64][4] Vector3;
    faces[0][0] = simplex.a;
    faces[0][1] = simplex.b;
    faces[0][2] = simplex.c;
    faces[0][3] = normalize(cross(simplex.b - simplex.a, simplex.c - simplex.a)); // abc
    faces[1][0] = simplex.a;
    faces[1][1] = simplex.c;
    faces[1][2] = simplex.d;
    faces[1][3] = normalize(cross(simplex.c - simplex.a, simplex.d - simplex.a)); // acd
    faces[2][0] = simplex.a;
    faces[2][1] = simplex.d;
    faces[2][2] = simplex.b;
    faces[2][3] = normalize(cross(simplex.d - simplex.a, simplex.b - simplex.a)); // adb
    faces[3][0] = simplex.b;
    faces[3][1] = simplex.d;
    faces[3][2] = simplex.c;
    faces[3][3] = normalize(cross(simplex.d - simplex.b, simplex.c - simplex.b)); // bdc

    num_faces := 4;
    closest_face: int;

    for i: 0..MAX_ITERATIONS - 1 {
        min_dist := dot(faces[0][0], faces[0][3]);
        closest_face = 0;
        for i: 1..num_faces - 1 {
            dist := dot(faces[i][0], faces[i][3]);
            if dist < min_dist {
                min_dist = dist;
                closest_face = i;
            }
        }

        search_dir := faces[closest_face][3];
        p := get_support(b, search_dir) - get_support(a, -search_dir);

        if dot(p, search_dir) - min_dist < TOLERANCE {
            return true, faces[closest_face][3] * dot(p, search_dir);
        }

        loose_edges: [MAX_LOOSE_EDGES][2] Vector3;
        num_loose_edges := 0;

        i := 0;
        while i < num_faces {
            if dot(faces[i][3], p - faces[i][0]) > 0 {
                for j: 0..2 {
                    current_edge: [2] Vector3;
                    current_edge[0] = faces[i][j];
                    current_edge[1] = faces[i][(j + 1) % 3];
                    found_edge := false;

                    for k: 0..num_loose_edges - 1 {
                        if loose_edges[k][1] == current_edge[0] && loose_edges[k][0] == current_edge[1] {
                            loose_edges[k][0] = loose_edges[num_loose_edges - 1][0];
                            loose_edges[k][1] = loose_edges[num_loose_edges - 1][1];
                            num_loose_edges -= 1;
                            found_edge = true;
                            break;
                        }
                    }

                    if !found_edge {
                        if num_loose_edges >= MAX_LOOSE_EDGES {
                            break;
                        }

                        loose_edges[num_loose_edges][0] = current_edge[0];
                        loose_edges[num_loose_edges][1] = current_edge[1];
                        num_loose_edges += 1;
                    }
                }

                faces[i] = faces[num_faces - 1];
                num_faces -= 1;
            } else {
                i += 1;
            }
        }

        for i: 0..num_loose_edges - 1 {
            if num_faces >= MAX_FACES {
                break;
            }

            faces[num_faces][0] = loose_edges[i][0];
            faces[num_faces][1] = loose_edges[i][1];
            faces[num_faces][2] = p;
            faces[num_faces][3] = normalize(cross(loose_edges[i][0] - loose_edges[i][1], loose_edges[i][0] - p));

            BIAS :: 0.000001;
            if dot(faces[num_faces][0], faces[num_faces][3]) + BIAS < 0 {
                temp_value := faces[num_faces][0];
                faces[num_faces][0] = faces[num_faces][1];
                faces[num_faces][1] = temp_value;
                faces[num_faces][3] = -faces[num_faces][3];
            }

            num_faces += 1;
        }
    }

    return true, faces[closest_face][3] * dot(faces[closest_face][0], faces[closest_face][3]);
}

#scope_file

get_support :: (vertices: KodaVertices, direction: Vector3) -> Vector3 {
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
