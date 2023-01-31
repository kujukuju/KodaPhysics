
solve_epa :: (a: KodaVertices, b: KodaVertices) -> bool, Vector3 {
    simplex: KodaVertices(4);
    inside := solve_gjk(a, b, .{}, *simplex);
    if !inside {
        return false, .{};
    }

    MAX_ITERATIONS :: 64;
    MAX_FACES :: 64;
    MAX_LOOSE_EDGES :: 32;
    TOLERANCE :: 0.0001;

    faces: [64][4] Vector3;
    faces[0][0] = simplex.fixed[0];
    faces[0][1] = simplex.fixed[1];
    faces[0][2] = simplex.fixed[2];
    faces[0][3] = normalize(cross(simplex.fixed[1] - simplex.fixed[0], simplex.fixed[2] - simplex.fixed[0])); // abc
    faces[1][0] = simplex.fixed[0];
    faces[1][1] = simplex.fixed[2];
    faces[1][2] = simplex.fixed[3];
    faces[1][3] = normalize(cross(simplex.fixed[2] - simplex.fixed[0], simplex.fixed[3] - simplex.fixed[0])); // acd
    faces[2][0] = simplex.fixed[0];
    faces[2][1] = simplex.fixed[3];
    faces[2][2] = simplex.fixed[1];
    faces[2][3] = normalize(cross(simplex.fixed[3] - simplex.fixed[0], simplex.fixed[1] - simplex.fixed[0])); // adb
    faces[3][0] = simplex.fixed[1];
    faces[3][1] = simplex.fixed[3];
    faces[3][2] = simplex.fixed[2];
    faces[3][3] = normalize(cross(simplex.fixed[3] - simplex.fixed[1], simplex.fixed[2] - simplex.fixed[1])); // bdc

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

solve_epa_open :: (a: KodaVertices, b: KodaVertices) -> bool, Vector3 {
    simplex: KodaVertices(4);
    inside := solve_gjk(a, b, .{}, *simplex);

    MAX_ITERATIONS :: 64;
    MAX_FACES :: 64;
    MAX_LOOSE_EDGES :: 32;
    TOLERANCE :: 0.0001;

    faces: [64][4] Vector3;
    faces[0][0] = simplex.fixed[0];
    faces[0][1] = simplex.fixed[1];
    faces[0][2] = simplex.fixed[2];
    faces[0][3] = normalize(cross(simplex.fixed[1] - simplex.fixed[0], simplex.fixed[2] - simplex.fixed[0])); // abc
    faces[1][0] = simplex.fixed[0];
    faces[1][1] = simplex.fixed[2];
    faces[1][2] = simplex.fixed[3];
    faces[1][3] = normalize(cross(simplex.fixed[2] - simplex.fixed[0], simplex.fixed[3] - simplex.fixed[0])); // acd
    faces[2][0] = simplex.fixed[0];
    faces[2][1] = simplex.fixed[3];
    faces[2][2] = simplex.fixed[1];
    faces[2][3] = normalize(cross(simplex.fixed[3] - simplex.fixed[0], simplex.fixed[1] - simplex.fixed[0])); // adb
    faces[3][0] = simplex.fixed[1];
    faces[3][1] = simplex.fixed[3];
    faces[3][2] = simplex.fixed[2];
    faces[3][3] = normalize(cross(simplex.fixed[3] - simplex.fixed[1], simplex.fixed[2] - simplex.fixed[1])); // bdc

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
            return inside, faces[closest_face][3] * dot(p, search_dir);
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

    return inside, faces[closest_face][3] * dot(faces[closest_face][0], faces[closest_face][3]);
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

// check_indices :: (polytope: KodaVertices($N), indices: KodaIndices($O)) {
//     for i: 0..indices.count - 1 {
//         assert(indices[i] < polytope.count, "Face index is not within the vertex range.");
//     }
// }

// Triangle :: struct {
//     p1: Vector3;
//     p2: Vector3;
//     p3: Vector3;
// }

// get_face_normals :: (polytope: KodaVertices($N), faces: KodaIndices($O)) -> KodaComplexNormals(O), int {
//     normals: KodaComplexNormals;
//     min_triangle := 0;
//     min_distance := FLOAT32_MAX;

//     for i: 0..faces.count / 3 - 1 {
//         a := polytope[faces[i * 3]];
//         b := polytope[faces[i * 3 + 1]];
//         c := polytope[faces[i * 3 + 2]];

//         normal := normalize(cross(b - a, c - a));
//         distance := dot(normal, a);

//         // TODO as long as I know the winding I dont think this is necessary, right?
//         if distance < 0 {
//             normal *= -1;
//             distance *= -1;
//         }

//         add(*normals, .{normal.x, normal.y, normal.z, distance});

//         if distance < min_distance {
//             min_triangle = i;
//             min_distance = distance;
//         }
//     }

// 	return normals, min_triangle;
// }

// add_if_unique_edge :: (edges: *KodaIndices, faces: KodaIndices, a: int, b: int) {
//     for i: 0..edges.count / 2 - 1 {
//         first := edges.fixed[i * 2];
//         second := edges.fixed[i * 2 + 1];

//         if first == faces[b] && second == faces[a] {
//             // TODO could do ordered_remove(edges, i * 2, 2); and cut it down to a single loop
//             ordered_remove(edges, i * 2);
//             ordered_remove(edges, i * 2);

//             return;
//         }
//     }

//     add(edges, faces[a], faces[b]);
// }

// get_furthest_point :: (vertices: KodaVertices($N), direction: Vector3) -> Vector3 {
//     if vertices.count == 0 {
//         return .{};
//     }

//     largest_dot := dot(vertices[0], direction);
//     largest_vertex := vertices[0];
//     for i: 1..vertices.count - 1 {
//         vertex := vertices[i];

//         current_dot := dot(vertex, direction);
//         if current_dot > largest_dot {
//             largest_dot = current_dot;
//             largest_vertex = vertex;
//         }
//     }

//     return largest_vertex;
// }

// get_support :: (a: KodaVertices, b: KodaVertices, direction: Vector3) -> Vector3 {
//     return get_furthest_point(a, direction) - get_furthest_point(b, -direction);
// }

// same_direction :: (direction: Vector3, ao: Vector3) -> bool {
//     return dot(direction, ao) > 0;
// }


//     polytope: [..] Vector3;
//     normals: [..] Vector3;

//     // calculate normals
    

//     min_normal: Vector3;
//     min_distance := FLOAT32_MAX;
//     while min_distance == FLOAT32_MAX {
//         min_normal = normals[min_face].xyz;
//         min_distance = normals[min_face].w;

//         support := get_support(a, b, min_normal);
//         s_distance := dot(min_normal, support);

//         if abs(s_distance - min_distance) > 0.000001 {
//             min_distance = FLOAT32_MAX;

//             unique_edges: [..] 
//         }
//     }
// }

// #scope_file