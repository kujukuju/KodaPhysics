
// Manifold :: struct {

// }

solve_epa :: (a: KodaVertices, b: KodaVertices) -> Vector3, bool {
    simplex: KodaVertices(4);
    // TODO smarter starting direction
    collide := solve_gjk(a, b, .{0, 0, -1}, *simplex);
    if !collide {
        return .{}, false;
    }

    polytope: KodaVertices(MAX_VERTICES * 3);
    for i: 0..simplex.count - 1 {
        add(*polytope, simplex[i]);
    }

    faces: KodaIndices;
    add(*faces, 0, 1, 2);
    add(*faces, 0, 3, 1);
    add(*faces, 0, 2, 3);
    add(*faces, 1, 3, 2);

    normals: KodaComplexNormals;
    min_face: int;
    normals, min_face = get_face_normals(polytope, faces);

    min_normal: Vector3;
    min_distance := FLOAT32_MAX;
    while min_distance == FLOAT32_MAX {
        min_normal = normals[min_face].xyz;
		min_distance = normals[min_face].w;
 
		support := get_support(a, b, min_normal);
		s_distance := dot(min_normal, support);
 
        // TODO better epsilon?
		if abs(s_distance - min_distance) > 0.001 {
			min_distance = FLOAT32_MAX;
            unique_edges: KodaIndices;

            check_indices(polytope, faces);

            i := 0;
            while i < normals.count {
				if same_direction(normals[i].xyz, support) {
					f := i * 3;

					add_if_unique_edge(*unique_edges, faces, f, f + 1);
					add_if_unique_edge(*unique_edges, faces, f + 1, f + 2);
					add_if_unique_edge(*unique_edges, faces, f + 2, f);

                    check_indices(polytope, unique_edges);

					faces[f + 2] = faces[faces.count - 1];
                    faces.count -= 1;
					faces[f + 1] = faces[faces.count - 1];
                    faces.count -= 1;
					faces[f] = faces[faces.count - 1];
                    faces.count -= 1;

                    check_indices(polytope, faces);

					normals[i] = normals[normals.count - 1];
                    normals.count -= 1;
				} else {
                    i += 1;
                }
			}

            check_indices(polytope, faces);

            new_faces: KodaIndices;
            for i: 0..unique_edges.count / 2 - 1 {
                edge_index1 := unique_edges[i * 2];
                edge_index2 := unique_edges[i * 2 + 1];
                add(*new_faces, edge_index1);
                add(*new_faces, edge_index2);
                add(*new_faces, xx polytope.count);
            }

            add(*polytope, support);

            print("% %\n", polytope.count, new_faces.count);

            check_indices(polytope, new_faces);

            new_normals: KodaComplexNormals;
            new_min_face: int;
            new_normals, new_min_face = get_face_normals(polytope, new_faces);
            old_min_distance := FLOAT32_MAX;
            for i: 0..normals.count - 1 {
                if normals[i].w < old_min_distance {
					old_min_distance = normals[i].w;
					min_face = i;
				}
			}
 
			if new_normals[new_min_face].w < old_min_distance {
				min_face = new_min_face + normals.count;
			}
 
            for i: 0..new_faces.count - 1 {
                add(*faces, new_faces[i]);
            }
            for i: 0..new_normals.count - 1 {
                add(*normals, new_normals[i]);
            }

            check_indices(polytope, new_faces);
			// faces.insert(faces  .end(), newFaces  .begin(), newFaces  .end());
			// normals.insert(normals.end(), newNormals.begin(), newNormals.end());
		}
    }

    return min_normal * min_distance, true;
}

#scope_file

check_indices :: (polytope: KodaVertices($N), indices: KodaIndices($O)) {
    for i: 0..indices.count - 1 {
        assert(indices[i] < polytope.count, "Face index is not within the vertex range.");
    }
}

Triangle :: struct {
    p1: Vector3;
    p2: Vector3;
    p3: Vector3;
}

get_face_normals :: (polytope: KodaVertices($N), faces: KodaIndices($O)) -> KodaComplexNormals(O), int {
    normals: KodaComplexNormals;
    min_triangle := 0;
    min_distance := FLOAT32_MAX;

    for i: 0..faces.count / 3 - 1 {
        a := polytope[faces[i * 3]];
        b := polytope[faces[i * 3 + 1]];
        c := polytope[faces[i * 3 + 2]];

        normal := normalize(cross(b - a, c - a));
        distance := dot(normal, a);

        // TODO as long as I know the winding I dont think this is necessary, right?
        if distance < 0 {
            normal *= -1;
            distance *= -1;
        }

        add(*normals, .{normal.x, normal.y, normal.z, distance});

        if distance < min_distance {
            min_triangle = i;
            min_distance = distance;
        }
    }

	return normals, min_triangle;
}

add_if_unique_edge :: (edges: *KodaIndices, faces: KodaIndices, a: int, b: int) {
    for i: 0..edges.count / 2 - 1 {
        first := edges.fixed[i * 2];
        second := edges.fixed[i * 2 + 1];

        if first == faces[b] && second == faces[a] {
            // TODO could do ordered_remove(edges, i * 2, 2); and cut it down to a single loop
            ordered_remove(edges, i * 2);
            ordered_remove(edges, i * 2);

            return;
        }
    }

    add(edges, faces[a], faces[b]);
}

get_furthest_point :: (vertices: KodaVertices($N), direction: Vector3) -> Vector3 {
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

same_direction :: (direction: Vector3, ao: Vector3) -> bool {
    return dot(direction, ao) > 0;
}


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