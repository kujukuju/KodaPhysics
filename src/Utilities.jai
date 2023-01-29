BodyVertices :: struct {
    vertices: [MAX_VERTICES] Vector2;
    count: int;
}

operator [] :: (vertices: BodyVertices, index: int) -> Vector2 {
    return vertices.vertices[index];
}

operator + :: (vertices: BodyVertices, offset: Vector2) -> BodyVertices {
    new := vertices;
    for i: 0..new.count - 1 {
        new.vertices[i] += offset;
    }

    return new;
}

view :: (vertices: *BodyVertices) -> [] Vector2 {
    view: [] Vector2;
    view.data = vertices.vertices.data;
    view.count = vertices.count;
    return view;
}

#scope_file

#import "Basic";
#import "JaiMath";