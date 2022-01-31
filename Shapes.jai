KodaShape :: struct {
    type: KodaShapeType;
}

KodaPolygon :: struct {
    using shape: KodaShape;

    count: int;
    vertices: [MAX_VERTICES] Vector2;
    external: [MAX_VERTICES] bool;
}

KodaRectangle :: struct {
    using shape: KodaShape;

    dimensions: Vector2;
}

KodaShapeType :: enum {
    Polygon;
    Rectangle;
}

create_polygon :: (vertices: [..] Vector2) -> KodaPolygon {
    polygon: KodaPolygon;
    polygon.type = KodaShapeType.Polygon;
    polygon.count = vertices.count;

    for i: 0..vertices.count - 1 {
        polygon.vertices[i] = vertices[i];
    }

    // why doesnt this work
    // memcpy(*polygon.vertices, *vertices, vertices.count * size_of(Vector2));

    return polygon;
}

create_rectangle :: (width: float, height: float) -> KodaRectangle {
    rectangle: KodaRectangle = ---;
    rectangle.type = KodaShapeType.Rectangle;
    rectangle.dimensions.x = width;
    rectangle.dimensions.y = height;

    return rectangle;
}

get_polygon_view :: (shape: *KodaPolygon) -> [] Vector2 {
    // I could maybe do some scuffed shit like delcaring a stack array of size MAX_VERTICES then just changing the count value... no memory leaks but weird
    view: [] Vector2;
    view.data = shape.vertices.data;
    view.count = shape.count;

    return view;
}

MAX_VERTICES :: 32;

#scope_file