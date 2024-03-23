main :: () {
    vertices: BigStruct;
    vertices_pointer: *BigStruct = *vertices;

    start1 := seconds_since_init();
    total1 := gjk_or_something(<< vertices_pointer);
    print("Normal GJK took % ms\n", (seconds_since_init() - start1) * 1000);

    print("%\n", total1);

    start2 := seconds_since_init();
    total2 := gjk_or_something_pointer(vertices_pointer);
    print("Pointer GJK took % ms\n", (seconds_since_init() - start2) * 1000);

    print("%\n", total2);
}

BigStruct :: struct (N: int = 4096) {
    vertices: [N] Vector3;
}

gjk_or_something :: (vertices: BigStruct($N)) -> float64 {
    // this function never takes a pointer to vertices

    exponential_total_x: float64;
    for i: 0..9999 {
        exponential_total_x += get_total(vertices);        
    }
    return exponential_total_x;
}

get_total :: (vertices: BigStruct($N)) -> float64 {
    // this function also never takes a pointer to vertices

    pointer_vertices := *vertices;

    total_x: float64;
    for v: pointer_vertices.vertices {
        total_x += v.x;
    }
    return total_x;
}

gjk_or_something_pointer :: (vertices: *BigStruct($N)) -> float64 {
    exponential_total_x: float64;
    for i: 0..9999 {
        exponential_total_x += get_total_pointer(vertices);        
    }
    return exponential_total_x;
}

get_total_pointer :: (vertices: *BigStruct($N)) -> float64 {
    total_x: float64;
    for v: vertices.vertices {
        total_x += v.x;
    }
    return total_x;
}

#import "Basic";
#import "Math";