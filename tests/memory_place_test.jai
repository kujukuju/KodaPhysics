main :: () {
    v1: Simplex;
    v1.a = .{1, 2, 3};
    v1.b = .{4, 5, 6};
    v1.c = .{7, 8, 9};
    v1.d = .{10, 11, 12};
    v1.count = 4;

    v1.bcd = v1.abc;
    print("v1 is %\n", v1);
}

Simplex :: struct {
    a: Vector3;
    b: Vector3;
    c: Vector3;
    d: Vector3;

    count: int;

#place a;
    abc: [3] Vector3 = ---;
#place b;
    bcd: [3] Vector3 = ---;
}

#import "Math";
#import "Basic";