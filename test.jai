#import "Basic";
#import "Math";
#import "Random";
#load "BoundingTree.jai";

Data :: struct (ValueType: Type) {
    aabb: AABB;
    data: ValueType;
}

main :: () {
    bounding_tree := BoundingTree(int).create();
    defer BoundingTree(int).free(bounding_tree);

    entries: [..] Data(int);
    defer array_free(entries);

    for i: 0..999 {
        x := random_get_zero_to_one() * 1000.0;
        y := random_get_zero_to_one() * 1000.0;
        w := random_get_zero_to_one() * 9.0 + 1.0;
        h := random_get_zero_to_one() * 9.0 + 1.0;

        aabb: AABB = ---;
        aabb.lower = xy(x, y) - xy(w / 2.0, h / 2.0);
        aabb.upper = xy(x, y) + xy(w / 2.0, h / 2.0);

        data: Data(int) = ---;
        data.aabb = aabb;
        data.data = i;

        create_proxy(*bounding_tree, aabb, i);
        array_add(*entries, data);
    }

    for i: 0..100000 {
        x := random_get_zero_to_one() * 1000.0;
        y := random_get_zero_to_one() * 1000.0;
        w := random_get_zero_to_one() * 499.0 + 1.0;
        h := random_get_zero_to_one() * 499.0 + 1.0;

        aabb: AABB = ---;
        aabb.lower = xy(x, y) - xy(w / 2.0, h / 2.0);
        aabb.upper = xy(x, y) + xy(w / 2.0, h / 2.0);

        found: [..] bool;
        defer array_free(found);
        expected: [..] int;
        defer array_free(expected);
        for a: 0..entries.count - 1 {
            entry := entries[a];
            if intersection(aabb, entry.aabb) {
                // print("original overlap with %\n", a);
                array_add(*found, false);
                array_add(*expected, a);
            }
        }

        Extra :: struct {
            expected: [..] int;
            found: [..] bool;
        }
        extras: Extra;
        extras.expected = expected;
        extras.found = found;
        query(bounding_tree, aabb, (data: int, extras: Extra) -> bool {
            // print("overlapping %\n", data);
            for a: 0..extras.expected.count - 1 {
                if data == extras.expected[a] {
                    extras.found[a] = true;
                }
            }

            return true;
        }, extras);

        for f: found {
            if !f {
                print("NOT VALID\n");
            }
        }

        // print("FOUND: %\n%\n", expected, found);
    }

    print("VALID\n");
}
