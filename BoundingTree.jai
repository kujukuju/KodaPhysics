// TODO a lot of these methods that accept pointers dont need to because were just modifying the underlying array
// TODO bounding tree should accept a default size and then after passing that size becomes memory

BoundingTree :: struct (ValueType: Type) {
    root: int = NULL_NODE;

    nodes: [] TreeNode(ValueType) = ---;
    node_count: int;
    node_capacity: int = 16;

    free_list: int;

    insertion_count: int;

    allocator: Allocator = ---;

    create :: (allocator: Allocator = context.allocator) -> BoundingTree(ValueType) {
        tree: BoundingTree(ValueType);
        tree.nodes = NewArray(tree.node_capacity, TreeNode(ValueType), false, allocator);
        tree.allocator = allocator;
        
        for i: 0..tree.node_capacity - 2 {
            tree.nodes[i].next = i + 1;
            tree.nodes[i].height = -1;
        }
        tree.nodes[tree.node_capacity - 1].next = NULL_NODE;
        tree.nodes[tree.node_capacity - 1].height = -1;

        return tree;
    }

    free :: (tree: BoundingTree) {
        free(tree.nodes.data, tree.allocator);
    }
}

TreeNode :: struct (ValueType: Type) {
    aabb: AABB;

    user_data: ValueType;

    union {
        parent: int;
        next: int = NULL_NODE;
    }

    child1: int;
    child2: int;

    height: int;

    moved: bool;
}

Segment :: struct {
    p1: Vector2;
    p2: Vector2;
}

AABB :: struct {
    lower: Vector2;
    upper: Vector2;
}

is_leaf :: (node: TreeNode) -> bool {
    return node.child1 == NULL_NODE;
}

create_proxy :: (tree: *BoundingTree($T), aabb: AABB, user_data: T) -> int {
    proxy_id := allocate_node(tree);

    r := Vector2.{AABB_EXTENSION, AABB_EXTENSION};
    tree.nodes[proxy_id].aabb.lower = aabb.lower - r;
    tree.nodes[proxy_id].aabb.upper = aabb.upper + r;
    tree.nodes[proxy_id].user_data = user_data;
    tree.nodes[proxy_id].height = 0;
    tree.nodes[proxy_id].moved = true;

    insert_leaf(tree, proxy_id);

    return proxy_id;
}

destroy_proxy :: (tree: *BoundingTree, proxy_id: int) {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	assert(is_leaf(tree.nodes[proxy_id]));

	remove_leaf(tree, proxy_id);
	free_node(tree, proxy_id);
}

move_proxy :: (tree: *BoundingTree, proxy_id: int, aabb: AABB, displacement: Vector2) -> bool {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	assert(is_leaf(tree.nodes[proxy_id]));

	// Extend AABB
	fat_aabb: AABB = ---;
	r := Vector2.{AABB_EXTENSION, AABB_EXTENSION};
    fat_aabb.lower = aabb.lower - r;
    fat_aabb.upper = aabb.upper + r;

	// Predict AABB movement
    d := AABB_MULTIPLIER * displacement;

	if d.x < 0 {
		fat_aabb.lower.x += d.x;
	} else {
		fat_aabb.upper.x += d.x;
	}

	if d.y < 0 {
		fat_aabb.lower.y += d.y;
	} else {
		fat_aabb.upper.y += d.y;
	}

    tree_aabb := tree.nodes[proxy_id].aabb;
	if contains(tree_aabb, aabb) {
		// The tree AABB still contains the object, but it might be too large.
		// Perhaps the object was moving fast but has since gone to sleep.
		// The huge AABB is larger than the new fat AABB.
		huge_aabb: AABB = ---;
		huge_aabb.lower = fat_aabb.lower - 4.0 * r;
		huge_aabb.upper = fat_aabb.upper + 4.0 * r;

		if contains(huge_aabb, tree_aabb) {
			// The tree AABB contains the object AABB and the tree AABB is
			// not too large. No tree update needed.
			return false;
		}

		// Otherwise the tree AABB is huge and needs to be shrunk
	}

	remove_leaf(tree, proxy_id);

	tree.nodes[proxy_id].aabb = fat_aabb;

	insert_leaf(tree, proxy_id);

	tree.nodes[proxy_id].moved = true;

	return true;
}

get_user_data :: inline (tree: BoundingTree($T), proxy_id: int) -> T {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	return tree.nodes[proxy_id].user_data;
}

was_moved :: inline (tree: BoundingTree, proxy_id: int) -> bool {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	return tree.nodes[proxy_id].moved;
}

clear_moved :: inline (tree: *BoundingTree, proxy_id: int) {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	tree.nodes[proxy_id].moved = false;
}

get_fat_aabb :: inline (tree: BoundingTree, proxy_id: int) -> AABB {
	assert(0 <= proxy_id && proxy_id < tree.node_capacity);
	return tree.nodes[proxy_id].aabb;
}

query :: inline (tree: BoundingTree($T), aabb: AABB, $$callback: (data: T, extras: $U) -> bool, extras: U) {
    stack := GrowableStack(int, 256).create();
    defer GrowableStack(int, 256).free(stack);

    push(*stack, tree.root);

	while !empty(stack) {
		node_id := pop(*stack);
		if node_id == NULL_NODE {
			continue;
		}

		if intersection(tree.nodes[node_id].aabb, aabb) {
			if is_leaf(tree.nodes[node_id]) {
				proceed := callback(tree.nodes[node_id].user_data, extras);
				if !proceed {
					return;
				}
			} else {
                push(*stack, tree.nodes[node_id].child1);
                push(*stack, tree.nodes[node_id].child2);
			}
		}
	}
}

// for the callback if the segment doesn't intersect the shape, return p1
// the return of the callback should be the ending point of the new segment
// TODO should the callback have you return a new segment to test?
ray_cast :: inline (tree: BoundingTree($T), input: Segment, $$callback: (segment: Segment, value: T, extras: $U) -> Vector2, extras: U) {
    p1 := input.p1;
	p2 := input.p2;
	r := p2 - p1;
	assert(length_squared(r) > 0.0);
	r = normalize(r);

	// v is perpendicular to the segment.
	v := cross(1.0, r);
	abs_v := abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	// maxFraction := input.maxFraction;

	// Build a bounding box for the segment.
	segment_aabb: AABB = ---;
    segment_aabb.lower = min(p1, p2);
    segment_aabb.upper = max(p1, p2);

    current_segment := input;

    stack := GrowableStack(int, 256).create();
    push(*stack, tree.m_root);

	while !empty(stack) {
        node_id := pop(*stack);
		if node_id == NULL_NODE {
			continue;
		}

		if !intersection(tree.nodes[node_id].aabb, segment_aabb) {
			continue;
		}

        if !intersection(current_segment, tree.nodes[node_id].aabb) {
            continue;
        }

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		c := get_center(tree.nodes[node_id].aabb);
		h := get_extents(tree.nodes[node_id].aabb);
		separation := abs(dot(v, p1 - c)) - dot(abs_v, h);
		if separation > 0.0 {
			continue;
		}

		if is_leaf(tree.nodes[node_id]) {
            point := callback(current_segment, tree.nodes[node_id].user_data, extras);

			if point == current_segment.p1 {
				// The client has terminated the ray cast.
				return;
			}

            // Update segment bounding box.
            current_segment.p2 = point;
            segment_aabb.lower = min(current_segment.p1, point);
            segment_aabb.upper = max(current_segment.p1, point);
		} else {
            push(*stack, tree.nodes[node_id].child1);
            push(*stack, tree.nodes[node_id].child2);
		}
	}
}

validate :: (tree: BoundingTree) {
    #if TREE_DEBUG {
        validate_structure(tree, tree.root);
        validate_metrics(tree, tree.root);

        free_count := 0;
        free_index := tree.free_list;
        while free_index != NULL_NODE {
            assert(0 <= free_index && free_index < tree.node_capacity);
            free_index = tree.nodes[free_index].next;
            free_count += 1;
        }

        assert(get_height(tree) == compute_height(tree));

        assert(tree.node_count + free_count == tree.node_capacity);
    }
}

get_height :: (tree: BoundingTree) -> int {
	if tree.root == NULL_NODE {
		return 0;
	}

	return tree.nodes[tree.root].height;
}

get_max_balance :: (tree: BoundingTree) -> int {
	max_balance := 0;

    for i: 0..tree.node_capacity - 1 {
		if tree.nodes[i].height <= 1 {
			continue;
		}

		assert(!is_leaf(tree.nodes[i]));

		child1 := tree.nodes[i].child1;
		child2 := tree.nodes[i].child2;
		balance := abs(tree.nodes[child2].height - tree.nodes[child1].height);
		max_balance = max(max_balance, balance);
	}

	return max_balance;
}

get_area_ratio :: (tree: BoundingTree) -> float {
	if tree.root == NULL_NODE {
		return 0;
	}

	root_area := get_perimeter(tree.nodes[tree.root].aabb);

	total_area := 0.0;
    for i: 0..tree.node_capacity - 1 {
		if tree.nodes[i].height < 0 {
			// Free node in pool
			continue;
		}

		total_area += get_perimeter(tree.nodes[i].aabb);
	}

	return total_area / root_area;
}

rebuild_bottom_up :: (tree: *BoundingTree($T)) {
    nodes: [] int = NewArray(tree.node_count, int, false);
	count := 0;

	// Build array of leaves. Free the rest.
    for i: 0..tree.node_capacity - 1  {
		if tree.nodes[i].height < 0 {
			// free node in pool
			continue;
		}

		if is_leaf(tree.nodes[i]) {
			tree.nodes[i].parent = NULL_NODE;
			nodes[count] = i;
			count += 1;
		} else {
			free_node(tree, i);
		}
	}

	while count > 1 {
		min_cost: float = 0xffffffff;
		imin := -1;
        jmin := -1;
        for i: 0..count - 1 {
			aabbi := tree.nodes[nodes[i]].aabb;

            for j: i + 1..count - 1 {
				aabbj := tree.nodes[nodes[j]].aabb;
				b := combine(aabbi, aabbj);
				cost := get_perimeter(b);
				if cost < min_cost {
					imin = i;
					jmin = j;
					min_cost = cost;
				}
			}
		}

		index1 := nodes[imin];
		index2 := nodes[jmin];
        child1: *TreeNode(T) = *tree.nodes[index1];
        child2: *TreeNode(T) = *tree.nodes[index2];

		parent_index := allocate_node(tree);
        parent: *TreeNode(T) = *tree.nodes[parent_index];
		parent.child1 = index1;
		parent.child2 = index2;
		parent.height = 1 + max(child1.height, child2.height);
		parent.aabb = combine(child1.aabb, child2.aabb);
		parent.parent = NULL_NODE;

		child1.parent = parent_index;
		child2.parent = parent_index;

		nodes[jmin] = nodes[count - 1];
		nodes[imin] = parent_index;
		count -= 1;
	}

	tree.root = nodes[0];
	array_free(nodes);

	validate(<< tree);
}

shift_origin :: (tree: BoundingTree, new_origin: Vector2) {
	// Build array of leaves. Free the rest.
    for i: 0..tree.node_capacity - 1 {
		tree.nodes[i].aabb.lower -= new_origin;
		tree.nodes[i].aabb.upper -= new_origin;
	}
}

intersection :: (a: AABB, b: AABB) -> bool {
    d1 := b.lower - a.upper;
    d2 := a.lower - b.upper;

	if d1.x > 0.0 || d1.y > 0.0
		return false;

	if d2.x > 0.0 || d2.y > 0.0
		return false;

	return true;
}

combine :: (aabb1: AABB, aabb2: AABB) -> AABB {
    aabb: AABB = ---;
    aabb.lower = min(aabb1.lower, aabb2.lower);
    aabb.upper = max(aabb1.upper, aabb2.upper);

    return aabb;
}

operator + :: (aabb: AABB, v: Vector2) -> AABB {
	r: AABB = ---;
	r.lower = aabb.lower + v;
	r.upper = aabb.upper + v;

	return r;
}





// GrowableStack :: struct (ValueType: Type, FakeSize: int) {
//     data: [..] ValueType;

//     create :: () -> GrowableStack(ValueType, FakeSize) {
//         stack: GrowableStack(ValueType, FakeSize);
//         return stack;
//     }

//     free :: (stack: GrowableStack) {
//         array_free(stack.data);
//     }
// }

// push :: (stack: *GrowableStack($T, $S), value: T) {
//     array_add(*stack.data, value);
// }

// pop :: (stack: *GrowableStack($T, $S)) -> T {
//     return pop(*stack.data);
// }

// get_count :: (stack: GrowableStack) -> int {
//     return stack.data.count;
// }

// empty :: (stack: GrowableStack) -> bool {
//     return stack.data.count == 0;
// }





// TODO this should be fixed, probably to remove the create method, allocating m_stack to an empty resizeable array, and creating when necessary
GrowableStack :: struct (ValueType: Type, Size: int) {
	m_stack: [] ValueType;
	m_array: [Size] ValueType;
	m_count: int;
	m_capacity: int = Size;

    allocator: Allocator = ---;

	initialized: bool = false;

    create :: (allocator: Allocator = context.allocator) -> GrowableStack(ValueType, Size) {
        stack: GrowableStack(ValueType, Size);
        stack.allocator = allocator;

        return stack;
    }

    free :: (stack: GrowableStack($T, $S)) {
		if stack.m_capacity > S {
            free(stack.m_stack.data, stack.allocator);
        }
    }
}

initialize :: (stack: *GrowableStack($T, $S)) {
	if stack.m_stack.count == 0 {
		stack.m_stack = stack.m_array;
	}
}

push :: (stack: *GrowableStack($T, $S), value: T) {
	if !stack.initialized {
		initialize(stack);
	}

    // TODO we should probably use a resizeable array if we get to this point, but checking if its already resizeable is kinda weird
    if stack.m_count == stack.m_capacity {
        old: [] T = stack.m_stack;
        stack.m_capacity *= 2;
        stack.m_stack = NewArray(stack.m_capacity, T, false, stack.allocator);
        memcpy(stack.m_stack.data, old.data, stack.m_count * size_of(T));

		// if this array is allocated, not the compile time data, free it
        if old.data != stack.m_array.data {
            free(old.data, stack.allocator);
        }
    }

    stack.m_stack[stack.m_count] = value;
    stack.m_count += 1;
}

pop :: (stack: *GrowableStack($T, $S)) -> T {
    assert(stack.m_count > 0);
    stack.m_count -= 1;
    return stack.m_stack[stack.m_count];
}

get_count :: (stack: GrowableStack) -> int {
    return stack.m_count;
}

empty :: (stack: GrowableStack) -> bool {
    return stack.m_count == 0;
}

#scope_file

allocate_node :: (tree: *BoundingTree($T)) -> int {
	// Expand the node pool as needed.
	if tree.free_list == NULL_NODE {
		assert(tree.node_count == tree.node_capacity);

		// The free list is empty. Rebuild a bigger pool.
        old_nodes: [] TreeNode(T) = tree.nodes;
        tree.node_capacity *= 2;
        tree.nodes = NewArray(tree.node_capacity, TreeNode(T), false, tree.allocator);
        memcpy(tree.nodes.data, old_nodes.data, tree.node_count * size_of(TreeNode(T)));
        free(old_nodes.data, tree.allocator);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
        for i: tree.node_count..tree.node_capacity - 2 {
			tree.nodes[i].next = i + 1;
			tree.nodes[i].height = -1;
		}
        tree.nodes[tree.node_capacity - 1].next = NULL_NODE;
        tree.nodes[tree.node_capacity - 1].height = -1;
        tree.free_list = tree.node_count;
	}

	// Peel a node off the free list.
    node_id := tree.free_list;
	tree.free_list = tree.nodes[node_id].next;
	tree.nodes[node_id].parent = NULL_NODE;
	tree.nodes[node_id].child1 = NULL_NODE;
	tree.nodes[node_id].child2 = NULL_NODE;
	tree.nodes[node_id].height = 0;
	// tree.nodes[node_id].user_data = null;
	tree.nodes[node_id].moved = false;
	tree.node_count += 1;
	return node_id;
}

free_node :: (tree: *BoundingTree, node_id: int) {
    assert(0 <= node_id && node_id < tree.node_capacity);
	assert(0 < tree.node_count);
	tree.nodes[node_id].next = tree.free_list;
	tree.nodes[node_id].height = -1;
	tree.free_list = node_id;
	tree.node_count -= 1;
}

insert_leaf :: (tree: *BoundingTree($T), leaf: int) {
    tree.insertion_count += 1;

	if tree.root == NULL_NODE {
		tree.root = leaf;
		tree.nodes[tree.root].parent = NULL_NODE;
		return;
	}

	// Find the best sibling for this node
	leaf_aabb := tree.nodes[leaf].aabb;
	index := tree.root;
	while !is_leaf(tree.nodes[index]) {
		child1 := tree.nodes[index].child1;
		child2 := tree.nodes[index].child2;

		area := get_perimeter(tree.nodes[index].aabb);

		combined_aabb := combine(tree.nodes[index].aabb, leaf_aabb);
		combined_area := get_perimeter(combined_aabb);

		// Cost of creating a new parent for this node and the new leaf
		cost := 2.0 * combined_area;

		// Minimum cost of pushing the leaf further down the tree
		inheritance_cost := 2.0 * (combined_area - area);

		// Cost of descending into child1
        cost1: float;
		if is_leaf(tree.nodes[child1]) {
			aabb := combine(leaf_aabb, tree.nodes[child1].aabb);
			cost1 = get_perimeter(aabb) + inheritance_cost;
		} else {
			aabb := combine(leaf_aabb, tree.nodes[child1].aabb);
			old_area := get_perimeter(tree.nodes[child1].aabb);
			new_area := get_perimeter(aabb);
			cost1 = new_area - old_area + inheritance_cost;
		}

		// Cost of descending into child2
		cost2: float;
		if is_leaf(tree.nodes[child2]) {
			aabb := combine(leaf_aabb, tree.nodes[child2].aabb);
			cost2 = get_perimeter(aabb) + inheritance_cost;
		} else {
			aabb := combine(leaf_aabb, tree.nodes[child2].aabb);
			old_area := get_perimeter(tree.nodes[child2].aabb);
			new_area := get_perimeter(aabb);
			cost2 = new_area - old_area + inheritance_cost;
		}

		// Descend according to the minimum cost.
		if cost < cost1 && cost < cost2 {
			break;
		}

		// Descend
		if cost1 < cost2 {
			index = child1;
		} else {
			index = child2;
		}
	}

	sibling := index;

	// Create a new parent.
	old_parent := tree.nodes[sibling].parent;
	new_parent := allocate_node(tree);
	tree.nodes[new_parent].parent = old_parent;
    uninitialized: T = ---;
	tree.nodes[new_parent].user_data = uninitialized;
	tree.nodes[new_parent].aabb = combine(leaf_aabb, tree.nodes[sibling].aabb);
	tree.nodes[new_parent].height = tree.nodes[sibling].height + 1;

	if old_parent != NULL_NODE {
		// The sibling was not the root.
		if tree.nodes[old_parent].child1 == sibling {
			tree.nodes[old_parent].child1 = new_parent;
		} else {
			tree.nodes[old_parent].child2 = new_parent;
		}

		tree.nodes[new_parent].child1 = sibling;
		tree.nodes[new_parent].child2 = leaf;
		tree.nodes[sibling].parent = new_parent;
		tree.nodes[leaf].parent = new_parent;
	} else {
		// The sibling was the root.
		tree.nodes[new_parent].child1 = sibling;
		tree.nodes[new_parent].child2 = leaf;
		tree.nodes[sibling].parent = new_parent;
		tree.nodes[leaf].parent = new_parent;
		tree.root = new_parent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = tree.nodes[leaf].parent;
	while index != NULL_NODE {
		index = balance(tree, index);

		child1 := tree.nodes[index].child1;
		child2 := tree.nodes[index].child2;

		assert(child1 != NULL_NODE);
		assert(child2 != NULL_NODE);

		tree.nodes[index].height = 1 + max(tree.nodes[child1].height, tree.nodes[child2].height);
		tree.nodes[index].aabb = combine(tree.nodes[child1].aabb, tree.nodes[child2].aabb);

		index = tree.nodes[index].parent;
	}

	// Validate();
}

remove_leaf :: (tree: *BoundingTree, leaf: int) {
    if leaf == tree.root {
		tree.root = NULL_NODE;
		return;
	}

	parent := tree.nodes[leaf].parent;
	grand_parent := tree.nodes[parent].parent;
	sibling: int;
	if tree.nodes[parent].child1 == leaf {
		sibling = tree.nodes[parent].child2;
	} else {
		sibling = tree.nodes[parent].child1;
	}

	if grand_parent != NULL_NODE {
		// Destroy parent and connect sibling to grand_parent.
		if tree.nodes[grand_parent].child1 == parent {
			tree.nodes[grand_parent].child1 = sibling;
		} else {
			tree.nodes[grand_parent].child2 = sibling;
		}
		tree.nodes[sibling].parent = grand_parent;
		free_node(tree, parent);

		// Adjust ancestor bounds.
		index := grand_parent;
		while index != NULL_NODE {
			index = balance(tree, index);

			child1 := tree.nodes[index].child1;
			child2 := tree.nodes[index].child2;

			tree.nodes[index].aabb = combine(tree.nodes[child1].aabb, tree.nodes[child2].aabb);
			tree.nodes[index].height = 1 + max(tree.nodes[child1].height, tree.nodes[child2].height);

			index = tree.nodes[index].parent;
		}
	} else {
		tree.root = sibling;
		tree.nodes[sibling].parent = NULL_NODE;
		free_node(tree, parent);
	}

	// Validate();
}

balance :: (tree: *BoundingTree($T), i_a: int) -> int {
    assert(i_a != NULL_NODE);

    a: *TreeNode(T) = *tree.nodes[i_a];
	if is_leaf(<< a) || a.height < 2 {
		return i_a;
	}

	i_b := a.child1;
	i_c := a.child2;
	assert(0 <= i_b && i_b < tree.node_capacity);
	assert(0 <= i_c && i_c < tree.node_capacity);

    b: *TreeNode(T) = *tree.nodes[i_b];
    c: *TreeNode(T) = *tree.nodes[i_c];

    balance := c.height - b.height;

	// Rotate C up
	if balance > 1 {
		i_f := c.child1;
		i_g := c.child2;
        f: *TreeNode(T) = *tree.nodes[i_f];
        g: *TreeNode(T) = *tree.nodes[i_g];
		assert(0 <= i_f && i_f < tree.node_capacity);
		assert(0 <= i_g && i_g < tree.node_capacity);

		// Swap A and C
        c.child1 = i_a;
		c.parent = a.parent;
		a.parent = i_c;

		// A's old parent should point to C
		if c.parent != NULL_NODE {
			if tree.nodes[c.parent].child1 == i_a {
				tree.nodes[c.parent].child1 = i_c;
			} else {
				assert(tree.nodes[c.parent].child2 == i_a);
				tree.nodes[c.parent].child2 = i_c;
			}
		} else {
			tree.root = i_c;
		}

		// Rotate
		if f.height > g.height {
			c.child2 = i_f;
			a.child2 = i_g;
			g.parent = i_a;
			a.aabb = combine(b.aabb, g.aabb);
			c.aabb = combine(a.aabb, f.aabb);

			a.height = 1 + max(b.height, g.height);
			c.height = 1 + max(a.height, f.height);
		} else {
			c.child2 = i_g;
			a.child2 = i_f;
			f.parent = i_a;
			a.aabb = combine(b.aabb, f.aabb);
			c.aabb = combine(a.aabb, g.aabb);

			a.height = 1 + max(b.height, f.height);
			c.height = 1 + max(a.height, g.height);
		}

		return i_c;
	}
	
	// Rotate B up
	if balance < -1 {
		i_d := b.child1;
		i_e := b.child2;
        d: *TreeNode(T) = *tree.nodes[i_d];
        e: *TreeNode(T) = *tree.nodes[i_e];
		assert(0 <= i_d && i_d < tree.node_capacity);
		assert(0 <= i_e && i_e < tree.node_capacity);

		// Swap A and B
		b.child1 = i_a;
		b.parent = a.parent;
		a.parent = i_b;

		// A's old parent should point to B
		if b.parent != NULL_NODE {
			if tree.nodes[b.parent].child1 == i_a {
				tree.nodes[b.parent].child1 = i_b;
			} else {
				assert(tree.nodes[b.parent].child2 == i_a);
				tree.nodes[b.parent].child2 = i_b;
			}
		} else {
			tree.root = i_b;
		}

		// Rotate
		if d.height > e.height {
			b.child2 = i_d;
			a.child1 = i_e;
			e.parent = i_a;
			a.aabb = combine(c.aabb, e.aabb);
			b.aabb = combine(a.aabb, d.aabb);

			a.height = 1 + max(c.height, e.height);
			b.height = 1 + max(a.height, d.height);
		} else {
			b.child2 = i_e;
			a.child1 = i_d;
			d.parent = i_a;
			a.aabb = combine(c.aabb, d.aabb);
			b.aabb = combine(a.aabb, e.aabb);

			a.height = 1 + max(c.height, d.height);
			b.height = 1 + max(a.height, e.height);
		}

		return i_b;
	}

	return i_a;
}

compute_height :: (tree: BoundingTree) -> int {
	return compute_height(tree, tree.root);
}

compute_height :: (tree: BoundingTree, node_id: int) -> int {
	assert(0 <= node_id && node_id < tree.node_capacity);

	if is_leaf(tree.nodes[node_id]) {
		return 0;
	}

	height1 := compute_height(tree, tree.nodes[node_id].child1);
	height2 := compute_height(tree, tree.nodes[node_id].child2);
	return 1 + max(height1, height2);
}

validate_structure :: (tree: BoundingTree, index: int) {
	if index == NULL_NODE {
		return;
	}

	if index == tree.root {
		assert(tree.nodes[index].parent == NULL_NODE);
	}

	child1 := tree.nodes[index].child1;
	child2 := tree.nodes[index].child2;

	if is_leaf(tree.nodes[index]) {
		assert(child1 == NULL_NODE);
		assert(child2 == NULL_NODE);
		assert(tree.nodes[index].height == 0);
		return;
	}

	assert(0 <= child1 && child1 < tree.node_capacity);
	assert(0 <= child2 && child2 < tree.node_capacity);

	assert(tree.nodes[child1].parent == index);
	assert(tree.nodes[child2].parent == index);

	validate_structure(tree, child1);
	validate_structure(tree, child2);
}

validate_metrics :: (tree: BoundingTree, index: int) {
	if index == NULL_NODE {
		return;
	}

	child1 := tree.nodes[index].child1;
	child2 := tree.nodes[index].child2;

	if is_leaf(tree.nodes[index]) {
		assert(child1 == NULL_NODE);
		assert(child2 == NULL_NODE);
		assert(node.height == 0);
		return;
	}

	assert(0 <= child1 && child1 < tree.node_capacity);
	assert(0 <= child2 && child2 < tree.node_capacity);

	height1 := tree.nodes[child1].height;
	height2 := tree.nodes[child2].height;
	height := 1 + max(height1, height2);
	assert(tree.nodes[index].height == height);

	aabb := combine(tree.nodes[child1].aabb, tree.nodes[child2].aabb);

	assert(aabb.lower == tree.nodes[index].aabb.lower);
	assert(aabb.upper == tree.nodes[index].aabb.upper);

	validate_metrics(tree, child1);
	validate_metrics(tree, child2);
}

length_squared :: (a: Vector2) -> float {
    return a.x * a.x + a.y * a.y;
}

normalize :: (a: Vector2) -> Vector2 {
    magnitude := length(a);

    v: Vector2 = ---;
    v.x = a.x / magnitude;
    v.y = a.y / magnitude;
    return v;
}

cross :: (a: float, b: Vector2) -> Vector2 {
    return xy(-a * b.y, a * b.x);
}

abs :: (a: Vector2) -> Vector2 {
    v: Vector2 = ---;
    v.x = abs(a.x);
    v.y = abs(a.y);

    return v;
}

min :: (a: Vector2, b: Vector2) -> Vector2 {
    v: Vector2 = ---;
    v.x = min(a.x, b.x);
    v.y = min(a.y, b.y);

    return v;
}

max :: (a: Vector2, b: Vector2) -> Vector2 {
    v: Vector2 = ---;
    v.x = max(a.x, b.x);
    v.y = max(a.y, b.y);

    return v;
}

get_center :: (aabb: AABB) -> Vector2 {
    return 0.5 * (aabb.lower + aabb.upper);
}

get_extents :: (aabb: AABB) -> Vector2 {
    return 0.5 * (aabb.upper - aabb.lower);
}

get_perimeter :: (aabb: AABB) -> float {
    delta := aabb.upper - aabb.lower;
    return 2.0 * (delta.x + delta.y);
}

contains :: (outer: AABB, inner: AABB) -> bool {
    result := true;
    result = result && outer.lower.x <= inner.lower.x;
    result = result && outer.lower.y <= inner.lower.y;
    result = result && inner.upper.x <= outer.upper.x;
    result = result && inner.upper.y <= outer.upper.y;

    return result;
}

intersection :: (segment: Segment, aabb: AABB) -> bool {
    // p183
    e := aabb.upper - aabb.lower;
    d := segment.p2 - segment.p1;
    m := segment.p1 + segment.p2 - aabb.lower - aabb.upper;
    // Try world coordinate axes as separating axes
    adx := abs(d.x);
    if abs(m.x) > e.x + adx {
        return false;
    }
    ady := abs(d.y);
    if abs(m.y) > e.y + ady {
        return false;
    }
    // Add in an epsilon term to counteract arithmetic errors when segment is
    // (near) parallel to a coordinate axis (see text for detail)
    adx += EPSILON;
    ady += EPSILON;
    // Try cross products of segment direction vector with coordinate axes
    if abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx {
        return false;
    }
    // No separating axis found; segment must be overlapping AABB
    return true;
}

#import "Basic";
#import "Math";

TREE_DEBUG :: true;
NULL_NODE :: -1;
EPSILON :: 0.000001;
AABB_EXTENSION :: 0.1;
AABB_MULTIPLIER :: 4.0;