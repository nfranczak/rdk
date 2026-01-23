package spatialmath

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"
)

// bvhNode represents a node in a Bounding Volume Hierarchy tree.
// Each node has an axis-aligned bounding box (AABB) and either:
// - Two children (internal node), or
// - A list of items (leaf node)
// Items can be any spatial object: *Triangle, Geometry, etc.
// wikipedia.org/wiki/Bounding_volume_hierarchy
type bvhNode struct {
	min, max r3.Vector     // AABB bounds
	left     *bvhNode      // left child (nil for leaf)
	right    *bvhNode      // right child (nil for leaf)
	items    []interface{} // spatial items (only for leaf nodes)
}

// maxItemsPerLeaf is the threshold for splitting BVH nodes.
const maxItemsPerLeaf = 4

// getItemAABB returns the AABB for an item (Triangle or Geometry).
// For Triangles, returns the AABB in local space.
// For Geometries, returns the AABB in world space.
func getItemAABB(item interface{}) (r3.Vector, r3.Vector) {
	switch v := item.(type) {
	case *Triangle:
		return computeTriangleAABB(v)
	case Geometry:
		return computeGeometryAABB(v)
	default:
		// Return empty AABB for unknown types
		return r3.Vector{}, r3.Vector{}
	}
}

// getItemCentroid returns the centroid of an item.
func getItemCentroid(item interface{}) r3.Vector {
	switch v := item.(type) {
	case *Triangle:
		return v.Centroid()
	case Geometry:
		// For geometries, use the pose point as centroid
		return v.Pose().Point()
	default:
		return r3.Vector{}
	}
}

// computeTriangleAABB computes the AABB for a single triangle.
func computeTriangleAABB(tri *Triangle) (r3.Vector, r3.Vector) {
	points := tri.Points()
	minPt := points[0]
	maxPt := points[0]

	for _, pt := range points[1:] {
		minPt.X = math.Min(minPt.X, pt.X)
		minPt.Y = math.Min(minPt.Y, pt.Y)
		minPt.Z = math.Min(minPt.Z, pt.Z)
		maxPt.X = math.Max(maxPt.X, pt.X)
		maxPt.Y = math.Max(maxPt.Y, pt.Y)
		maxPt.Z = math.Max(maxPt.Z, pt.Z)
	}
	return minPt, maxPt
}

// computeGeometryAABB computes the AABB for any Geometry.
func computeGeometryAABB(g Geometry) (r3.Vector, r3.Vector) {
	switch geom := g.(type) {
	case *box:
		// Get the 8 corners of the box in world space
		corners := make([]r3.Vector, 8)
		halfSize := geom.halfSize
		localCorners := []r3.Vector{
			{X: -halfSize[0], Y: -halfSize[1], Z: -halfSize[2]},
			{X: -halfSize[0], Y: -halfSize[1], Z: halfSize[2]},
			{X: -halfSize[0], Y: halfSize[1], Z: -halfSize[2]},
			{X: -halfSize[0], Y: halfSize[1], Z: halfSize[2]},
			{X: halfSize[0], Y: -halfSize[1], Z: -halfSize[2]},
			{X: halfSize[0], Y: -halfSize[1], Z: halfSize[2]},
			{X: halfSize[0], Y: halfSize[1], Z: -halfSize[2]},
			{X: halfSize[0], Y: halfSize[1], Z: halfSize[2]},
		}
		for i, corner := range localCorners {
			corners[i] = Compose(geom.center, NewPoseFromPoint(corner)).Point()
		}
		return computePointsAABB(corners)
	case *sphere:
		center := geom.pose.Point()
		r := geom.radius
		return r3.Vector{X: center.X - r, Y: center.Y - r, Z: center.Z - r},
			r3.Vector{X: center.X + r, Y: center.Y + r, Z: center.Z + r}
	case *point:
		pt := geom.position
		return pt, pt
	case *capsule:
		// Get the two end points of the capsule line segment
		p1 := geom.segA
		p2 := geom.segB
		r := geom.radius
		minPt := r3.Vector{
			X: math.Min(p1.X, p2.X) - r,
			Y: math.Min(p1.Y, p2.Y) - r,
			Z: math.Min(p1.Z, p2.Z) - r,
		}
		maxPt := r3.Vector{
			X: math.Max(p1.X, p2.X) + r,
			Y: math.Max(p1.Y, p2.Y) + r,
			Z: math.Max(p1.Z, p2.Z) + r,
		}
		return minPt, maxPt
	case *Mesh:
		// For mesh, use the BVH root if available, otherwise compute from triangles
		if geom.bvh != nil {
			min, max := transformAABB(geom.bvh.min, geom.bvh.max, geom.pose)
			return min, max
		}
		// Compute from all triangles in world space
		points := make([]r3.Vector, 0, len(geom.triangles)*3)
		for _, tri := range geom.triangles {
			worldTri := tri.Transform(geom.pose)
			points = append(points, worldTri.Points()...)
		}
		return computePointsAABB(points)
	default:
		// Unknown geometry type - return zero AABB
		return r3.Vector{}, r3.Vector{}
	}
}

// computePointsAABB computes the AABB encompassing a set of points.
func computePointsAABB(points []r3.Vector) (r3.Vector, r3.Vector) {
	if len(points) == 0 {
		return r3.Vector{}, r3.Vector{}
	}
	minPt := points[0]
	maxPt := points[0]
	for _, pt := range points[1:] {
		minPt.X = math.Min(minPt.X, pt.X)
		minPt.Y = math.Min(minPt.Y, pt.Y)
		minPt.Z = math.Min(minPt.Z, pt.Z)
		maxPt.X = math.Max(maxPt.X, pt.X)
		maxPt.Y = math.Max(maxPt.Y, pt.Y)
		maxPt.Z = math.Max(maxPt.Z, pt.Z)
	}
	return minPt, maxPt
}

// buildBVH constructs a BVH from a list of spatial items (triangles, geometries, etc.).
func buildBVH(items []interface{}) *bvhNode {
	if len(items) == 0 {
		return nil
	}
	return buildBVHNode(items)
}

// buildBVHFromTriangles is a convenience function to build a BVH from triangles.
func buildBVHFromTriangles(triangles []*Triangle) *bvhNode {
	items := make([]interface{}, len(triangles))
	for i, tri := range triangles {
		items[i] = tri
	}
	return buildBVH(items)
}

func buildBVHNode(items []interface{}) *bvhNode {
	node := &bvhNode{}

	// Compute AABB (axis aligned bounding box) for all items
	node.min, node.max = computeItemsAABB(items)

	// If few enough items, make this a leaf node
	if len(items) <= maxItemsPerLeaf {
		node.items = items
		return node
	}

	// Find the longest axis to split on
	extent := node.max.Sub(node.min)
	axis := 0 // X
	if extent.Y > extent.X && extent.Y > extent.Z {
		axis = 1 // Y
	} else if extent.Z > extent.X && extent.Z > extent.Y {
		axis = 2 // Z
	}

	// Sort items by centroid along the chosen axis
	sort.Slice(items, func(i, j int) bool {
		ci := getItemCentroid(items[i])
		cj := getItemCentroid(items[j])
		switch axis {
		case 0:
			return ci.X < cj.X
		case 1:
			return ci.Y < cj.Y
		default:
			return ci.Z < cj.Z
		}
	})

	// Split at median
	mid := len(items) / 2
	node.left = buildBVHNode(items[:mid])
	node.right = buildBVHNode(items[mid:])

	return node
}

// computeItemsAABB computes the AABB encompassing all given items.
func computeItemsAABB(items []interface{}) (r3.Vector, r3.Vector) {
	minPt := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	maxPt := r3.Vector{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}

	for _, item := range items {
		itemMin, itemMax := getItemAABB(item)
		minPt.X = math.Min(minPt.X, itemMin.X)
		minPt.Y = math.Min(minPt.Y, itemMin.Y)
		minPt.Z = math.Min(minPt.Z, itemMin.Z)
		maxPt.X = math.Max(maxPt.X, itemMax.X)
		maxPt.Y = math.Max(maxPt.Y, itemMax.Y)
		maxPt.Z = math.Max(maxPt.Z, itemMax.Z)
	}
	return minPt, maxPt
}

// aabbOverlap checks if two AABBs overlap.
func aabbOverlap(min1, max1, min2, max2 r3.Vector) bool {
	return min1.X <= max2.X && max1.X >= min2.X &&
		min1.Y <= max2.Y && max1.Y >= min2.Y &&
		min1.Z <= max2.Z && max1.Z >= min2.Z
}

// aabbDistance computes the minimum distance between two non-overlapping AABBs.
func aabbDistance(min1, max1, min2, max2 r3.Vector) float64 {
	dx := math.Max(0, math.Max(min1.X-max2.X, min2.X-max1.X))
	dy := math.Max(0, math.Max(min1.Y-max2.Y, min2.Y-max1.Y))
	dz := math.Max(0, math.Max(min1.Z-max2.Z, min2.Z-max1.Z))
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// transformAABB transforms an AABB by a pose, returning a new (potentially larger) AABB.
func transformAABB(minPt, maxPt r3.Vector, pose Pose) (r3.Vector, r3.Vector) {
	// Get the 8 corners of the AABB
	corners := []r3.Vector{
		{X: minPt.X, Y: minPt.Y, Z: minPt.Z},
		{X: minPt.X, Y: minPt.Y, Z: maxPt.Z},
		{X: minPt.X, Y: maxPt.Y, Z: minPt.Z},
		{X: minPt.X, Y: maxPt.Y, Z: maxPt.Z},
		{X: maxPt.X, Y: minPt.Y, Z: minPt.Z},
		{X: maxPt.X, Y: minPt.Y, Z: maxPt.Z},
		{X: maxPt.X, Y: maxPt.Y, Z: minPt.Z},
		{X: maxPt.X, Y: maxPt.Y, Z: maxPt.Z},
	}

	newMin := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	newMax := r3.Vector{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}

	for _, corner := range corners {
		worldPt := Compose(pose, NewPoseFromPoint(corner)).Point()
		newMin.X = math.Min(newMin.X, worldPt.X)
		newMin.Y = math.Min(newMin.Y, worldPt.Y)
		newMin.Z = math.Min(newMin.Z, worldPt.Z)
		newMax.X = math.Max(newMax.X, worldPt.X)
		newMax.Y = math.Max(newMax.Y, worldPt.Y)
		newMax.Z = math.Max(newMax.Z, worldPt.Z)
	}
	return newMin, newMax
}

// bvhCollidesWithBVH checks if two BVH trees collide, using the given poses to transform them.
func bvhCollidesWithBVH(node1, node2 *bvhNode, pose1, pose2 Pose, collisionBufferMM float64) (bool, float64) {
	if node1 == nil || node2 == nil {
		return false, math.Inf(1)
	}

	// Transform AABBs to world space
	min1, max1 := transformAABB(node1.min, node1.max, pose1)
	min2, max2 := transformAABB(node2.min, node2.max, pose2)

	// Expand first AABB by collision buffer
	min1.X -= collisionBufferMM
	min1.Y -= collisionBufferMM
	min1.Z -= collisionBufferMM
	max1.X += collisionBufferMM
	max1.Y += collisionBufferMM
	max1.Z += collisionBufferMM

	// Check if AABBs overlap
	if !aabbOverlap(min1, max1, min2, max2) {
		return false, aabbDistance(min1, max1, min2, max2)
	}

	// Both are leaves - do item-item checks (expecting triangles for mesh-mesh collision)
	if node1.items != nil && node2.items != nil {
		return leafCollidesWithLeaf(node1.items, node2.items, pose1, pose2, collisionBufferMM)
	}

	// Recurse into children
	// Strategy: descend into the larger node first for better culling
	if node1.items != nil {
		// node1 is leaf, recurse into node2's children
		leftCollide, leftDist := bvhCollidesWithBVH(node1, node2.left, pose1, pose2, collisionBufferMM)
		if leftCollide {
			return true, -1
		}
		rightCollide, rightDist := bvhCollidesWithBVH(node1, node2.right, pose1, pose2, collisionBufferMM)
		if rightCollide {
			return true, -1
		}
		return false, math.Min(leftDist, rightDist)
	}

	if node2.items != nil {
		// node2 is leaf, recurse into node1's children
		leftCollide, leftDist := bvhCollidesWithBVH(node1.left, node2, pose1, pose2, collisionBufferMM)
		if leftCollide {
			return true, -1
		}
		rightCollide, rightDist := bvhCollidesWithBVH(node1.right, node2, pose1, pose2, collisionBufferMM)
		if rightCollide {
			return true, -1
		}
		return false, math.Min(leftDist, rightDist)
	}

	// Both are internal nodes - check all 4 combinations
	minDist := math.Inf(1)
	pairs := [][2]*bvhNode{
		{node1.left, node2.left},
		{node1.left, node2.right},
		{node1.right, node2.left},
		{node1.right, node2.right},
	}

	for _, pair := range pairs {
		collide, dist := bvhCollidesWithBVH(pair[0], pair[1], pose1, pose2, collisionBufferMM)
		if collide {
			return true, -1
		}
		if dist < minDist {
			minDist = dist
		}
	}

	return false, minDist
}

// leafCollidesWithLeaf performs item-item collision between two leaf nodes.
// Currently expects items to be *Triangle for mesh-mesh collision.
func leafCollidesWithLeaf(items1, items2 []interface{}, pose1, pose2 Pose, collisionBufferMM float64) (bool, float64) {
	minDist := math.Inf(1)

	for _, item1 := range items1 {
		t1, ok := item1.(*Triangle)
		if !ok {
			continue
		}
		worldTri1 := t1.Transform(pose1)
		p1 := worldTri1.Points()

		for _, item2 := range items2 {
			t2, ok := item2.(*Triangle)
			if !ok {
				continue
			}
			worldTri2 := t2.Transform(pose2)
			p2 := worldTri2.Points()

			// Check segments from tri1 against tri2
			for i := 0; i < 3; i++ {
				start := p1[i]
				end := p1[(i+1)%3]
				bestSegPt, bestTriPt := ClosestPointsSegmentTriangle(start, end, worldTri2)
				dist := bestSegPt.Sub(bestTriPt).Norm()
				if dist <= collisionBufferMM {
					return true, -1
				}
				if dist < minDist {
					minDist = dist
				}
			}

			// Check segments from tri2 against tri1
			for i := 0; i < 3; i++ {
				start := p2[i]
				end := p2[(i+1)%3]
				bestSegPt, bestTriPt := ClosestPointsSegmentTriangle(start, end, worldTri1)
				dist := bestSegPt.Sub(bestTriPt).Norm()
				if dist <= collisionBufferMM {
					return true, -1
				}
				if dist < minDist {
					minDist = dist
				}
			}
		}
	}

	return false, minDist
}

// bvhDistanceFromBVH computes the minimum distance between two BVH trees.
func bvhDistanceFromBVH(node1, node2 *bvhNode, pose1, pose2 Pose) float64 {
	if node1 == nil || node2 == nil {
		return math.Inf(1)
	}

	// Transform AABBs to world space
	min1, max1 := transformAABB(node1.min, node1.max, pose1)
	min2, max2 := transformAABB(node2.min, node2.max, pose2)

	// Check if AABBs overlap
	if !aabbOverlap(min1, max1, min2, max2) {
		// If AABBs don't overlap, the AABB distance is a lower bound
		// For distant meshes, this is good enough
		return aabbDistance(min1, max1, min2, max2)
	}

	// Both are leaves - compute exact distance
	if node1.items != nil && node2.items != nil {
		return leafDistanceFromLeaf(node1.items, node2.items, pose1, pose2)
	}

	// Recurse into children
	if node1.items != nil {
		leftDist := bvhDistanceFromBVH(node1, node2.left, pose1, pose2)
		rightDist := bvhDistanceFromBVH(node1, node2.right, pose1, pose2)
		return math.Min(leftDist, rightDist)
	}

	if node2.items != nil {
		leftDist := bvhDistanceFromBVH(node1.left, node2, pose1, pose2)
		rightDist := bvhDistanceFromBVH(node1.right, node2, pose1, pose2)
		return math.Min(leftDist, rightDist)
	}

	// Both are internal nodes
	minDist := math.Inf(1)
	pairs := [][2]*bvhNode{
		{node1.left, node2.left},
		{node1.left, node2.right},
		{node1.right, node2.left},
		{node1.right, node2.right},
	}

	for _, pair := range pairs {
		dist := bvhDistanceFromBVH(pair[0], pair[1], pose1, pose2)
		if dist < minDist {
			minDist = dist
		}
	}

	return minDist
}

// leafDistanceFromLeaf computes the minimum distance between two sets of items.
// Currently expects items to be *Triangle for mesh-mesh distance.
func leafDistanceFromLeaf(items1, items2 []interface{}, pose1, pose2 Pose) float64 {
	minDist := math.Inf(1)

	for _, item1 := range items1 {
		t1, ok := item1.(*Triangle)
		if !ok {
			continue
		}
		worldTri1 := t1.Transform(pose1)
		p1 := worldTri1.Points()

		for _, item2 := range items2 {
			t2, ok := item2.(*Triangle)
			if !ok {
				continue
			}
			worldTri2 := t2.Transform(pose2)
			p2 := worldTri2.Points()

			// Check segments from tri1 against tri2
			for i := 0; i < 3; i++ {
				start := p1[i]
				end := p1[(i+1)%3]
				bestSegPt, bestTriPt := ClosestPointsSegmentTriangle(start, end, worldTri2)
				dist := bestSegPt.Sub(bestTriPt).Norm()
				if dist < minDist {
					minDist = dist
				}
			}

			// Check segments from tri2 against tri1
			for i := 0; i < 3; i++ {
				start := p2[i]
				end := p2[(i+1)%3]
				bestSegPt, bestTriPt := ClosestPointsSegmentTriangle(start, end, worldTri1)
				dist := bestSegPt.Sub(bestTriPt).Norm()
				if dist < minDist {
					minDist = dist
				}
			}
		}
	}

	return minDist
}

// bvhCollidesWithGeometry checks if a BVH tree collides with any geometry type.
// Uses AABB (box) collision checks to traverse the tree efficiently.
func bvhCollidesWithGeometry(node *bvhNode, meshPose Pose, g Geometry, collisionBufferMM float64) (bool, float64) {
	if node == nil {
		return false, math.Inf(1)
	}

	// Transform AABB to world space
	minWorld, maxWorld := transformAABB(node.min, node.max, meshPose)

	// Create a box from the AABB
	center := minWorld.Add(maxWorld).Mul(0.5)
	halfSize := maxWorld.Sub(minWorld).Mul(0.5)
	aabbBox := &box{
		center:   NewPoseFromPoint(center),
		halfSize: [3]float64{halfSize.X, halfSize.Y, halfSize.Z},
	}

	// Check if AABB collides with geometry
	collides, dist, err := g.CollidesWith(aabbBox, collisionBufferMM)
	if err != nil {
		// If collision type unsupported, fall back to checking all items
		return leafCollidesWithGeometry(node.getAllItems(), meshPose, g, collisionBufferMM)
	}
	if !collides {
		// AABB doesn't collide, so nothing inside it can collide
		return false, dist
	}

	// AABB collides - need to check further
	if node.items != nil {
		// Leaf node - check actual items
		return leafCollidesWithGeometry(node.items, meshPose, g, collisionBufferMM)
	}

	// Internal node - recurse into children
	leftCollide, leftDist := bvhCollidesWithGeometry(node.left, meshPose, g, collisionBufferMM)
	if leftCollide {
		return true, -1
	}
	rightCollide, rightDist := bvhCollidesWithGeometry(node.right, meshPose, g, collisionBufferMM)
	if rightCollide {
		return true, -1
	}
	return false, math.Min(leftDist, rightDist)
}

// leafCollidesWithGeometry checks if items collide with a geometry.
// Currently expects items to be *Triangle for mesh collision.
func leafCollidesWithGeometry(items []interface{}, meshPose Pose, g Geometry, collisionBufferMM float64) (bool, float64) {
	minDist := math.Inf(1)

	for _, item := range items {
		tri, ok := item.(*Triangle)
		if !ok {
			continue
		}
		worldTri := tri.Transform(meshPose)

		// Check triangle vs geometry by converting triangle to a temporary mesh
		triMesh := &Mesh{
			pose:      NewZeroPose(),
			triangles: []*Triangle{worldTri},
		}

		collides, dist, err := g.CollidesWith(triMesh, collisionBufferMM)
		if err != nil {
			// Fall back to individual point checks if collision type unsupported
			for _, pt := range worldTri.Points() {
				ptGeom := &point{position: pt}
				c, d, _ := g.CollidesWith(ptGeom, collisionBufferMM)
				if c {
					return true, -1
				}
				if d < minDist {
					minDist = d
				}
			}
			continue
		}

		if collides {
			return true, -1
		}
		if dist < minDist {
			minDist = dist
		}
	}

	return false, minDist
}

// bvhDistanceFromGeometry computes the minimum distance between a BVH tree and any geometry type.
func bvhDistanceFromGeometry(node *bvhNode, meshPose Pose, g Geometry) float64 {
	if node == nil {
		return math.Inf(1)
	}

	// Transform AABB to world space
	minWorld, maxWorld := transformAABB(node.min, node.max, meshPose)

	// Create a box from the AABB
	center := minWorld.Add(maxWorld).Mul(0.5)
	halfSize := maxWorld.Sub(minWorld).Mul(0.5)

	aabbBox := &box{
		center:   NewPoseFromPoint(center),
		halfSize: [3]float64{halfSize.X, halfSize.Y, halfSize.Z},
	}

	// Check distance from AABB to geometry
	_, err := g.DistanceFrom(aabbBox)
	if err != nil {
		// If distance type unsupported, fall back to checking all items
		return leafDistanceFromGeometry(node.getAllItems(), meshPose, g)
	}

	// Check further
	if node.items != nil {
		// Leaf node - check actual items
		return leafDistanceFromGeometry(node.items, meshPose, g)
	}

	// Internal node - recurse into children
	leftDist := bvhDistanceFromGeometry(node.left, meshPose, g)
	rightDist := bvhDistanceFromGeometry(node.right, meshPose, g)
	return math.Min(leftDist, rightDist)
}

// leafDistanceFromGeometry computes the minimum distance between items and a geometry.
// Currently expects items to be *Triangle for mesh distance.
func leafDistanceFromGeometry(items []interface{}, meshPose Pose, g Geometry) float64 {
	minDist := math.Inf(1)

	for _, item := range items {
		tri, ok := item.(*Triangle)
		if !ok {
			continue
		}
		worldTri := tri.Transform(meshPose)

		// Check triangle vs geometry by converting triangle to a temporary mesh
		triMesh := &Mesh{
			pose:      NewZeroPose(),
			triangles: []*Triangle{worldTri},
		}

		dist, err := g.DistanceFrom(triMesh)
		if err != nil {
			// Fall back to individual point distances if type unsupported
			for _, pt := range worldTri.Points() {
				ptGeom := &point{position: pt}
				d, _ := g.DistanceFrom(ptGeom)
				if d < minDist {
					minDist = d
				}
			}
			continue
		}

		if dist < minDist {
			minDist = dist
		}
	}

	return minDist
}

// getAllItems recursively collects all items from a BVH node and its children.
func (n *bvhNode) getAllItems() []interface{} {
	if n == nil {
		return nil
	}
	if n.items != nil {
		return n.items
	}
	result := make([]interface{}, 0)
	result = append(result, n.left.getAllItems()...)
	result = append(result, n.right.getAllItems()...)
	return result
}
