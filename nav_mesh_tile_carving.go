package unityai

import (
	"math"
	"reflect"
	"sort"
	"unsafe"
)

type CarveResultStatus int32

const (
	kReplaceTile CarveResultStatus = iota
	kRestoreTile
	kRemoveTile
)

type ClippedDetailMesh struct {
	polyIndex int
	vertices  []Vector3f
	triangles []uint16
}

func NewClippedDetailMesh() *ClippedDetailMesh {
	return &ClippedDetailMesh{
		polyIndex: 0,
		vertices:  nil,
		triangles: nil,
	}
}

type DetailMeshBVNode struct {
	min Vector3f
	max Vector3f
	idx int32
}

type DetailMeshPoly struct {
	vertBase  int32
	vertCount int32 // 三角形的点数量;包含poly的点
	triBase   int32
	triCount  int32 // 三角形的点数量;排除Poly的点
	bvBase    int32
	bvCount   int32
}

type Triangles []uint16

func (t *Triangles) resize_uninitialized(size int) {
	if cap(*t) >= size {
		*t = (*t)[:size]
	} else {
		*t = append(*t, make([]uint16, size-len(*t))...)
	}
}

type PolyContainer []DetailMeshPoly

func (c *PolyContainer) resize_uninitialized(size int) {
	if cap(*c) >= size {
		*c = (*c)[:size]
	} else {
		*c = append(*c, make([]DetailMeshPoly, size-len(*c))...)
	}
}

type DetailMesh struct {
	vertices  []Vector3f
	triangles Triangles
	polys     PolyContainer
	bvNodes   DetailMeshBVNodeContainer
}

type DetailMeshBVNodeContainer []DetailMeshBVNode

func (this *DetailMeshBVNodeContainer) resize_uninitialized(size int32) {
	if cap(*this) >= int(size) {
		*this = (*this)[:size]
	} else {
		*this = append(*this, make([]DetailMeshBVNode, int(size)-len(*this))...)
	}
}

func NewDetailMesh() *DetailMesh {
	return &DetailMesh{}
}

func CarveNavMeshTile(tileData *[]byte, tileDataSize *uint32,
	sourceData []byte, sourceDataSize int32,
	shapes []NavMeshCarveShape, shapeCount int,
	carveDepth float32, carveWidth float32, quantSize float32,
	position Vector3f, rotation Quaternionf) CarveResultStatus {
	Assert(sourceData != nil)
	Assert(sourceDataSize > 0)
	*tileData = nil
	*tileDataSize = 0
	if shapeCount == 0 {
		return kRestoreTile
	}

	tile := NewNavMeshTile()
	if !PatchMeshTilePointers(tile, sourceData, sourceDataSize) {
		// remove tile altogether if we cannot patch source data pointers
		return kRemoveTile
	}

	Assert(tile.header != nil)
	tileOffset := tile.header.bmin.Add(tile.header.bmax).Mulf(0.5)

	detailMesh := NewDetailMesh()
	UnpackDetailMesh(detailMesh, tile, tileOffset)
	var mat Matrix4x4f
	mat.SetTRInverse(position, rotation)
	hull := Hull{}
	var carveHulls DetailHullContainer
	for i := 0; i < shapeCount; i++ {
		var localShape NavMeshCarveShape
		localShape.shape = shapes[i].shape
		localShape.center = mat.MultiplyPoint3(shapes[i].center)
		localShape.extents = shapes[i].extents
		localShape.xAxis = mat.MultiplyVector3(shapes[i].xAxis)
		localShape.yAxis = mat.MultiplyVector3(shapes[i].yAxis)
		localShape.zAxis = mat.MultiplyVector3(shapes[i].zAxis)
		TransformAABBSlow(shapes[i].bounds, mat, &localShape.bounds)
		validHull := false
		var localBounds MinMaxAABB
		if localShape.shape == kObstacleShapeCapsule {
			validHull = CalculateCapsuleHull(&hull, &localBounds, &localShape, tileOffset, carveDepth, carveWidth)
		} else if localShape.shape == kObstacleShapeBox {
			validHull = CalculateBoxHull(&hull, &localBounds, &localShape, tileOffset, carveDepth, carveWidth)
		}

		if !validHull {
			continue
		}

		localBounds.m_Min = localBounds.m_Min.Sub(NewVector3f(quantSize, quantSize, quantSize))
		localBounds.m_Max = localBounds.m_Max.Add(NewVector3f(quantSize, quantSize, quantSize))

		// Find potentially intersecting polygons and create new cutter
		// based on the intersection points in detail mesh.
		var detailHulls DetailHullContainer
		validHull = BuildDetailHulls(&detailHulls, hull, localBounds, detailMesh, tile, tileOffset, quantSize)
		if validHull {
			carveHulls = append(carveHulls, detailHulls...)
		}
	}

	// The vertex quantization factor needs to match the tile size
	// in order to not get any gaps at tile boundaries.
	// As long as the divider is large enough and divisible by 2
	// (because tileOffset is at tile center during carving),
	// things should work fine.
	quantFactor := quantSize
	dynamicMesh := NewDynamicMesh(quantFactor)
	TileToDynamicMesh(tile, dynamicMesh, tileOffset)

	// Restore if nothing was clipped
	if !dynamicMesh.ClipPolys2(carveHulls) {
		return kRestoreTile
	}

	// Remove if nothing is left
	if dynamicMesh.PolyCount() == 0 {
		return kReplaceTile
	}

	// 修正新增加的poly的高度
	// Project new vertices to detail meshes.
	ProjectNewVerticesToDetailMesh(dynamicMesh, detailMesh)
	dynamicMesh.FindNeighbors()

	// Clip the detail triangles of the original polygons to match each new polygon.
	var clipped []*ClippedDetailMesh
	clipped = make([]*ClippedDetailMesh, dynamicMesh.PolyCount())
	// 上面是对Poly进行裁剪，这里是对三角形进行裁剪
	ClipDetailMeshes(clipped, dynamicMesh, detailMesh, tile, tileOffset, quantFactor)
	*tileData = DynamicMeshToTile(tileDataSize, dynamicMesh, clipped, tile, tileOffset)
	return kReplaceTile
}

func PatchMeshTilePointers(tile *NavMeshTile, data []byte, dataSize int32) bool {
	header := (*NavMeshDataHeader)(unsafe.Pointer(&(data[0])))
	tile.header = nil
	if header.magic != kNavMeshMagic {
		return false
	}
	if header.version != kNavMeshVersion {
		return false
	}

	tile.header = header

	// Patch header pointers.
	headerSize := Align4(unsafe.Sizeof(NavMeshDataHeader{}))
	vertsSize := Align4(unsafe.Sizeof(Vector3f{}) * uintptr(header.vertCount))
	polysSize := Align4(unsafe.Sizeof(NavMeshPoly{}) * uintptr(header.polyCount))
	detailMeshesSize := Align4(unsafe.Sizeof(NavMeshPolyDetail{}) * uintptr(header.detailMeshCount))
	detailVertsSize := Align4(unsafe.Sizeof(Vector3f{}) * uintptr(header.detailVertCount))
	detailTrisSize := Align4(unsafe.Sizeof(NavMeshPolyDetailIndex(0)) * 4 * uintptr(header.detailTriCount))
	bvtreeSize := Align4(unsafe.Sizeof(NavMeshBVNode{}) * uintptr(header.bvNodeCount))

	d := headerSize
	var verts []Vector3f
	sliceHeader := (*reflect.SliceHeader)(unsafe.Pointer(&(verts)))
	sliceHeader.Cap = int(header.vertCount)
	sliceHeader.Len = int(header.vertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += vertsSize

	var polys []NavMeshPoly
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(polys)))
	sliceHeader.Cap = int(header.polyCount)
	sliceHeader.Len = int(header.polyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += polysSize

	var detailMeshes []NavMeshPolyDetail
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailMeshes)))
	sliceHeader.Cap = int(header.detailMeshCount)
	sliceHeader.Len = int(header.detailMeshCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailMeshesSize

	var detailVerts []Vector3f
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailVerts)))
	sliceHeader.Cap = int(header.detailVertCount)
	sliceHeader.Len = int(header.detailVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailVertsSize

	var detailTris []NavMeshPolyDetailIndex
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detailTris)))
	sliceHeader.Cap = int(header.detailTriCount * 4)
	sliceHeader.Len = int(header.detailTriCount * 4)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += detailTrisSize

	var bvTree []NavMeshBVNode
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(bvTree)))
	sliceHeader.Cap = int(header.bvNodeCount)
	sliceHeader.Len = int(header.bvNodeCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(data[d])))
	d += bvtreeSize

	tile.verts = verts
	tile.polys = polys
	tile.detailMeshes = detailMeshes
	tile.detailVerts = detailVerts
	tile.detailTris = detailTris
	tile.bvTree = bvTree

	// If there are no items in the bvtree, reset the tree pointer.
	if bvtreeSize == 0 {
		tile.bvTree = nil
	}

	return true
}

/*
*
给你一个「多边形」，再给你一组「平面」定义的空间区域（hull），
这个函数会让多边形被这些平面一层层“切”掉 hull 外面的部分，最后只留下 hull 内的那部分。
inside 是待切割的多边形
temp 是本次切割的多边形保存容器
*/
func HullPolygonIntersection(inside *Polygon, hull *Hull, temp *Polygon, quantFactor float32) {
	planeCount := len(*hull)

	for ic := 0; ic < planeCount; ic++ {
		plane := (*hull)[ic]
		result := SplitPoly(temp, *inside, plane, quantFactor, nil, 0)
		if result == 0 {
			inside.resize_uninitialized(len(*temp))
			copy(*inside, *temp) // 将切割结果保存在inside里面
		} else if result == 1 {
			inside.resize_uninitialized(0)
			return
		}
	}
}

type DetailNodeXSorter []DetailMeshBVNode

func (d DetailNodeXSorter) Len() int {
	return len(d)
}

func (d DetailNodeXSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.x + ra.max.x) * 0.5
	b := (rb.min.x + rb.max.x) * 0.5
	return a < b
}

func (d DetailNodeXSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

type DetailNodeYSorter []DetailMeshBVNode

func (d DetailNodeYSorter) Len() int {
	return len(d)
}

func (d DetailNodeYSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.y + ra.max.y) * 0.5
	b := (rb.min.y + rb.max.y) * 0.5
	return a < b
}

func (d DetailNodeYSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

type DetailNodeZSorter []DetailMeshBVNode

func (d DetailNodeZSorter) Len() int {
	return len(d)
}

func (d DetailNodeZSorter) Less(i, j int) bool {
	ra := d[i]
	rb := d[j]
	a := (ra.min.z + ra.max.z) * 0.5
	b := (rb.min.z + rb.max.z) * 0.5
	return a < b
}

func (d DetailNodeZSorter) Swap(i, j int) {
	d[i], d[j] = d[j], d[i]
}

func OverlapBoundsXZ(amin Vector3f, amax Vector3f,
	bmin Vector3f, bmax Vector3f) bool {
	if amin.x > bmax.x || amax.x < bmin.x {
		return false
	}
	if amin.z > bmax.z || amax.z < bmin.z {
		return false
	}
	return true
}

func DetailLongestAxis(v Vector3f) int32 {
	axis := int32(0)
	max := v.x
	if v.y > max {
		axis = 1
		max = v.y
	}
	if v.z > max {
		axis = 2
		max = v.z
	}
	return axis
}

func Subdivide(nodes *[]DetailMeshBVNode,
	items []DetailMeshBVNode,
	imin int32, imax int32) {
	inum := imax - imin
	*nodes = append(*nodes, DetailMeshBVNode{})
	icur := len(*nodes) - 1
	node := &(*nodes)[icur]

	// Update bounds
	node.min = items[imin].min
	node.max = items[imin].max
	for i := imin + 1; i < imax; i++ {
		node.min = MinVector3f(node.min, items[i].min)
		node.max = MaxVector3f(node.max, items[i].max)
	}

	if (imax - imin) <= 1 {
		// Leaf, copy triangles.
		node.idx = items[imin].idx
	} else {
		// Split remaining items along longest axis
		axis := DetailLongestAxis(node.max.Sub(node.min))
		if axis == 0 {
			sort.Sort(DetailNodeXSorter(items[imin:imax]))
		} else if axis == 1 {
			sort.Sort(DetailNodeYSorter(items[imin:imax]))
		} else {
			sort.Sort(DetailNodeZSorter(items[imin:imax]))
		}
		isplit := imin + inum/2

		// Left
		Subdivide(nodes, items, imin, isplit)
		// Right
		Subdivide(nodes, items, isplit, imax)
		iescape := (len(*nodes) - 1) - icur
		// Negative index means escape.
		(*nodes)[icur].idx = int32(-iescape) // 'node' ref may be invalid because of realloc.
	}
}

func BuildBVTree(nodes *[]DetailMeshBVNode,
	vertices []Vector3f,
	tris []uint16, triCount int32) bool {
	*nodes = (*nodes)[:0]

	// Build input items
	items := make([]DetailMeshBVNode, triCount)
	for i := int32(0); i < triCount; i++ {
		t := tris[i*4:]
		it := &items[i]
		it.idx = int32(i)
		// Calc triangle bounds.
		it.min = vertices[t[0]]
		it.max = vertices[t[0]]
		it.min = MinVector3f(it.min, vertices[t[1]])
		it.max = MaxVector3f(it.max, vertices[t[1]])
		it.min = MinVector3f(it.min, vertices[t[2]])
		it.max = MaxVector3f(it.max, vertices[t[2]])
	}

	Subdivide(nodes, items, 0, triCount)
	return true
}

type QueryDetailBVTreeCallback interface {
	process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32)
}

const BATCH_SIZE = 32

func QueryDetailBVTree(detailMesh *DetailMesh, poly *DetailMeshPoly,
	queryMin Vector3f, queryMax Vector3f,
	callback QueryDetailBVTreeCallback) {
	var batch [BATCH_SIZE]int32
	batchCount := int32(0)

	// Clip all detail triangles against the polygon.
	if poly.bvCount > 0 {
		nodes := detailMesh.bvNodes[poly.bvBase:]
		n := int32(0)
		for n < poly.bvCount {
			node := &nodes[n]
			overlap := OverlapBoundsXZ(queryMin, queryMax, node.min, node.max)
			isLeafNode := node.idx >= int32(0)
			if isLeafNode && overlap {
				if batchCount+1 > BATCH_SIZE {
					callback.process(detailMesh, poly, batch[:], batchCount)
					batchCount = 0
				}
				batch[batchCount] = poly.triBase + node.idx
				batchCount++
			}

			if overlap || isLeafNode {
				n++
			} else {
				escapeIndex := -node.idx
				n += escapeIndex
			}
		}
	} else {
		for j := int32(0); j < poly.triCount; j++ {
			if batchCount+1 > BATCH_SIZE {
				callback.process(detailMesh, poly, batch[:], batchCount)
				batchCount = 0
			}
			batch[batchCount] = poly.triBase + j
			batchCount++
		}
	}

	if batchCount > 0 {
		callback.process(detailMesh, poly, batch[:], batchCount)
		batchCount = 0
	}
}

func UnpackDetailMesh(detailMesh *DetailMesh, tile *NavMeshTile, tileOffset Vector3f) {
	// Unpack
	polyCount := tile.header.polyCount
	detailTriCount := tile.header.detailTriCount
	detailPolyCount := tile.header.detailMeshCount
	Assert(polyCount == detailPolyCount)
	detailMesh.triangles.resize_uninitialized(int(detailTriCount) * 4)
	detailMesh.polys.resize_uninitialized(int(detailPolyCount))
	bvTriCount := int32(0)
	maxTriCount := int32(0)
	kBVTreeThreshold := int32(6)
	for i := int32(0); i < polyCount; i++ {
		p := &tile.polys[i]
		pd := &tile.detailMeshes[i]
		poly := &detailMesh.polys[i]
		poly.bvBase = 0
		poly.bvCount = 0
		poly.vertBase = int32(len(detailMesh.vertices))
		poly.vertCount = int32(p.vertCount) + int32(pd.vertCount)
		for j := uint8(0); j < p.vertCount; j++ {
			detailMesh.vertices = append(detailMesh.vertices, tile.verts[p.verts[j]].Sub(tileOffset))
		}
		for j := uint32(0); j < uint32(pd.vertCount); j++ {
			detailMesh.vertices = append(detailMesh.vertices, tile.detailVerts[pd.vertBase+j].Sub(tileOffset))
		}

		poly.triBase = int32(pd.triBase)
		poly.triCount = int32(pd.triCount)
		for j := uint32(0); j < uint32(pd.triCount); j++ {
			t := tile.detailTris[(pd.triBase+j)*4:]
			detailMesh.triangles[(pd.triBase+j)*4+0] = uint16(t[0])
			detailMesh.triangles[(pd.triBase+j)*4+1] = uint16(t[1])
			detailMesh.triangles[(pd.triBase+j)*4+2] = uint16(t[2])
			detailMesh.triangles[(pd.triBase+j)*4+3] = uint16(t[3])
		}

		// 只要有一个poly的triCount>kBVTreeThreshold, 就会建立 BVH树
		if poly.triCount > kBVTreeThreshold {
			bvTriCount += poly.triCount
			if maxTriCount < poly.triCount {
				maxTriCount = poly.triCount
			}
		}
	}

	if bvTriCount > 0 {
		// Build BV-tree for polys which have many detail triangles.
		var nodes []DetailMeshBVNode
		for i := int32(0); i < polyCount; i++ {
			poly := &detailMesh.polys[i]
			if poly.triCount > kBVTreeThreshold {
				BuildBVTree(&nodes, detailMesh.vertices[poly.vertBase:], detailMesh.triangles[poly.triBase*4:], poly.triCount)
				nodeCount := int32(len(nodes))
				if nodeCount > 0 {
					poly.bvBase = int32(len(detailMesh.bvNodes))
					poly.bvCount = nodeCount
					detailMesh.bvNodes.resize_uninitialized(poly.bvBase + nodeCount)
					for j := int32(0); j < nodeCount; j++ {
						detailMesh.bvNodes[poly.bvBase+j] = nodes[j]
					}
				}
			}
		}
	}
}

func ClosestHeightToTriangleEdge(height *float32, dmin *float32,
	samplePos, va, vb, vc Vector3f) {
	var d, t float32
	*dmin = math.MaxFloat32
	d = SqrDistancePointSegment2D(&t, samplePos, va, vb)
	if d < *dmin {
		*height = va.y + (vb.y-va.y)*t
		*dmin = d
	}
	d = SqrDistancePointSegment2D(&t, samplePos, vb, vc)
	if d < *dmin {
		*height = vb.y + (vc.y-vb.y)*t
		*dmin = d
	}
	d = SqrDistancePointSegment2D(&t, samplePos, vc, va)
	if d < *dmin {
		*height = vc.y + (va.y-vc.y)*t
		*dmin = d
	}
}

func PickDetailTriHeight(height *float32, dmin *float32,
	samplePos, va, vb, vc Vector3f) {
	var h float32
	if ClosestHeightPointTriangle(&h, samplePos, va, vb, vc) {
		*height = h
		*dmin = 0.0
	}
	if *dmin > 0.0 {
		var dist float32
		ClosestHeightToTriangleEdge(&h, &dist, samplePos, va, vb, vc)
		if dist < *dmin {
			*height = h
			*dmin = dist
		}
	}
}

type PickHeightCallback struct {
	samplePos    Vector3f
	height, dmin float32
}

func NewPickHeightCallback(pos Vector3f) *PickHeightCallback {
	return &PickHeightCallback{
		samplePos: pos,
		height:    pos.y,
		dmin:      math.MaxFloat32,
	}
}

func (this *PickHeightCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		va := detailMesh.vertices[poly.vertBase+int32(t[0])]
		vb := detailMesh.vertices[poly.vertBase+int32(t[1])]
		vc := detailMesh.vertices[poly.vertBase+int32(t[2])]
		PickDetailTriHeight(&this.height, &this.dmin, this.samplePos, va, vb, vc)
	}
}

func PickDetailPolyHeight(detailMesh *DetailMesh, polyIdx int32, samplePos Vector3f) float32 {
	poly := &detailMesh.polys[polyIdx]
	sampleExt := NewVector3f(0.1, 0, 0.1)
	queryMin := samplePos.Sub(sampleExt)
	queryMax := samplePos.Add(sampleExt)
	callback := NewPickHeightCallback(samplePos)
	QueryDetailBVTree(detailMesh, poly, queryMin, queryMax, callback)
	return callback.height
}

/*
*
根据 detail 查询凸多边形每个点的高度
*/
func ProjectNewVerticesToDetailMesh(mesh *DynamicMesh, detailMesh *DetailMesh) {
	vertCount := mesh.VertCount()
	polyCount := mesh.PolyCount()
	vertexSourcePoly := make([]int32, vertCount)
	for i := range vertexSourcePoly {
		vertexSourcePoly[i] = -1
	}

	// Check which vertices have changed and store their original polygon too.
	// TODO: check if we need to store all source polys, now just projecting to the last one.
	for i := 0; i < polyCount; i++ {
		p := mesh.GetPoly(i)
		if p.m_Status != kOriginalPolygon {
			sourcePolyIndex := *mesh.GetData(i)
			for j := uint8(0); j < p.m_VertexCount; j++ {
				// index:vertexId   value: detailMesh.polyIndex
				vertexSourcePoly[p.m_VertexIDs[j]] = int32(sourcePolyIndex)
			}
		}
	}

	for i := 0; i < vertCount; i++ {
		ip := vertexSourcePoly[i]
		if ip == -1 {
			continue
		}
		pos := mesh.GetVertex(i)
		pos.y = PickDetailPolyHeight(detailMesh, ip, pos)
		mesh.SetVertex(i, pos)
	}
}

func CalcPolyDetailBounds(bounds *MinMaxAABB, detailMesh *DetailMesh, ip int32) {
	poly := detailMesh.polys[ip]
	bounds.m_Min = detailMesh.vertices[poly.vertBase]
	bounds.m_Max = detailMesh.vertices[poly.vertBase]
	for i := int32(1); i < poly.vertCount; i++ {
		bounds.EncapsulateV(detailMesh.vertices[poly.vertBase+i])
	}
}

func HasBoundaryVertices(verts Vertex2Array, bmin Vector2f, bmax Vector2f) bool {
	if len(verts) == 0 {
		return false
	}

	var vmin, vmax Vector2f
	vmin.x = verts[0].x
	vmax.x = verts[0].x
	vmin.y = verts[0].y
	vmax.y = verts[0].y
	for i := 1; i < len(verts); i++ {
		vmin = MinVector2f(vmin, verts[i])
		vmax = MaxVector2f(vmax, verts[i])
	}

	dmin := vmin.Sub(bmin)
	if Sqr(dmin.x) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	if Sqr(dmin.y) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}

	dmax := vmax.Sub(bmax)
	if Sqr(dmax.x) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	if Sqr(dmax.y) < Sqr(MAGIC_EDGE_DISTANCE) {
		return true
	}
	return false
}

type ClipCallback struct {
	m_Hull        *Hull         // 要裁剪/与之相交的凸包（或多边形）
	m_Inside      *Polygon      // 存放被裁剪后（相交后）的多边形顶点（结果输出）
	m_Temp        *Polygon      // 临时多边形缓冲区（用于计算/中间步骤）
	m_Footprint   *Vertex2Array // 2D 顶点数组（有时用于投影或网格化） 切割好的2d顶点放在里面
	m_QuantFactor float32       // 量化因子（用于坐标量化/整数化以减少浮点误差）
	m_Hit         bool          // 是否发生过相交/裁剪（结果标记）
}

func NewClipCallback(hull *Hull, inside *Polygon, temp *Polygon, footPrint *Vertex2Array, quantFactor float32) *ClipCallback {
	return &ClipCallback{
		m_Hull:        hull,
		m_Inside:      inside,
		m_Temp:        temp,
		m_Footprint:   footPrint,
		m_QuantFactor: quantFactor,
		m_Hit:         false,
	}
}

func (this *ClipCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		this.m_Inside.resize_uninitialized(3)
		(*this.m_Inside)[0] = detailMesh.vertices[poly.vertBase+int32(t[0])]
		(*this.m_Inside)[1] = detailMesh.vertices[poly.vertBase+int32(t[1])]
		(*this.m_Inside)[2] = detailMesh.vertices[poly.vertBase+int32(t[2])]
		HullPolygonIntersection(this.m_Inside, this.m_Hull, this.m_Temp, this.m_QuantFactor)
		if len(*this.m_Inside) == 0 {
			continue
		}

		// 将x,z记录下来;前面切割是切割三维x,y,z都切割的凸多边形;然后将切割好的凸多边形记录下来;
		for i := 0; i < len(*this.m_Inside); i++ {
			*this.m_Footprint = append(*this.m_Footprint, Vector2f{})
			v := &(*this.m_Footprint)[len(*this.m_Footprint)-1]
			v.x = (*this.m_Inside)[i].x
			v.y = (*this.m_Inside)[i].z
		}
		this.m_Hit = true
	}
}

func BuildDetailHulls(detailHulls *DetailHullContainer,
	hull Hull, bounds MinMaxAABB,
	detailMesh *DetailMesh, tile *NavMeshTile, tileOffset Vector3f, quantSize float32) bool {
	polyCount := tile.header.polyCount
	var inside Polygon
	var temp Polygon
	var footPrint Vertex2Array = make([]Vector2f, 0, 32)

	// Find polygons that potentially intersect with the cave hull.
	// We'll use detail mesh for this to capture all cases.
	// As we go we keep track of the polygons that were touched
	// as well as the vertices of the detail mesh intersection.
	// These intersection points will later be used to create a new infinite
	// carver which is actually used for carving.

	kTouched := byte(1)
	kVisited := byte(2)
	visited := make([]byte, polyCount)

	// TODO: we should be able to use BV-tree for this.
	nTouched := 0
	for ip := int32(0); ip < polyCount; ip++ {
		var polyBounds MinMaxAABB
		CalcPolyDetailBounds(&polyBounds, detailMesh, ip)
		if !IntersectAABBAABB(bounds, polyBounds) {
			continue
		}
		visited[ip] = kTouched
		nTouched++
	}

	var stack []int32

	/**
	这里是为了防止 用原始的Hull进行裁切的话 有Y轴 比如我是斜着的 poly有两层，我切了两层poly不同的地方，但是我用两层poly收集出来的x,z平面的话，就会多切数据 会出问题;
	将联通的poly变成一个hull;不联通的poly用两个hull;
	*/
	// Merge connecting regions.
	for ip := int32(0); ip < polyCount; ip++ {
		if visited[ip] != kTouched {
			continue
		}

		var detailHull DetailHull
		stack = stack[:0]
		stack = append(stack, ip)

		for len(stack) != 0 {
			curLen := len(stack)
			cur := stack[curLen-1]
			stack = stack[:curLen-1]
			detailHull.polysIds = append(detailHull.polysIds, int(cur))
			poly := &tile.polys[cur]
			for j := uint8(0); j < poly.vertCount; j++ {
				// Skip if no neighbour or if at tile border.
				if poly.neis[j] == uint16(0) || poly.neis[j]&uint16(0x8000) != 0 {
					continue
				}
				nei := poly.neis[j] - 1
				if visited[nei] == kTouched {
					visited[nei] = kVisited
					stack = append(stack, int32(nei))
				}
			}
		}
		*detailHulls = append(*detailHulls, detailHull)
	}

	if len(*detailHulls) == 0 {
		return false
	}

	var convexHull Vertex2Array

	/**
	这里因为前面将不联通的poly分类了放到不同的detailHull里面去，所以这里可以将其转换成2d的hull形式;
	*/
	detailHullCount := len(*detailHulls)
	for hi := 0; hi < detailHullCount; hi++ {
		detailHull := &(*detailHulls)[hi]
		footPrint = footPrint[:0]

		for i := 0; i < len(detailHull.polysIds); i++ {
			ip := detailHull.polysIds[i]
			dpoly := &detailMesh.polys[ip]
			callback := NewClipCallback(&hull, &inside, &temp, &footPrint, quantSize)
			QueryDetailBVTree(detailMesh, dpoly, bounds.m_Min, bounds.m_Max, callback)
			if !callback.m_Hit {
				polyIdLen := len(detailHull.polysIds)
				detailHull.polysIds[i] = detailHull.polysIds[polyIdLen-1]
				detailHull.polysIds = detailHull.polysIds[:polyIdLen-1]
				i--
			}
		}

		// TODO: Optimization, if all the potentially intersecting polygons are flat, we could
		// just use the original hull.

		// Build carve hull from a convex hull of footprint.
		if len(footPrint) == 0 {
			detailHull.polysIds = detailHull.polysIds[:0]
			continue
		}

		/**
		这里为什么可能是凹的,因为footPrint是多个poly被切割而组成的; 就算都是凸的也没事，这个是保险丝;
		*/
		// 计算凸包 因为切割过了 可能不是凸的;  hull必须是凸的,不然plane的判断方法就失效了;
		CalculateConvexHull(&convexHull, &footPrint)

		// Avoid simplifying the hull if it touches the tile boundary.
		tileOffset2 := NewVector2f(tileOffset.x, tileOffset.z)
		bmin := NewVector2f(tile.header.bmin.x, tile.header.bmin.z).Sub(tileOffset2)
		bmax := NewVector2f(tile.header.bmax.x, tile.header.bmax.z).Sub(tileOffset2)
		// 检测是否有顶点靠近边界; 没有靠近边界,则可以简化顶点;不然可能会跑出边界
		if !HasBoundaryVertices(convexHull, bmin, bmax) {
			SimplifyPolyline(&convexHull, quantSize)
		}

		if len(convexHull) < 3 {
			detailHull.polysIds = detailHull.polysIds[:0]
			continue
		}

		// 前面已经生成过plane了，这里是用切割后的点再次生成plane;只生成x,z平面的
		// 前面的Plane是根据obstacle构建的,这里切割过以后才是真的需要切割的plane;
		// Create hull planes from the polygon
		detailHull.hull = detailHull.hull[:0]
		convexHullCount := len(convexHull)
		for i := 0; i < convexHullCount; i++ {
			position2 := convexHull[i]
			dir2 := convexHull[NextIndex(int32(i), int32(convexHullCount))].Sub(position2)
			len2 := Magnitude2(dir2)
			if len2 <= kEpsilon {
				continue
			}

			dir2 = dir2.Div(len2)
			position := NewVector3f(position2.x, 0, position2.y)
			normal := NewVector3f(-dir2.y, 0, dir2.x)
			detailHull.hull.emplace_back_uninitialized().SetNormalAndPosition(normal, position)
		}
	}

	/**
	直接用3D的hull挖还可能会有另外一个问题：
	比如有一个倾斜20°的obb，然后他特别特别高，是不是这个时候hull就会把obb的投影全部挖掉，但是其实只需要挖到玩家可行走的高度过就行了;
	这个其实不会的;虽然hull是倾斜的obb在x,z上的投影，但是他还有顶和底呀;如果太倾斜还有倾斜两边的plane;能保证不会多切割;

	这里为什么要将一个3Dhull，变成一堆2dhull,不知道为什么，可能就是要这么弄得吧;毕竟切完以后还要各种合并之类的，如果用3DHULL直接切的话,
	合并什么的好像弄不了;
	将一个3D Hull 转换成多个2D Hull，并不是为了简单地“这么做”，而是为了简化后续的几何操作（如切割、合并、布尔运算等），使得整个过程更加高效、稳定和易于控制。
	这种方式非常常见于许多建模工具、物理引擎和游戏引擎中。

	这里的代码没有考虑到，agent可行走的高度，只是切割了被阻挡覆盖的地方，如果阻挡是斜的就没有考虑到;
	*/
	return true
}

func HullFromPoly(hull *Hull, poly []Vector3f) {
	vertCount := len(poly)
	*hull = make([]Plane, vertCount)
	for i := 0; i < vertCount; i++ {
		position := poly[i]
		dir := poly[NextIndex(int32(i), int32(vertCount))].Sub(position)
		normal := NewVector3f(-dir.z, 0, dir.x)
		normal = NormalizeSafe(normal, NewVector3f(0, 0, 0))
		(*hull)[i].SetNormalAndPosition(normal, position)
	}
}

type ClipDetailMeshCallback struct {
	dmesh       *ClippedDetailMesh
	hull        *Hull
	welder      *VertexWelder //64
	inside      *Polygon
	temp        *Polygon
	quantFactor float32
}

func NewDetailMeshClipCallback(dmeshIn *ClippedDetailMesh, hullIn *Hull, welderIn *VertexWelder,
	insideIn *Polygon, tempIn *Polygon, quantFactorIn float32) *ClipDetailMeshCallback {
	return &ClipDetailMeshCallback{
		dmesh:       dmeshIn,
		hull:        hullIn,
		welder:      welderIn,
		inside:      insideIn,
		temp:        tempIn,
		quantFactor: quantFactorIn,
	}
}

const MAGIC_EDGE_DISTANCE = 1e-2

func (this *ClipDetailMeshCallback) process(detailMesh *DetailMesh, poly *DetailMeshPoly, tris []int32, triCount int32) {
	for i := int32(0); i < triCount; i++ {
		t := detailMesh.triangles[tris[i]*4:]
		this.inside.resize_uninitialized(3)
		(*this.inside)[0] = detailMesh.vertices[poly.vertBase+int32(t[0])]
		(*this.inside)[1] = detailMesh.vertices[poly.vertBase+int32(t[1])]
		(*this.inside)[2] = detailMesh.vertices[poly.vertBase+int32(t[2])]
		HullPolygonIntersection(this.inside, this.hull, this.temp, this.quantFactor)
		vertexCount := len(*this.inside)
		if vertexCount < 3 {
			continue
		}

		v0 := this.welder.AddUnique((*this.inside)[0])
		v1 := this.welder.AddUnique((*this.inside)[1])
		for i := 2; i < vertexCount; i++ {
			v2 := this.welder.AddUnique((*this.inside)[i])
			triArea2 := TriArea2D((*this.inside)[0], (*this.inside)[i-1], (*this.inside)[i])
			// 三角形过小就抛弃
			if triArea2 < MAGIC_EDGE_DISTANCE*MAGIC_EDGE_DISTANCE {
				v1 = v2
				continue
			}

			if v0 != v1 && v1 != v2 && v2 != v0 {
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v0))
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v1))
				this.dmesh.triangles = append(this.dmesh.triangles, uint16(v2))
			}
			v1 = v2
		}
	}
}

// 上面是对Poly进行裁剪，这里是对三角形进行裁剪
func ClipDetailMeshes(clipped []*ClippedDetailMesh,
	mesh *DynamicMesh, detailMesh *DetailMesh,
	tile *NavMeshTile,
	tileOffset Vector3f,
	quantFactor float32) {

	queryPadding := NewVector3f(quantFactor*2.0, 0, quantFactor*2.0)
	polyCount := mesh.PolyCount()
	var hull Hull = make([]Plane, 0, 8)
	verts := make([]Vector3f, 0, 8)
	var inside Polygon = make([]Vector3f, 0, 32)
	var temp Polygon = make([]Vector3f, 0, 32)
	welder := NewVertexWelder(64, nil, quantFactor)
	for i := 0; i < polyCount; i++ {
		p := mesh.GetPoly(i)
		// Process only new polygons
		if p.m_Status == kOriginalPolygon {
			continue
		}
		ip := *mesh.GetData(i)
		dpoly := &detailMesh.polys[ip]

		// 为什么点一样就不用处理; 因为vertCount是三角形的点的数量;  m_VertexCount是凸多边形的数量;
		//他们两个的点数量一样,说明这个三角形里面没有特殊的高度信息。 后面直接进行简单将Poly三角化就好了;
		// If the detail mesh does not have any extra vertices,
		// no need to clip, just retriangulate later.
		// 如果细节网格没有任何额外顶点，  不需要剪辑，稍后再重新调整。
		if dpoly.vertCount == int32(p.m_VertexCount) {
			continue
		}

		// 将其构建成hull
		// Build clip hull from the polygons
		verts = make([]Vector3f, p.m_VertexCount)
		for j := uint8(0); j < p.m_VertexCount; j++ {
			verts[j] = mesh.GetVertex(int(p.m_VertexIDs[j]))
		}
		HullFromPoly(&hull, verts)

		// Build query box from the polygon.
		var queryMin, queryMax Vector3f
		queryMin = verts[0]
		queryMax = verts[0]
		vertsCount := len(verts)
		for j := 0; j < vertsCount; j++ {
			queryMin = MinVector3f(queryMin, verts[j])
			queryMax = MaxVector3f(queryMax, verts[j])
		}
		queryMin = queryMin.Sub(queryPadding)
		queryMax = queryMax.Add(queryPadding)
		clipped[i] = NewClippedDetailMesh()
		dmesh := clipped[i]
		dmesh.polyIndex = i
		welder.SetVertexArray(&dmesh.vertices)
		welder.Reset()

		/**
		下面又进行了第二次裁剪，这次裁剪是用切割合并好的三角形进行裁剪的,
		第一次切割是考虑了y的 hull一开始是3D的plane，将其关联的poly都找出来，然后再构建进行切的，第二次我看好像也没构建底部和顶部的Plane呀为什么？
		这里是因为 直接把切割的Polyindex找出来了，然后重新切了一遍。比如我切割好的ploy.index是2,那我只找这个2的进行再次切割;

		这里为什么要再切一遍？
		之前切割是切割的凸多边形，这里是把凸多边形里面的细节三角形进行切割, QueryDetailBVTree只查询dpoly的三角形
		*/
		// Clip all detail triangles against the polygon.
		callback := NewDetailMeshClipCallback(dmesh, &hull, welder, &inside, &temp, quantFactor)
		QueryDetailBVTree(detailMesh, dpoly, queryMin, queryMax, callback) // 查询dpoly里的 queryMin queryMax

		// Offset dmesh back to tile location.
		vertCount := len(dmesh.vertices)
		for j := 0; j < vertCount; j++ {
			dmesh.vertices[j] = dmesh.vertices[j].Add(tileOffset)
		}

		if len(dmesh.vertices) < 3 || len(dmesh.triangles) < 3 {
			clipped[i] = nil
		}
	}
}

func AreColinear(u, v Vector3f, cosAngleAccept float32) bool {
	return FloatAbs(DotVector3f(v, u)) > cosAngleAccept
}

func DistancePointSegmentSqr(pt, s1, s2 Vector2f) float32 {
	ds := s2.Sub(s1)
	dp := pt.Sub(s1)
	den := DotVector2f(ds, ds)
	if den == 0 {
		return DotVector2f(dp, dp)
	}
	t := DotVector2f(ds, dp) / den
	t = FloatClamp(t, 0, 1)
	diff := ds.Mulf(t).Sub(dp)
	return DotVector2f(diff, diff)
}

func SimplifyPolyline(hull *Vertex2Array, thr float32) {
	i := 0
	count := len(*hull)
	for i < count && count > 2 {
		pa := (*hull)[PrevIndex(int32(i), int32(count))]
		pb := (*hull)[i]
		pc := (*hull)[NextIndex(int32(i), int32(count))]
		if DistancePointSegmentSqr(pb, pa, pc) < thr*thr {
			hull.erase(i)
			count--
		} else {
			i++
		}
	}
}

/*
*
这段代码的目的是将一个多边形的每个顶点进行偏移，生成一个新的多边形。通过计算每个顶点的法向量（从相邻的边获得方向），
并根据转角的大小来决定是否平滑转角（小于 90 度的转弯用两个点表示，大于 90 度的转弯则只用一个点表示）。最终结果保存在 dest 数组中。

这里为什么不能矩阵直接放大呢,你肯定会有这个疑问：矩阵放大是直接将每个点都放大指定倍数; 因为他是加的offset;并不是加的倍数;
每个点和中心点的比例是不同的，就比如我有数字 1,10,100 都想+1, 这个用倍数是解决不了的;

还有个grok给的例子：
一个超级细长的三角形（像一根针）
原三角形三个顶点（单位：米）：
A: (0, 0)
B: (100, 1)    ← 离中心很远
C: (100, -1)   ← 离中心也很远

这个三角形长 100 米，高只有 2 米，就像一根横在地图上的极细长针。
几何中心大约在 (50, 0)
实验一：你说的“对每个点都等比放大 1.2 倍”（正确绕中心放大）
新坐标 = 中心 + (原坐标 − 中心) × 1.2
结果：
A (0,0) → 变成大约 (-10, 0)      向左膨胀了 10 米
B (100,1) → 变成大约 (110, 1.2)   向右膨胀了 10 米，上移了 0.2 米
C (100,-1) → 变成大约 (110, -1.2) 向右膨胀了 10 米，下移了 0.2 米

膨胀后这个针变成了：
长度：120 米（从 100 米变成 120 米）
高度：2.4 米（从 2 米变成 2.4 米）
你觉得没问题对吧？肉眼看确实就是整体变大了 20%。
实验二：真正的等距 offset +0.5 米（游戏真正需要的）
每条边向外平行平移 0.5 米，锐角自动切角。
结果：
长度：依然几乎是 100 米（两端变成半圆或小 bevel，最多增加 1 米）
高度：变成 3 米（上下各膨胀 0.5 米）

现在把两个结果直接对比：
方法   				膨胀后长度			膨胀后高度			游戏里会发生什么可怕的事
你说的“每个点等比放大 	1.2”120 米			2.4 米				这根针横穿了原本 20 米宽的通道！AI 直接傻掉，子弹粗了 20 倍
正确 offset +0.5米	≈100米				3米					完美，只是胖了一点点，通道依然通畅

它其实是在实现经典的 “直骨架偏移法”（Straight Skeleton / Offset Polygon with miter/bevel/round） 的简化版，具体特点：
情况,处理方式,对应代码
凸角（>90.1°）,Miter 连接（尖角），法线平均,dm = dla.Add(dlb).Mulf(0.5) + 归一化
锐角（<90.1°）,Bevel 连接（切角），插入两个点,插入两个偏移点
接近90°的角,有 slack，避免误判为锐角,kCos90p1 := -0.00174542 (~cos(90.1°))
偏移量动态调整（那段神秘的 0.25 +,dot,*0.75）
*/
func OffsetPolygon(dest *Vertex2Array, poly Vertex2Array, offset float32) {
	count := int32(len(poly))
	*dest = make([]Vector2f, 0, count)
	for i := int32(0); i < count; i++ {
		curr := poly[i]
		prev := poly[PrevIndex(i, count)]
		next := poly[NextIndex(i, count)]
		diffa := NormalizeSafe2(curr.Sub(prev), NewVector2f(0, 0)) // 是当前顶点到前一个顶点的单位向量，表示前一边的方向。
		diffb := NormalizeSafe2(next.Sub(curr), NewVector2f(0, 0)) // 是当前顶点到下一个顶点的单位向量，表示后一边的方向。

		// Calculate offset vectors based on neighbor segment directions.
		// Scale the offsets to maintain constant line width.
		dla := NewVector2f(-diffa.y, diffa.x) // 得到法向量
		dlb := NewVector2f(-diffb.y, diffb.x) // 得到法向量

		//转弯超过90.1度，加2分。
		//使用松弛，这样大约90度的角不会倾斜（常见的情况与盒子障碍）。
		// More than 90.1 degree turn, add 2 points.
		// Use slack so that about 90 degree corners won't get beveled (common case with box obstacle).
		dot := DotVector2f(diffa, diffb) //diffa;diffb 这两个都是单位向量单位, 单位向量之间的点乘;所以这两个的点乘=cos(角度);
		kCos90p1 := float32(-0.00174542)
		/**
		这里判断的是锐角！！！, diffa = curr-prev， diffb=next-curr, 这两个变是计算的是外夹角不是内夹角,
		想想一下 > 这样两个线段,求出diffa,diffb,的dot是钝角
		*/
		if dot < kCos90p1 { // 点乘判断是否是负数,比这个负数还小，说明两个之间的角度都是大于90度的
			// This is poorman's approximation of a bevel which is offset
			// so that it approximates a circle. Consider 2 cases below.
			//   B._____   A._____.B
			// A.´          |     |
			//  |  x----    |  x  |
			//  |  |        |  |  |
			// A correct version likely needs asin () & co, not used for speed reasons.
			/**
			为何使用 0.25 + FloatAbs(dot) * 0.75：
			这个偏移因子的目的是根据 dot 的绝对值来调整偏移的强度。
			0.25 是一个基础偏移值，它为偏移提供了最小值。
			FloatAbs(dot) * 0.75 动态地调整偏移量，随着 dot 的绝对值增加（即夹角越小，方向越接近），偏移量的增幅也会增大。
			当 dot 接近 1（即两边几乎是平行的），0.75 * FloatAbs(dot) 接近 0.75，这时整体的偏移因子接近 1.0。
			当 dot 接近 0（即夹角接近 90°），FloatAbs(dot) 接近 0，因此偏移因子接近 0.25。
			当 dot 接近 -1（即两边方向相反），FloatAbs(dot) 仍然接近 1，因此偏移因子接近 1.0。
			这意味着：
					夹角越小（即边几乎平行），偏移会越大。
					夹角越大（即边之间的转弯越尖锐），偏移会越小，从而避免产生过大的偏移量，保持平滑。
			diffa.Mulf 是指往线段的这个方向增加
			*/
			pos := curr.Add(diffa.Mulf(0.25 + FloatAbs(dot)*0.75*offset)) // 这里往线段的这个方向增加
			*dest = append(*dest, pos.Add(dla.Mulf(offset)))              // 这里是往线段法线方向,为了将其弄成弧形;
			*dest = append(*dest, pos.Add(dlb.Mulf(offset)))              // 同上
		} else {
			dm := dla.Add(dlb).Mulf(0.5)
			dmr2 := DotVector2f(dm, dm)
			if dmr2 > 0.0 {
				dm = dm.Mulf(1.0 / dmr2)
			}
			*dest = append(*dest, curr.Add(dm.Mulf(offset)))
		}
	}
}

/*
	*步骤,真实意图
	精算45°斜切点,	让XZ投影后的2D轮廓严格包含真实占地（防漏挖,或者多挖）
	扔掉Y信息做2D凸包,		生成最规则、最少的垂直侧壁（利于裁剪和缝合）
	只用1~2个大平面封顶底	,保证100%不穿模，哪怕多挖空气也在所不惜
	倾斜严重时加世界上下平面	,防止极端倾斜导致大平面失效
	底部狠挖-carveDepth,	防止从下面钻进去
	顶部绝不往上扩,	防止把上层地板挖穿出天洞

为什么胶囊体顶部和底部要45°倾斜，直接用不45°直接封顶或者舍弃不行吗，不行,舍弃的话少挖了肯定不行，封顶(直接用一个矩形套住半圆的话),
想想一下胶囊体是躺下来的情况投影在x,z上,45°倾斜会少挖点, 矩形直接套住半圆的话会多挖很多;
*/
func CalculateCarveHullFromPoints(carveHull *Hull, localHullBounds *MinMaxAABB, shape *NavMeshCarveShape,
	tileOffset Vector3f, carveDepth float32, carveWidth float32,
	points []Vector3f, pointCount int32) bool {
	// Calculate convex hull of the obstacle on XZ plane
	*carveHull = make([]Plane, 0)
	var projectedPoints Vertex2Array
	var hull Vertex2Array
	var hullOffset Vertex2Array
	projectedPoints = make([]Vector2f, pointCount)
	for i := int32(0); i < pointCount; i++ {
		projectedPoints[i] = NewVector2f(points[i].x, points[i].z)
	}

	// 这里虽然将3D的数据直接投影到了2D上,如果是斜着的阻挡会比其真正占地面积大很多，但是添加的后面y轴的平面帮你处理了这个问题;添加倾斜的平面，将xz多切的部分还回去
	// 这里为什么要投影，可能是因为如果阻挡是斜着的，他可能就一个角在地上,但是我的用投影将其映射到地上找到占地,但是会出现上面的问题， 这里不考虑角的边上很高 可能agent可能还可以行走;
	/**
	我这里有个疑问,如果是按这样的话那我一个特别斜的物体，比如相对于地面70°，特别贴近地面，会不会这部分地面还会被烘焙成可行走;其实不会，因为
	*carveHull = append(*carveHull, NewPlane(yAxis.Mulf(-1), yAxis.Mulf(distMin-carveDepth)))
	这一句;将其下plane的平面向下移动了carveDepth, 保证不满足agentHeight烘焙掉,查看UnityAI发现也是这样,没问题;
	*/
	CalculateConvexHull(&hull, &projectedPoints)
	SimplifyPolyline(&hull, carveWidth*0.1)
	OffsetPolygon(&hullOffset, hull, carveWidth)

	// Bail out if hull has been degenerated.
	// It is possible that SimplifyPolyline will simplify the obstacle down to a line,
	// OffsetPolygon can take care of that. This is should only happen when the obstacle
	// degenerates to a point.
	if len(hullOffset) < 3 {
		return false
	}
	/*	aaa := hull.Len() > projectedPoints.Len()
		if aaa {

		}
	*/
	*localHullBounds = shape.bounds
	localHullBounds.m_Min = localHullBounds.m_Min.Sub(tileOffset)
	localHullBounds.m_Max = localHullBounds.m_Max.Sub(tileOffset)
	hullCount := int32(len(hullOffset))
	// 构建x,z平面的plane 如果是斜着的不就会很大吗？
	for i := int32(0); i < hullCount; i++ {
		pt := hullOffset[i]
		dir := hullOffset[NextIndex(i, hullCount)].Sub(pt)
		position := NewVector3f(pt.x, 0, pt.y)
		normal := NewVector3f(-dir.y, 0, dir.x)
		normal = NormalizeSafe(normal, NewVector3f(0, 0, 0))
		*carveHull = append(*carveHull, Plane{})
		plane := &(*carveHull)[len(*carveHull)-1]
		plane.SetNormalAndPosition(normal, position)
		localHullBounds.m_Min.x = FloatMin(localHullBounds.m_Min.x, pt.x)
		localHullBounds.m_Min.z = FloatMin(localHullBounds.m_Min.z, pt.y)
		localHullBounds.m_Max.x = FloatMax(localHullBounds.m_Max.x, pt.x)
		localHullBounds.m_Max.z = FloatMax(localHullBounds.m_Max.z, pt.y)
	}

	// Calculate approximate up axis.
	// The approx normal is chosen so that it is visually plausible, i.e. prefer larger extents.
	zero := NewVector3f(0.0, 0.0, 0.0)
	yAxis := zero
	// shape.xAxis.y * FloatMax(shape.extents.y, shape.extents.z) 是将相对于XAixs的extents的y,z应用在y轴上,
	// shape.xAxis.Mulf(xxx) 因为xAixs是标准化的，所以相乘以后应该是和shape.xAxis.y=xxx;相等的
	// yAxis.Add将其y轴累加起来
	// 如果物体的 x 轴“朝上”（xAxis.y 大），并且物体在与之垂直的方向上很长（extents 大），那么这个 x 轴对“整体上方向”的贡献就应该更大。于是把它按权重加入 yAxis 的累加和里。
	// 三行合在一起就是把三根局部轴按「它们朝上程度 × 相关尺寸」加权求和，得到一个“综合的上方向”向量。
	/**
	我们不知道这个障碍物哪个面是‘上’，但可以通过一个启发式规则来猜测：
	哪个局部轴最朝上，并且在垂直于它的平面上物体拉得最长（最扁），哪个轴就最有可能是上方向。
	把三个轴按这个规则加权求和，再归一化，就得到了一个视觉上最合理的近似上方向。”

	shape.xAxis.y是标准化过的, shape.xAxis.y * FloatMax(shape.extents.y, shape.extents.z)就是将y或者z投影到xAxis的y轴上,
	// 给局部 X 轴投票：你朝上吗？而且你是不是一根长长的东西？
	yAxis += xAxis * (xAxis朝上程度 * 垂直于X轴的扁平程度)
	// 给局部 Y 轴投票
	yAxis += yAxis * (yAxis朝上程度 * 垂直于Y轴的扁平程度)
	// 给局部 Z 轴投票
	yAxis += zAxis * (zAxis朝上程度 * 垂直于Z轴的扁平程度)
	// 最后把三票加起来，归一化 → 就是最合理的“上方向”
	如果y轴最后是负的也没关系，因为下面的代码根本无所谓是正的还是负的,下面会建立底部的hull和顶部的hull
	*/
	yAxis = yAxis.Add(shape.xAxis.Mulf(shape.xAxis.y * FloatMax(shape.extents.y, shape.extents.z)))
	yAxis = yAxis.Add(shape.yAxis.Mulf(shape.yAxis.y * FloatMax(shape.extents.z, shape.extents.x)))
	yAxis = yAxis.Add(shape.zAxis.Mulf(shape.zAxis.y * FloatMax(shape.extents.x, shape.extents.y)))
	yAxis = NormalizeSafe(yAxis, zero)
	worldYAxis := NewVector3f(0.0, 1.0, 0.0)
	if CompareApproximately(yAxis, zero, kEpsilon) {
		yAxis = worldYAxis
	}

	distMin := float32(math.MaxFloat32)
	distMax := float32(-math.MaxFloat32)
	for i := int32(0); i < pointCount; i++ {
		dist := DotVector3f(points[i], yAxis)
		distMin = FloatMin(distMin, dist)
		distMax = FloatMax(distMax, dist)
	}

	// 构建y轴hull
	// Add top/bottom caps
	// carveDepth 和 radius一个意思，扩大agentHeight大小
	*carveHull = append(*carveHull, NewPlane(yAxis.Mulf(-1), yAxis.Mulf(distMin-carveDepth))) // 底面
	*carveHull = append(*carveHull, NewPlane(yAxis, yAxis.Mulf(distMax)))                     // 顶面

	// 最终效果 几乎竖直的物体，挖的洞非常紧凑；斜得厉害的，就老老实实多挖一点保证不漏。
	// The aabb top/bottom planes if needed
	cosAngleConsiderAxisAligned := float32(0.984807753012208) // Consider colinear if within 10 degrees
	isAlmostAxisAlignedY := AreColinear(yAxis, worldYAxis, cosAngleConsiderAxisAligned)
	if !isAlmostAxisAlignedY {
		min := shape.bounds.m_Min.Sub(tileOffset)
		max := shape.bounds.m_Max.Sub(tileOffset)
		// 这里如果和世界坐标的0,1,0有超过10度的偏移，就再加两个平面来保证y
		//直接加两个相对于世界坐标的0,1,0和(0,-1,0)的上下两个Hull,防止倾斜太厉害上下两个Plane漏掉,导致没有过滤y轴的三角形;
		// min.Sub(NewVector3f(0, 1, 0).Mulf(carveDepth))) 这里其实就是 min向量减去carveDepth,   NewVector3f(0, 1, 0).Mulf(carveDepth)是计算carveDepth的向量
		*carveHull = append(*carveHull, NewPlane(worldYAxis.Mulf(-1), min.Sub(NewVector3f(0, 1, 0).Mulf(carveDepth))))
		*carveHull = append(*carveHull, NewPlane(worldYAxis, max))
	} else {
		diagonal2D := localHullBounds.m_Max.Sub(localHullBounds.m_Min)
		diagonal2D.y = 0 // 获取max-min = obb的x和z轴的长度的向量
		//这里最多有10%的偏移;
		halfSpread := 0.5 * Magnitude(diagonal2D)                 // 获取向量的长度.*0.5
		tanAngleConsiderAxisAligned := float32(0.176326980708464) // Cap slanted up to 10 degrees
		maxCapDepth := tanAngleConsiderAxisAligned * halfSpread   // 找到偏移10%最多会高出来的长度;
		localHullBounds.m_Min.y -= maxCapDepth                    // +-上这个长度 保证斜着不会被少切割
		localHullBounds.m_Max.y += maxCapDepth
	}

	/**
	// 这里为什么只减不加, 因为底部必须狠挖（防止角色从下面钻进去）
	顶部不能增加，因为plane本来就是粗略的，如果上层刚好是一个地板，可能会导致地板出现一个洞！！！;
	操作是否安全为什么m_Min.y -= carveDepth100% 安全防止地面 poly 漏掉，误杀也没事（本来就该杀）m_Max.y += carveDepth极其危险会导致远处高空的 poly 被误杀，空中出现莫名其妙的洞
	你想的“反正有 plane 精筛”在 99.9% 情况下是对的，
	但在 0.1% 的极端角落 + 优化路径 + 浮点误差下，
	它会变成一个几乎无法复现、玩家疯狂骂街的空中穿模 bug。
	所以引擎作者宁可让顶部多留一点空气，也绝不往上扩一毫米。
	现在你明白为什么这行代码是“只减不加”的终极原因了吧？
	这不是逻辑问题，是用无数血泪换来的黑魔法。
	*/
	// 这4个平面是用来过滤y轴方向的 这里-carveDepth;保证不会后面少过滤poly;后面用localHullBounds来过滤poly的;
	localHullBounds.m_Min.y -= carveDepth
	return true
}

// Compute the set of planes defining an extruded bounding box.
// Bounding box is represented by transform and size.
// Extrusion is based on 'carveWidth' horizontally and 'carveDepth' vertically down.
// Everything is translated relative to 'tileOffset'.
func CalculateBoxHull(
	carveHull *Hull, localHullBounds *MinMaxAABB,
	shape *NavMeshCarveShape, tileOffset Vector3f,
	carveDepth, carveWidth float32) bool {
	// Calculate obstacle vertices.
	var box [8]Vector3f
	for i := 0; i < 8; i++ {
		box[i] = shape.center.Sub(tileOffset)
		x := shape.extents.x
		if i&1 == 0 {
			x = -shape.extents.x
		}
		box[i] = box[i].Add(shape.xAxis.Mulf(x))
		y := shape.extents.y
		if i&2 == 0 {
			y = -shape.extents.y
		}
		box[i] = box[i].Add(shape.yAxis.Mulf(y))
		z := shape.extents.z
		if i&4 == 0 {
			z = -shape.extents.z
		}
		box[i] = box[i].Add(shape.zAxis.Mulf(z))
	}

	return CalculateCarveHullFromPoints(carveHull, localHullBounds, shape, tileOffset, carveDepth, carveWidth, box[:], 8)
}

/*
*
圆柱体或胶囊体的横向划分数量（类似于把圆周分成 8 份）。
在几何计算里，通常我们用 kDivs 来控制圆周多边形的精度：
kDivs = 8 → 每个圆形被近似成 8 个点，就是一个八边形。
分得越多，圆柱体或胶囊体的多边形越圆滑。
*/
const kDivs = 8

/*
*
详细可以看图 CalculateCapsuleHull.png
*/
func CalculateCapsuleHull(
	carveHull *Hull, localHullBounds *MinMaxAABB,
	shape *NavMeshCarveShape, tileOffset Vector3f,
	carveDepth, carveWidth float32) bool {
	// TODO: it should be possible to optimize the hull shape a bit more by
	// creating the capsule data in 2D, and add min/max points.
	// See how a 2D capsule is drawn in NavMeshVisulization.cpp

	var radius float32 = 0
	var height float32 = 0
	FitCapsuleToExtents(&radius, &height, shape.extents)

	// Calculate obstacle vertices.
	var cylinder [(kDivs + kDivs + 1) * 2]Vector3f
	{
	}
	var n int32 = 0
	/**
	内接圆是在多边形内部的圆，多边形里面有一个圆;
	外接圆是在多边形外部的圆，多边形外面有一个圆;

	这里想把胶囊体转换成 kDivs(默认是8)条边的多边体,
	计算外接圆半径 R = r/cos(pi/k); r是内接圆半径; k是边数;  pi/k 也被称作半角; 具体问AI

	radiusScale := float32(1.0 / math.Cos(math.Pi*2.0/float64(kDivs)*0.5))
	为什么不这么写 radiusScale := float32(1.0 / math.Cos(math.Pi/float64(kDivs)))
	数学里2pi弧度=360度;
	圆被分成 kDivs 等份，每份角度 = 2π / kDivs;取半角就是再*0.5;
	为什么是1.0/xxx, 而不是内接圆半径/xxx;因为代码一直有个计算技巧就是，比如要计算 a/b;就先计算c=1/b, 然后再计算a*c;


	*/
	// Scale for "outer" polygon, the polygon is created so that the cylinder circle is inscribing the polygon.
	radiusScale := float32(1.0 / math.Cos(math.Pi*2.0/float64(kDivs)*0.5)) // 内接圆/外接圆的比例
	// We have 8 divs effectively in other direction too.
	// 这里比如是个直线，让其偏移45°变成斜的
	h := float32(0.7071067812) * radius * radiusScale // 沿 45°方向; h = r * 0.707（约等于 45°方向外沿） sin(45)=cos(45)=√2/2≈0.7071
	r := radius * radiusScale                         //获取外接圆半径  radius:是胶囊体的半径,多边形应该是要包含胶囊体的,所以radius是内接圆半径;r就是外接圆半径了
	center := shape.center.Sub(tileOffset)
	for i := 0; i < kDivs; i++ {

		//把圆周分成 kDivs 等份，计算当前点的角度。圆周一圈是 2π，所以第 i 个点的角度是：angle=i/jDive *2pi
		angle := float64(i) / float64(kDivs) * math.Pi * 2.0
		dx := math.Cos(angle) // 对应 X 轴方向的单位坐标 这是基于基向量做的，
		dz := math.Sin(angle)
		ax := shape.xAxis.Mulf(float32(dx)) //轴上面 再次投影dx,获取该边的实际长度,这里是将基向量变成x轴,相当于一个变换
		az := shape.zAxis.Mulf(float32(dz))
		/*
			这下面4个点就是，y轴上的4个点,我们不是要把胶囊分成8面体吗,
				第一个点就是胶囊的最下面胶囊半圆处 0 下面半圆的y轴最低的那个点;
				第二个点就是就是下面的半圆和矩阵的交界处；
				第3个点就是就是上面的半圆和矩阵的交界处；
				第4个点就是就是最上面的半圆的那个尖尖处;
		*/
		// ax.Mulf(r) az.Mulf(r) 把这两个轴都正常弄上 相加,
		cylinder[n] = center.Add(ax.Mulf(r)).Add(az.Mulf(r)).Sub(shape.yAxis.Mulf(height))
		n++
		cylinder[n] = center.Add(ax.Mulf(r)).Add(az.Mulf(r)).Add(shape.yAxis.Mulf(height))
		n++
		//  这里比如是个直线，让其偏移45°变成斜的  ax.Mulf(h) az.Mulf(h)是定义x轴上的
		// 比如我有一个直线是竖着的,现在我要往45°偏移,那不就是长和高都短了吗,
		// 所以ax.Mulf(h)).Add(az.Mulf(h), 还有shape.yAxis.Mulf(height + h)是三个轴的45°偏移，
		// 这里为什么是height + h，因为这个偏移是在胶囊的半圆那里的的呀,所以y轴要+height, 不能光用h;
		cylinder[n] = center.Add(ax.Mulf(h)).Add(az.Mulf(h)).Sub(shape.yAxis.Mulf(height + h))
		n++
		cylinder[n] = center.Add(ax.Mulf(h)).Add(az.Mulf(h)).Add(shape.yAxis.Mulf(height + h))
		n++
	}

	// Capsule tips
	cylinder[n] = center.Sub(shape.yAxis.Mulf(height + r))
	n++
	cylinder[n] = center.Add(shape.yAxis.Mulf(height + r))
	n++

	return CalculateCarveHullFromPoints(carveHull, localHullBounds, shape, tileOffset, carveDepth, carveWidth, cylinder[:], n)
}

// Set flags on polygon edges colinear to tile edges.
// Flagged edges are considered when dynamically stitching neighboring tiles.
func WritePortalFlags(verts []Vector3f, polys []NavMeshPoly, polyCount int32, sourceHeader *NavMeshDataHeader) {
	bmax := sourceHeader.bmax
	bmin := sourceHeader.bmin
	for ip := int32(0); ip < polyCount; ip++ {
		poly := &polys[ip]
		for iv := uint8(0); iv < poly.vertCount; iv++ {
			// Skip already connected edges
			if poly.neis[iv] != 0 {
				continue
			}

			vert := verts[poly.verts[iv]]
			ivn := iv + 1
			if iv+1 == poly.vertCount {
				ivn = 0
			}
			nextVert := verts[poly.verts[ivn]]

			//
			//       z+
			//    o---->o
			//    ^     |
			// x- |     | x+
			//    |     v
			//    o<----o
			//       z-

			// 这里为什么要dtMax;因为怕凸多边形是斜的;但是应该不可能是斜的呀
			// 不过这里因为是取得abs最大的,可能是怕小数有误差不能相等吧;如果和bmax.x距离的最大值都不超过MAGIC_EDGE_DISTANCE;那就基本可以说明边就是里边界特别特别近的了;
			// 代码里应该还有其他如果三角形的面积过小就排除掉，排除了MAGIC_EDGE_DISTANCE小于这个的距离的情况下，还能 产生凸多边形
			dx := nextVert.x - vert.x
			dz := nextVert.z - vert.z
			nei := uint16(0)
			if dz < 0.0 && FloatMax(FloatAbs(vert.x-bmax.x), FloatAbs(nextVert.x-bmax.x)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 0 // x+ portal
			} else if dx > 0.0 && FloatMax(FloatAbs(vert.z-bmax.z), FloatAbs(nextVert.z-bmax.z)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 2 // z+ portal
			} else if dz > 0.0 && FloatMax(FloatAbs(vert.x-bmin.x), FloatAbs(nextVert.x-bmin.x)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 4 // x- portal
			} else if dx < 0.0 && FloatMax(FloatAbs(vert.z-bmin.z), FloatAbs(nextVert.z-bmin.z)) < MAGIC_EDGE_DISTANCE {
				nei = kNavMeshExtLink | 6 // z- portal
			}
			poly.neis[iv] = nei
		}
	}
}

func SimplePolygonTriangulation(dtl *NavMeshPolyDetail, dtris []NavMeshPolyDetailIndex, detailTriBase int32, polygonVertexCount int32) int32 {
	dtl.vertBase = 0
	dtl.vertCount = 0
	dtl.triBase = uint32(detailTriBase)
	dtl.triCount = NavMeshPolyDetailIndex(polygonVertexCount - 2)

	// Triangulate polygon (local indices).
	for j := int32(2); j < polygonVertexCount; j++ {
		t := dtris[4*detailTriBase:]
		t[0] = 0
		t[1] = NavMeshPolyDetailIndex(j - 1)
		t[2] = NavMeshPolyDetailIndex(j)
		// Bit for each edge that belongs to poly boundary.
		t[3] = 1 << 2
		if j == 2 {
			t[3] |= 1 << 0
		}
		if j == polygonVertexCount-1 {
			t[3] |= 1 << 4
		}
		detailTriBase++
	}
	return detailTriBase
}

func GetEdgeFlags(va Vector3f, poly []Vector3f) byte {
	// Return mask indicating which edges the vertex touches.
	thrSqr := Sqr(MAGIC_EDGE_DISTANCE)
	npoly := len(poly)
	var flags byte = 0
	for i, j := 0, npoly-1; i < npoly; i, j = i+1, i {
		var t float32
		if SqrDistancePointSegment2D(&t, va, poly[j], poly[i]) < thrSqr {
			flags |= 1 << uint8(j)
		}
	}
	return flags
}

func GetTriFlags(va, vb, vc byte) byte {
	var flags byte = 0
	if (va & vb) != 0 {
		flags |= 1 << 0
	}
	if (vb & vc) != 0 {
		flags |= 1 << 2
	}
	if (vc & va) != 0 {
		flags |= 1 << 4
	}
	return flags
}

func TileToDynamicMesh(tile *NavMeshTile, mesh *DynamicMesh, tileOffset Vector3f) {

	vertCount := tile.header.vertCount
	polyCount := tile.header.polyCount
	mesh.Reserve(vertCount, polyCount)
	for iv := int32(0); iv < vertCount; iv++ {
		mesh.AddVertex(tile.verts[iv].Sub(tileOffset))
	}

	for ip := int32(0); ip < polyCount; ip++ {
		srcPoly := tile.polys[ip]
		mesh.AddPolygon3(srcPoly.verts[:], DataType(ip), int32(srcPoly.vertCount))
	}
}

func DynamicMeshToTile(dataSize *uint32, mesh *DynamicMesh, clipped []*ClippedDetailMesh,
	sourceTile *NavMeshTile, tileOffset Vector3f) []byte {

	// Determine data size
	vertCount := mesh.VertCount()
	polyCount := mesh.PolyCount()
	sourceHeader := sourceTile.header
	totVertCount := vertCount
	totPolyCount := polyCount
	detailVertCount := int32(0)
	detailTriCount := int32(0)
	RequirementsForDetailMeshMixed(&detailVertCount, &detailTriCount, mesh, sourceTile, clipped)
	headSize := Align4(unsafe.Sizeof(NavMeshDataHeader{}))
	vertSize := Align4(uintptr(totVertCount) * unsafe.Sizeof(Vector3f{}))
	polySize := Align4(uintptr(totPolyCount) * unsafe.Sizeof(NavMeshPoly{}))
	detailMeshesSize := Align4(uintptr(polyCount) * unsafe.Sizeof(NavMeshPolyDetail{}))
	detailVertsSize := Align4(uintptr(detailVertCount) * unsafe.Sizeof(Vector3f{}))
	detailTrisSize := Align4(uintptr(detailTriCount) * 4 * unsafe.Sizeof(NavMeshPolyDetailIndex(0)))
	bvTreeSize := uint32(0)
	newSize := headSize + vertSize + polySize +
		detailTrisSize + detailVertsSize + detailMeshesSize + bvTreeSize
	newTile := make([]byte, newSize+1)
	if newTile == nil {
		*dataSize = 0
		return nil
	}
	*dataSize = newSize

	// Serialize in the detour recognized format
	header := (*NavMeshDataHeader)(unsafe.Pointer(&(newTile[0])))
	d := headSize
	var verts []Vector3f
	sliceHeader := (*reflect.SliceHeader)(unsafe.Pointer(&(verts)))
	sliceHeader.Cap = int(totVertCount)
	sliceHeader.Len = int(totVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += vertSize

	var polys []NavMeshPoly
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(polys)))
	sliceHeader.Cap = int(totPolyCount)
	sliceHeader.Len = int(totPolyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += polySize

	var detail []NavMeshPolyDetail
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(detail)))
	sliceHeader.Cap = int(polyCount)
	sliceHeader.Len = int(polyCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailMeshesSize

	var dverts []Vector3f
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(dverts)))
	sliceHeader.Cap = int(detailVertCount)
	sliceHeader.Len = int(detailVertCount)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailVertsSize

	var dtris []NavMeshPolyDetailIndex
	sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(dtris)))
	sliceHeader.Cap = int(detailTriCount * 4)
	sliceHeader.Len = int(detailTriCount * 4)
	sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	d += detailTrisSize

	/*
		var bvTree []NavMeshBVNode
		sliceHeader = (*reflect.SliceHeader)(unsafe.Pointer(&(bvTree)))
		sliceHeader.Cap = int(header.bvNodeCount)
		sliceHeader.Len = int(header.bvNodeCount)
		sliceHeader.Data = uintptr(unsafe.Pointer(&(newTile[d])))
	*/
	d += bvTreeSize
	Assert(d == newSize)
	for iv := 0; iv < vertCount; iv++ {
		// TODO: apply tile offset earlier, after carving. Now needs to be handled all over the place.
		verts[iv] = mesh.GetVertex(iv).Add(tileOffset)
	}

	for ip := 0; ip < polyCount; ip++ {
		p := mesh.GetPoly(ip)
		sourcePolyIndex := *mesh.GetData(ip)
		srcPoly := sourceTile.polys[sourcePolyIndex]
		poly := &polys[ip]
		copy(poly.verts[:], p.m_VertexIDs[:])
		copy(poly.neis[:], p.m_Neighbours[:])
		area := srcPoly.area
		poly.flags = 1 << area
		poly.area = area
		poly.vertCount = p.m_VertexCount
	}

	// Set external portal flags
	WritePortalFlags(verts, polys, int32(polyCount), sourceHeader)
	WriteDetailMeshMixed(detail, dverts, dtris, mesh, sourceTile, tileOffset, clipped,
		int32(detailTriCount), int32(detailVertCount))

	// Copy values from source
	*header = *sourceHeader

	// (re)set new tile values
	header.polyCount = int32(totPolyCount)
	header.vertCount = int32(totVertCount)
	header.detailMeshCount = int32(polyCount)
	header.detailVertCount = int32(detailVertCount)
	header.detailTriCount = int32(detailTriCount)
	header.bvNodeCount = 0 // Fixme: bv-tree

	return newTile
}

func RequirementsForDetailMeshMixed(detailVertCount *int32, detailTriCount *int32,
	mesh *DynamicMesh, sourceTile *NavMeshTile, clipped []*ClippedDetailMesh) {
	vertCount := NavMeshPolyDetailIndex(0)
	triCount := NavMeshPolyDetailIndex(0)

	// Collect sizes needed for detail mesh
	polyCount := mesh.PolyCount()
	for ip := 0; ip < polyCount; ip++ {
		p := mesh.GetPoly(ip)
		sourcePolyIndex := *mesh.GetData(ip)
		if p.m_Status == kOriginalPolygon {
			// When preserving polygon detail mesh just add the source counts
			sourceDetail := sourceTile.detailMeshes[sourcePolyIndex]
			vertCount += sourceDetail.vertCount
			triCount += sourceDetail.triCount
		} else {
			if clipped[ip] != nil {
				vertCount += NavMeshPolyDetailIndex(len(clipped[ip].vertices))
				triCount += NavMeshPolyDetailIndex(len(clipped[ip].triangles)) / 3
			} else {
				// Simple triangulation needs n-2 triangles but no extra detail vertices
				triCount += NavMeshPolyDetailIndex(p.m_VertexCount) - 2
			}
		}
	}
	*detailVertCount = int32(vertCount)
	*detailTriCount = int32(triCount)
}

func WriteDetailMeshMixed(detail []NavMeshPolyDetail, dverts []Vector3f, dtris []NavMeshPolyDetailIndex,
	mesh *DynamicMesh, sourceTile *NavMeshTile, tileOffset Vector3f,
	clipped []*ClippedDetailMesh, detailTriCount int32, detailVertCount int32) {

	detailVertBase := uint32(0)
	detailTriBase := uint32(0)
	var edgeFlags []byte
	var poly []Vector3f

	polyCount := mesh.PolyCount()
	for ip := 0; ip < polyCount; ip++ {
		dtl := &detail[ip]
		p := mesh.GetPoly(ip)
		if p.m_Status == kOriginalPolygon {
			// Fill in the original detail mesh for this polygon
			sourcePolyIndex := *mesh.GetData(ip)
			sourceDetail := sourceTile.detailMeshes[sourcePolyIndex]
			dtl.vertBase = detailVertBase
			dtl.vertCount = sourceDetail.vertCount
			dtl.triBase = detailTriBase
			dtl.triCount = sourceDetail.triCount

			// copy source detail vertices and triangles
			size := uint32(sourceDetail.vertCount)
			copy(dverts[detailVertBase:detailVertBase+size], sourceTile.detailVerts[sourceDetail.vertBase:sourceDetail.vertBase+size])
			size = uint32(4 * uintptr(sourceDetail.triCount))
			copy(dtris[4*detailTriBase:4*detailTriBase+size], sourceTile.detailTris[4*sourceDetail.triBase:4*sourceDetail.triBase+size])

			detailVertBase += uint32(sourceDetail.vertCount)
			detailTriBase += uint32(sourceDetail.triCount)
		} else {
			if clipped[ip] != nil {
				poly = make([]Vector3f, p.m_VertexCount)
				for j := uint8(0); j < p.m_VertexCount; j++ {
					poly[j] = tileOffset.Add(mesh.GetVertex(int(p.m_VertexIDs[j])))
				}

				// TODO: check vertex count so that detail vertex index won't overflow.
				// TODO: locate and remap polygon vertices to reduce space (now stores poly vertices too).
				clip := clipped[ip]
				vertCount := len(clip.vertices)
				triCount := len(clip.triangles) / 3
				dtl.vertBase = detailVertBase
				dtl.vertCount = NavMeshPolyDetailIndex(vertCount)
				dtl.triBase = detailTriBase
				dtl.triCount = NavMeshPolyDetailIndex(triCount)

				// Copy vertices
				for j := uint32(0); j < uint32(vertCount); j++ {
					dverts[detailVertBase+j] = clip.vertices[j]
				}

				// Calculate edge flags.
				edgeFlags = make([]byte, vertCount)
				for j := 0; j < vertCount; j++ {
					edgeFlags[j] = GetEdgeFlags(clip.vertices[j], poly)
				}

				// Copy triangles.
				for j := uint32(0); j < uint32(triCount); j++ {
					t := dtris[4*(detailTriBase+j):]

					// 看 duDebugDrawNavMeshPoly的dd->vertex(&tile->detailVerts[(pd->vertBase+t[j]-poly->vertCount)*3], c); 就知道了
					// 这里感觉不需要加 p.m_VertexCount; 到时候可以运行的时候调试下;看下哪个是对的
					// 因为原始存储的时候只存储三角形内部的点;用的时候需要 -vertCount; 所以这里直接加上m_VertexCount;以便于之前减;
					t[0] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+0])
					t[1] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+1])
					t[2] = NavMeshPolyDetailIndex(uint16(p.m_VertexCount) + clip.triangles[j*3+2])
					t[3] = NavMeshPolyDetailIndex(GetTriFlags(edgeFlags[clip.triangles[j*3+0]],
						edgeFlags[clip.triangles[j*3+1]],
						edgeFlags[clip.triangles[j*3+2]]))
				}

				detailVertBase += uint32(vertCount)
				detailTriBase += uint32(triCount)
			} else {
				// 如果没有切割过 就直接将ploy 直接重新切割成简单三角形
				detailTriBase = uint32(SimplePolygonTriangulation(dtl, dtris, int32(detailTriBase), int32(p.m_VertexCount)))
			}
		}
	}
	Assert(detailTriBase == uint32(detailTriCount))
	Assert(detailVertBase == uint32(detailVertCount))
}
