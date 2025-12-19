package unityai

const MAX_OUTPUT_VERTICES = 32
const PLANE_FLAG byte = 0x80

const PLANE_INDEX_MASK = PLANE_FLAG - 1

func DegenerateTriangle(tri Polygon) bool {
	Assert(len(tri) == 3)
	ab := tri[1].Sub(tri[0])
	ac := tri[2].Sub(tri[0])
	n := Cross(ab, ac)
	areaSq := SqrMagnitude(n)
	return areaSq == 0
}

func IsSafeConvex(vertices []Vector3f) bool {
	vertexCount := int32(len(vertices))
	for i := int32(0); i < vertexCount; i++ {
		v0 := vertices[PrevIndex(i, vertexCount)]
		v1 := vertices[i]
		v2 := vertices[NextIndex(i, vertexCount)]
		triArea := TriArea2D(v0, v1, v2)
		if triArea <= 1e-2 {
			return false
		}
	}
	return true
}

func FindFurthest(plane Plane, vertices []Vector3f, quantFactor float32) int {
	bestIndex := -1
	bestDist := quantFactor
	for iv := 0; iv < len(vertices); iv++ {
		dist := plane.GetDistanceToPoint(vertices[iv])
		if dist > bestDist {
			bestDist = dist
			bestIndex = iv
		}
	}
	return bestIndex
}

func PolygonDegenerate(vertexCount int32, indices []uint16, vertices []Vector3f, quantFactor float32) bool {
	if vertexCount < 3 {
		return true
	} // 点小于3个
	area := float32(0.0)
	maxSideSq := float32(0.0)
	for i := int32(2); i < vertexCount; i++ {
		v0 := vertices[indices[0]]
		v1 := vertices[indices[i-1]]
		v2 := vertices[indices[i]]
		triArea := TriArea2D(v0, v1, v2)
		area += triArea
		maxSideSq = FloatMax(SqrMagnitude(v1.Sub(v0)), maxSideSq)
		maxSideSq = FloatMax(SqrMagnitude(v2.Sub(v0)), maxSideSq)
	}
	if area <= 0 {
		return true
	} // 面积小于等于0
	safety := 1e-2 * quantFactor
	return area*area <= safety*safety*maxSideSq // 面积过小
	// 这里为什么要maxSideSq呢;
	/**
	如果没有maxSideSq  那么阈值 safety 是个固定的绝对数值（1e-2 * quantFactor）。这会导致不同大小的多边形被同一个标准判断。
	这里把面积阈值与 maxSideSq（最长边的平方）挂钩，
	相当于 “根据多边形的尺寸自适应地判断是否退化”。

	这样小边长的 polygon，如果面积也小，不算退化；

	退化的多边形其实就是：“点几乎都在一条直线上”。如果你只看 area，没法知道“在多大尺度下”算小。
	但如果你看的是area / maxSide²（面积与边长平方的比值），那就能判断它是否“扁平”。
	而 area² <= safety² * maxSideSq其实就是在比较这个比值是否低于阈值。
	*/
}

func (this *DynamicMesh) CreatePolygon(vertices Polygon, status PolyStatus) Poly {
	vertexCount := int32(len(vertices))
	Assert(vertexCount <= kNumVerts)
	Assert(vertexCount > 2)

	// Ensure neighbour ids are zero'ed
	newPoly := Poly{}

	newPoly.m_VertexCount = uint8(vertexCount)
	newPoly.m_Status = PolyStatus(status)
	for i := int32(0); i < vertexCount; i++ {
		vi := this.m_Welder.AddUnique(vertices[i])
		Assert(vi < 0xffff) //< vertex overflow
		newPoly.m_VertexIDs[i] = uint16(vi)
	}
	return newPoly
}

func (this *DynamicMesh) RemovePolygonUnordered(i int) {
	Assert(i < len(this.m_Polygons))
	Assert(len(this.m_Data) == len(this.m_Polygons))
	this.m_Polygons[i] = this.m_Polygons[len(this.m_Polygons)-1]
	this.m_Polygons = this.m_Polygons[:len(this.m_Polygons)-1]

	this.m_Data[i] = this.m_Data[len(this.m_Data)-1]
	this.m_Data = this.m_Data[:len(this.m_Data)-1]
}

func (this *DynamicMesh) CollapseEdge(va, vb int) {
	for i := 0; i < len(this.m_Polygons); i++ {
		poly := &this.m_Polygons[i]
		// 检测所有凸多边形 如果顶点是va得  就将其设置为vb;
		for j := uint8(0); j < poly.m_VertexCount; j++ {
			if poly.m_VertexIDs[j] == uint16(va) {
				poly.m_VertexIDs[j] = uint16(vb)
			}
		}
	}
}

/*
*
这里为什么没有直接删多边形
1. 邻接关系（邻居索引）断裂
其他多边形可能还“以为”这个 polygon 存在；
这些多边形的邻居引用或共享边会失效；
下游函数（例如 ConnectPolygons()、FindNeighbors()）可能访问无效的 index。

 2. 顶点无法安全释放

多边形共享顶点；
如果这个 polygon 被直接删除，而它的顶点也被删，
其他还在用这些顶点的 polygon 会指向错误内存；
如果不删顶点，那 mesh 就会越来越多“孤点”。

 3. 精度问题下的“伪退化”多边形

有时 polygon 看起来退化（比如几乎共线），但面积不完全为零。
这类 polygon 可以通过坍塌边让它平滑地“变短”，避免删除时引入拓扑不连续。
*/
func (this *DynamicMesh) CollapsePolygonUnordered(ip int) {
	Assert(ip < len(this.m_Polygons))
	Assert(len(this.m_Data) == len(this.m_Polygons))
	poly := this.m_Polygons[ip]
	var edgeLengths [kNumVerts]float32
	for i := uint8(0); i < poly.m_VertexCount; i++ {
		j := uint8(0)
		if i+1 < poly.m_VertexCount {
			j = i + 1
		}
		va := this.m_Vertices[poly.m_VertexIDs[i]]
		vb := this.m_Vertices[poly.m_VertexIDs[j]]
		edgeLengths[i] = SqrMagnitude(va.Sub(vb)) // 测量所有边长
	}

	// Collapse polygon to line, by collapsing the shortest edge at a time.
	for poly.m_VertexCount > 2 { // 循环坍塌短边
		// Find shortest edge
		shortestDist := edgeLengths[0]
		shortest := uint8(0)
		for i := uint8(1); i < poly.m_VertexCount; i++ {
			if edgeLengths[i] < shortestDist {
				shortestDist = edgeLengths[i]
				shortest = i
			}
		}
		//  如果比 m_QuantFactor 长就break
		if shortestDist > this.m_QuantFactor*this.m_QuantFactor {
			break
		}

		// 下面是这两个点太靠近了 将两个点只要用 其中一个点就好了,处理好以后 还是要把这个凸多边形删除
		next := uint8(0)
		if shortest+1 < poly.m_VertexCount {
			next = shortest + 1
		}
		va := poly.m_VertexIDs[shortest]
		vb := poly.m_VertexIDs[next]

		// 检测所有凸多边形 如果顶点是va得  就将其设置为vb;
		// Collapse edge va->vb  实际坍塌顶点
		if va != vb {
			this.CollapseEdge(int(va), int(vb))
		}

		for i := shortest; i < poly.m_VertexCount-1; i++ {
			edgeLengths[i] = edgeLengths[i+1]
			poly.m_VertexIDs[i] = poly.m_VertexIDs[i+1]
		}
		poly.m_VertexCount--
	}

	this.RemovePolygonUnordered(ip)
}

func SplitPoly(inside *Polygon, poly Polygon, plane Plane, quantFactor float32, usedEdges []byte, ip int32) int32 {
	vertexCount := len(poly)

	/**
	这里为什么用32:
	因为 用多个平面裁剪一个多边形，最终得到的交集多边形（inside部分）的顶点数，最大可能值 = 原多边形顶点数 + 所有裁剪平面的数量（每平面贡献一个新顶点）
	多边形顶点最大默认是6;平面最大数量圆柱来算;圆柱默认8个边;上下两个;如果倾斜再加2个 = 6+8+2+2 = 18; 也就是说最大就是18;32是安全值;
	*/
	// Worst case number of vertices is kNumVerts + hull clipping planes
	Assert(vertexCount < MAX_OUTPUT_VERTICES)
	var dist [MAX_OUTPUT_VERTICES]float32

	// Compute signed distance to plane for each vertex
	distance := plane.GetDistanceToPoint(poly[0])
	if FloatAbs(distance) < quantFactor {
		distance = 0
	}

	var minDistance, maxDistance float32
	minDistance = distance
	maxDistance = distance
	dist[0] = distance
	for iv := 1; iv < vertexCount; iv++ {
		v := poly[iv]
		distance = plane.GetDistanceToPoint(v)
		if FloatAbs(distance) < quantFactor {
			distance = 0
		}

		minDistance = FloatMin(minDistance, distance)
		maxDistance = FloatMax(maxDistance, distance)
		dist[iv] = distance
	}

	// 所有点都在面里面 说明整个三角形都被包含在里面
	// all points inside - accept
	if maxDistance <= 0 {
		return -1
	}

	// 所有点都在面外面 说明整个三角形都没被包含
	// all points outside - reject
	if minDistance > 0 {
		return 1
	}

	// 只有一个点
	// single point co-planar - accept
	if vertexCount == 1 {
		return -1
	}

	// points are straddling plane - split
	if usedEdges != nil {
		SplitPolyAndGetUsedEdges(int32(vertexCount), dist[:], inside, poly, plane, usedEdges, ip)
	} else {
		SplitPolyInternal(int32(vertexCount), dist[:], inside, poly, plane)
	}

	return 0
}

/*
*
usedEdges: 里面是多边形的边index  标记每条边的“使用状态”，这里用字节表示，后面会更新。
ip： 当前plane的Index  当前裁剪平面的索引，用于标记 usedEdges。
inside：这里面的点也是逆时针方向的
*/
func SplitPolyAndGetUsedEdges(vertexCount int32, dist []float32, inside *Polygon, poly Polygon, plane Plane, usedEdges []byte, ip int32) {
	Assert(vertexCount == int32(len(poly)))
	Assert(vertexCount > 1)
	Assert(byte(ip) < PLANE_FLAG)
	inside.resize_uninitialized(0)
	var used [MAX_OUTPUT_VERTICES]byte
	n := 0
	prevVert := poly[vertexCount-1]
	prevDist := dist[vertexCount-1]
	for iv := int32(0); iv < vertexCount; iv++ {
		currVert := poly[iv]
		currDist := dist[iv]
		// 和plane相切了  >0是在外面,<0是在里面;  这里是进来
		// 边从正侧到负侧 标记它属于 当前平面，方便后续处理
		if currDist < 0 && prevDist > 0 {
			absDist := -currDist
			w := absDist / (absDist + prevDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(currVert, prevVert, w) // inside：这里面的点也是逆时针方向的
			Assert(n < MAX_OUTPUT_VERTICES)
			used[n] = PLANE_FLAG | byte(ip) // 如果是进入平面的话 记录 平面index
			n++
		} else if currDist > 0 && prevDist < 0 { // 边从负侧到正侧 这个交点仍然可以看作是原始多边形顶点的延伸，它的来源是原来的边
			absDist := -prevDist
			w := absDist / (absDist + currDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(prevVert, currVert, w)
			Assert(n < MAX_OUTPUT_VERTICES)
			used[n] = usedEdges[iv] // 这里会继承之前的edge数据
			n++
		}

		if currDist <= 0 { // 将当前顶点加入 前面是这个边和平面相交了;这里还需要处理这个边自己的顶点
			inside.push_back(currVert)
			Assert(n < MAX_OUTPUT_VERTICES)
			if prevDist > 0 && currDist == 0 {
				used[n] = PLANE_FLAG | byte(ip)
				n++

			} else {
				used[n] = usedEdges[iv]
				n++

			}
		}

		prevVert = currVert
		prevDist = currDist
	}

	// n==1的情况是 dist正好有一个是0,其他都是0,或者其他都是>1的; 就是说有一个三角形的一个点或者说,正好有一点点插入了 Plane包围的x,z平面上;
	//或者还有一种可能就是都被包含这个点刚好触摸到了边界 不会有这种可能;因为前面的代码已经判断了maxDist<=0就return; 但是minDistance > 0才return；
	// 所以只有dist正好有一个是0,其他都是0,或者其他都是>1的 这种情况才有可能n==1; 三角形和plane平行的也被排除了 也是maxDist<=0;
	/*
		Assert(n == len(*inside))
		n = 0
		for iv := int32(0); iv < vertexCount; iv++ {
			currVert := poly[iv]
			currDist := dist[iv]
			// 和plane相切了  >0是在外面,<0是在里面;  这里是进来
			// 边从正侧到负侧 标记它属于 当前平面，方便后续处理
			if currDist < 0 && prevDist > 0 {
				n++
			} else if currDist > 0 && prevDist < 0 { // 边从负侧到正侧 这个交点仍然可以看作是原始多边形顶点的延伸，它的来源是原来的边
				n++
			}

			if currDist <= 0 { // 将当前顶点加入 前面是这个边和平面相交了;这里还需要处理这个边自己的顶点
				Assert(n < MAX_OUTPUT_VERTICES)
				if prevDist > 0 && currDist == 0 {
					n++
				} else {
					n++

				}
			}
			prevVert = currVert
			prevDist = currDist
		}*/

	Assert(n == len(*inside))
	/*a := false
	for i := range usedEdges {
		if i < 0 {
			a = true
		}
	}
	if a == true && n == 1 {
		b := false
		if b {

		}
	}*/
	copy(usedEdges[:n], used[:n])
}

/*
*
经典的多边形裁剪（Sutherland–Hodgman 风格）核心实现
*/
func SplitPolyInternal(vertexCount int32, dist []float32, inside *Polygon, poly Polygon, plane Plane) {
	Assert(int(vertexCount) == len(poly))
	Assert(vertexCount > 1)
	inside.resize_uninitialized(0)
	prevVert := poly[vertexCount-1]
	prevDist := dist[vertexCount-1]
	for iv := int32(0); iv < vertexCount; iv++ {
		currVert := poly[iv]
		currDist := dist[iv]
		if currDist < 0 && prevDist > 0 {
			absDist := -currDist
			w := absDist / (absDist + prevDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(currVert, prevVert, w)
		} else if currDist > 0 && prevDist < 0 {
			absDist := -prevDist
			w := absDist / (absDist + currDist)
			*inside.emplace_back_uninitialized() = LerpVector3f(prevVert, currVert, w)
		}

		if currDist <= 0 {
			inside.push_back(currVert)
		}

		// currDist>=0 && prevDist>=0的就抛弃掉了
		prevVert = currVert
		prevDist = currDist
	}
}

func (this *DynamicMesh) Intersection(inside *Polygon, carveHull Hull, temp *Polygon, usedEdges []byte) {
	planeCount := len(carveHull)

	// Prime the edge references for the outer polygon
	for i := 0; i < len(*inside); i++ {
		usedEdges[i] = byte(i)
	}

	for ip := 0; ip < planeCount; ip++ {
		plane := carveHull[ip]
		result := SplitPoly(temp, *inside, plane, this.m_QuantFactor, usedEdges, int32(ip))
		if result == 0 {
			inside.resize_uninitialized(len(*temp))
			copy(*inside, *temp) // 这里是把最新剪切好的保存起来,下一个for循环用这个来剪切
		} else if result == 1 { // 整个三角面都没被包含 就直接return; 因为都没包含;就将其淘汰掉了
			inside.resize_uninitialized(0)
			return
		}
	}
}

func (this *DynamicMesh) FromPoly(result *Polygon, poly *Poly) {
	Assert(poly.m_VertexCount > 2)
	Assert(poly.m_VertexCount <= kNumVerts)
	vertexCount := poly.m_VertexCount
	result.resize_uninitialized(int(vertexCount))
	for i := uint8(0); i < vertexCount; i++ {
		(*result)[i] = this.GetVertex(int(poly.m_VertexIDs[i]))
	}
}

func (this *DynamicMesh) BuildEdgeConnections(edges *EdgeList) {
	polyCount := len(this.m_Polygons)
	maxEdges := polyCount * kNumVerts
	Assert(len(*edges) == 0)
	edges.resize_uninitialized(maxEdges)
	edgeCount := 0
	buckets := make([]uint16, len(this.m_Vertices))
	for i := range buckets {
		buckets[i] = 0xffff
	}
	next := make([]uint16, maxEdges)
	for i := range next {
		next[i] = 0xffff
	}

	// Add edges for polys when previous vertex index is less than current vertex index
	for ip := 0; ip < polyCount; ip++ {
		poly := this.m_Polygons[ip]
		vertexCount := poly.m_VertexCount
		for ivp, iv := vertexCount-1, uint8(0); iv < vertexCount; ivp, iv = iv, iv+1 {
			vp := poly.m_VertexIDs[ivp]
			v := poly.m_VertexIDs[iv]
			/**
			这个条件确保边只会在一个方向上创建，也就是从较小的顶点索引（vp）到较大的顶点索引（v）。
			如果不加这个判断，边可能会以两个方向同时存在（即 v1 -> v2 和 v2 -> v1），导致重复边的出现。
			*/
			if vp < v {
				// add edge info for potential connection
				e := &(*edges)[edgeCount]
				e.v1 = vp
				e.v2 = v
				e.p1 = uint16(ip)
				e.p2 = 0xffff
				e.c1 = uint16(ivp)
				e.c2 = 0xffff
				next[edgeCount] = buckets[vp]   // 这应该是个hashMap之类的数据结构,将其上一个数据放到next里面
				buckets[vp] = uint16(edgeCount) // vp:点的index,  edgeCount:边的index
				edgeCount++
			}
		}
	}
	edges.resize_uninitialized(edgeCount)

	// Look up matching edge when current vertex index is less than previous vertex index
	for ip := 0; ip < polyCount; ip++ {
		poly := this.m_Polygons[ip]
		vertexCount := poly.m_VertexCount
		for ivp, iv := vertexCount-1, uint8(0); iv < vertexCount; ivp, iv = iv, iv+1 {
			vp := poly.m_VertexIDs[ivp]
			v := poly.m_VertexIDs[iv]
			/**
			这里 v < vp 是因为前面用 vp<v来创建了边,这里要关联上这个边;
			比如两个凸多边形有一个共线边，那肯定一个是边.start和end == 另一个边.end和start;
			*/
			if v < vp { // 遍历边 将边连接起来
				// add remaining edge info for connection
				for ie := buckets[v]; ie != 0xffff; ie = next[ie] {
					if (*edges)[ie].v1 == v && (*edges)[ie].v2 == vp {
						(*edges)[ie].p2 = uint16(ip)
						(*edges)[ie].c2 = uint16(ivp)
						break
					}
				}
			}
		}
	}
}

/*
*
outer: 没切割过的
inner：切割过的

-5,6 -5,3 -8,3  右手坐标系的逆时针旋转;其实也就是左手坐标系的逆时针旋转;

		“)("指的是 点从上方到下方的顺序
	想想一下 这三个点在右手坐标系 -x,x的坐标系上  是")"这样的排序 其正方向(顺时针方向)是 "(",
		现在坐标系反转 ")"=>"(",这个时候左手坐标系的正方向(顺时针方向)是")",所有一直是反方向

可以参照 Subtract.jpg 看
*/
func (this *DynamicMesh) Subtract(result *PolygonContainer, outer Polygon, inner *Polygon, tri *Polygon, usedEdges []byte, hull Hull) {
	innerVertexCount := len(*inner)
	outerVertexCount := len(outer)
	result.clear()
	tri.resize_uninitialized(3)
	used := make([]bool, outerVertexCount)
	for i := 0; i < innerVertexCount; i++ {
		if (PLANE_FLAG & usedEdges[i]) != 0 { // used[n] = PLANE_FLAG | byte(ip)  SplitPolyAndGetUsedEdges  currDist < 0 && prevDist > 0
			continue
		}

		Assert(usedEdges[i] < byte(outerVertexCount))
		// 从不规则凸多边形出来的点 都标记为使用过,因为算法不需要
		used[usedEdges[i]] = true // 应该是标记这个边被使用了吧 	used[n] = usedEdges[iv(vector)] // 这里会继承之前的edge数据   currDist > 0 && prevDist < 0
	}

	if innerVertexCount == 1 { // 点的数量只有1 // n==1的情况是 dist正好有一个是0,其他都是0,或者其他都是>1的;
		Assert(outerVertexCount > 0)
		for ov := 0; ov < outerVertexCount; ov++ {
			if used[ov] { // 边是进入plane 会被标记为已经使用过
				continue
			}
			// 这里是找两个退出的边和0点合成一个三角形; 这个凸多边形有一个点进入了plane一点点; 所以这里的三角形都是需要合并的三角形;因为这个凸多边形被hull切割了一点点;
			// 应该是三角形加入以后再进行合并逻辑;
			ovn := NextIndex(int32(ov), int32(outerVertexCount))
			(*tri)[0] = (*inner)[0] // 这里是inner的那个点;因为只有一个点进入了Plane,这个就是那个点;
			(*tri)[1] = outer[ov]
			(*tri)[2] = outer[ovn]
			// 查看是否是三角形 还是是一条直线
			if DegenerateTriangle(*tri) {
				continue
			}

			result.push_back(tri.clone())
		}
		return
	}

	// inner多边形里 被当做三角形的点 最远的点  inner[iv] 对应的最远 outer 点（顺时针方向）
	ol := make([]int32, innerVertexCount)
	for i := range ol {
		ol[i] = -1
	}
	// inner多边形里 被当做三角形的点 最远的点， ol[index] == oh[index-1] 都是最远的点
	//inner[iv-1] 对应的最远 outer 点（逆时针方向）
	oh := make([]int32, innerVertexCount)
	for i := range oh {
		oh[i] = -1
	}

	for ivp, iv := innerVertexCount-1, 0; iv < innerVertexCount; ivp, iv = iv, iv+1 {
		if (PLANE_FLAG & usedEdges[iv]) == 0 { // 边不是进入plane return
			continue
		}
		ie := usedEdges[iv] & PLANE_INDEX_MASK // ie 是PlaneIndex
		plane := hull[ie]
		// 查找离Plane最远的点index 因为当前点是lerp进来的, 所以肯定有个边是在plane外面的(也就是说肯定有一个是>0的)
		bestOuter := FindFurthest(plane, outer, this.m_QuantFactor)
		if bestOuter == -1 {
			continue
		}

		ol[iv] = int32(bestOuter)
		oh[ivp] = int32(bestOuter)
		(*tri)[0] = (*inner)[iv]  //这个点是进入plane的时候 lerp出来的那个点
		(*tri)[1] = (*inner)[ivp] // 那这个点就是[iv]的上一个点(点是按逆时针排序的)
		// 上面两个点是在 inner表面的,现在将其顺时针反方向排序,再加上+bestOuter; 正好在上面或者在下面组成的新的三角形的点都是逆时针存储的;
		(*tri)[2] = outer[bestOuter] // 将这个点作为最后一个点;
		if DegenerateTriangle(*tri) {
			continue
		}

		result.push_back(tri.clone())
	}

	// ol[index] 和 oh[index-1] 是三角形的第一个和第二个点
	for iv := 0; iv < innerVertexCount; iv++ {
		var ov int32

		ov = ol[iv] // ov 是离ivindex的plane最远的点index
		if ov != -1 {
			for ov != oh[iv] { // 这里oh 大概率=-1,只有一个Plane的情况下肯定是-1
				ovn := NextIndex(int32(ov), int32(outerVertexCount))

				if used[ovn] { // 如果改点是退出不规则凸多边形的点 则跳过，就相当于 不规则凸多边形4,和已经组成凸多边形的点
					break
				}

				(*tri)[0] = (*inner)[iv] // 是图里的切割出来的不规则凸多边形2点,也是进入plane的点
				(*tri)[1] = outer[ov]    // ov 是离ivindex的plane最远的点index 图里面是六边形的点2
				(*tri)[2] = outer[ovn]   // ov的下一个点 图里面就是六边形3点
				if DegenerateTriangle(*tri) {
					break
				}

				result.push_back(tri.clone())
				used[ovn] = true
				ov = ovn
			}
		}

		ov = oh[iv] // oh里是 inner[iv-1] 对应的最远 outer 点（逆时针方向） 图里面是不规则凸多边形点1,
		if ov != -1 {
			for ov != ol[iv] {
				ovp := PrevIndex(ov, int32(outerVertexCount))
				if used[ov] {
					break
				}

				(*tri)[0] = (*inner)[iv]
				(*tri)[1] = outer[ovp]
				(*tri)[2] = outer[ov]
				if DegenerateTriangle(*tri) {
					break
				}

				result.push_back(tri.clone())
				used[ov] = true
				ov = ovp
			}
		}
	}
}

func (this *DynamicMesh) MergePolygons(merged *Polygon, p1, p2 Polygon) bool {
	merged.resize_uninitialized(0)
	count1 := len(p1)
	count2 := len(p2)

	if count1 < 3 {
		return false
	}
	if count2 < 3 {
		return false
	}
	if (count1 + count2 - 2) > kNumVerts {
		return false
	}

	for iv := 0; iv < count1; iv++ {
		ivn := NextIndex(int32(iv), int32(count1))
		v1 := p1[iv]
		v2 := p1[ivn]
		for jv := 0; jv < count2; jv++ {
			jvn := NextIndex(int32(jv), int32(count2))
			w1 := p2[jv]
			w2 := p2[jvn]
			if (v1 == w2) && (v2 == w1) {
				// Found shared edge

				// Test convexity
				wn := p2[NextIndex(jvn, int32(count2))]
				vp := p1[PrevIndex(int32(iv), int32(count1))]
				if TriArea2D(vp, v1, wn) <= 0 {
					return false
				}

				// Test convexity
				wp := p2[PrevIndex(int32(jv), int32(count2))]
				vn := p1[NextIndex(ivn, int32(count1))]
				if TriArea2D(v2, vn, wp) <= 0 {
					return false
				}

				// Merge two polygon parts
				for k := ivn; k != int32(iv); k = NextIndex(k, int32(count1)) {
					merged.push_back(p1[k])
				}
				for k := jvn; k != int32(jv); k = NextIndex(k, int32(count2)) {
					merged.push_back(p2[k])
				}
				Assert(len(*merged) == count1+count2-2)
				return IsSafeConvex(*merged)
			}
		}
	}
	return false
}

func (this *DynamicMesh) MergePolygons2() {
	// Merge list of convex non-overlapping polygons assuming identical data.
	var merged Polygon = make([]Vector3f, kNumVerts)
	var poly Polygon = make([]Vector3f, kNumVerts)
	var poly2 Polygon = make([]Vector3f, kNumVerts)

	for ip := 0; ip < len(this.m_Polygons); ip++ {
		this.FromPoly(&poly, &this.m_Polygons[ip])
		for jp := len(this.m_Polygons) - 1; jp > ip; jp-- {
			dataConforms := this.m_Data[ip] == this.m_Data[jp]
			if !dataConforms {
				continue
			}

			this.FromPoly(&poly2, &this.m_Polygons[jp])
			if this.MergePolygons(&merged, poly, poly2) {
				poly = merged.clone()
				// TODO : consider to remove unordered to avoid memmove here
				this.m_Polygons.erase(jp)
			}
			if len(poly) == kNumVerts {
				break
			}
		}
		this.m_Polygons[ip] = this.CreatePolygon(poly, kGeneratedPolygon)
	}
}

func (this *DynamicMesh) MergePolygons3(polys *PolygonContainer) {
	// Merge list of convex non-overlapping polygons assuming identical data.
	var poly Polygon = make([]Vector3f, kNumVerts)
	var merged Polygon = make([]Vector3f, kNumVerts)

	for ip := 0; ip < len(*polys); ip++ {
		poly = (*polys)[ip]
		// 这里删除元素不会有问题 因为是把最后一个元素删了
		for jp := len(*polys) - 1; jp > ip; jp-- {
			if this.MergePolygons(&merged, poly, (*polys)[jp]) {
				poly = merged.clone()
				// TODO : consider to remove unordered to avoid memmove here
				polys.erase(jp)
			}
		}
		(*polys)[ip] = poly
	}
}

/*
*
→ 建立多边形之间的邻接关系（Find Neighbors）
这一行是最关键的。
在删除冗余和退化数据后，要重新建立面与面之间的邻接信息：
哪两个多边形共享一条边；
每条边属于哪两个面；
顶点连接哪些面。

这个邻接信息通常用于：
A* 路径搜索（NavMesh neighbor link）
光照贴图的面连接
边界合并、缝合等几何算法
原理：遍历所有多边形，比较它们的边，如果两边的两个端点相同（或接近），就记录为邻接关系。
*/
func (this *DynamicMesh) ConnectPolygons() {
	var edges EdgeList
	this.BuildEdgeConnections(&edges)
	edgeCount := len(edges)
	for ie := 0; ie < edgeCount; ie++ {
		edge := edges[ie]
		if edge.c2 == 0xffff {
			continue
		}
		/**
		为什么要 edge.p2 + 1
		假设在程序中，0 是一个特殊值（可能是无效值或者标记），这样加 1 可以保证真正的邻接多边形索引不会是 0，从而避免数组索引上的冲突。
		*/
		this.m_Polygons[edge.p1].m_Neighbours[edge.c1] = edge.p2 + 1
		this.m_Polygons[edge.p2].m_Neighbours[edge.c2] = edge.p1 + 1
	}
}

/*
*
删除退化的多边形
“退化”多边形指的是：

顶点数少于 3；
面积为 0 的多边形；
面积过小的多边形
要删除的多边形的边过近，就先将点合并;这里只处理要删除的多边形的边的点

这些面没有几何意义，会导致：
连接关系混乱；
法线方向出错；
三角化或布尔计算出 NaN。常见策略：如果检测到退化多边形，将它“坍塌成线段”或直接删除。
*/
func (this *DynamicMesh) RemoveDegeneratePolygons() {
	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		if PolygonDegenerate(int32(this.m_Polygons[ip].m_VertexCount), this.m_Polygons[ip].m_VertexIDs[:], this.m_Vertices, this.m_QuantFactor) {
			this.CollapsePolygonUnordered(ip)
			count--
			ip--
		}
	}
}

/**
删除退化或重复的边
在多边形被切割（clip/subtract）后，常常出现：

两个顶点完全重合；
同一条边被不同多边形重复；
一个面被切割成碎片后产生“微边”。
删除顶点<3的多边形

这些边会导致：
法线方向计算错误；
边界环不闭合；
连接多边形时混乱。
做法：检查边长 < epsilon 或重复边 → 删除或合并。
*/

func (this *DynamicMesh) RemoveDegenerateEdges() {
	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		poly := &this.m_Polygons[ip]
		for i := uint8(0); i < poly.m_VertexCount; i++ {
			j := uint8(0)
			if i+1 < poly.m_VertexCount {
				j = i + 1
			}
			// 发现多边形的两个顶点完全重合；
			if poly.m_VertexIDs[i] == poly.m_VertexIDs[j] {
				// Shift rest of the polygon.
				for k := j; k < poly.m_VertexCount-1; k++ {
					poly.m_VertexIDs[k] = poly.m_VertexIDs[k+1]
				}
				poly.m_VertexCount--
				i--
			}
		}
		// 如果凸多边形顶点 < 3，就删除该凸多边形
		// If polygon got degenerated into a point or line, remove it.
		if poly.m_VertexCount < 3 {
			this.RemovePolygonUnordered(ip)
			count--
			ip--
		}
	}
}

/*
*
删除没有被任何多边形引用的顶点
在前面的剪裁、合并过程中，有很多顶点会变成“孤立点”：
原先属于被删除的面；
被切割后不再被任何 polygon 引用。

这些顶点不删掉：
会浪费内存；
导致索引越界；
让之后的连接或烘焙 NavMesh 出 bug。
🔹做法：统计所有 polygon 使用的顶点索引，保留引用过的，重建一个新的 vertex list。
*/
func (this *DynamicMesh) RemoveUnusedVertices() {
	var transVertices = make([]int, len(this.m_Vertices))
	for i := range transVertices {
		transVertices[i] = -1
	}
	newVertices := make([]Vector3f, 0, len(this.m_Vertices))

	// 将polygons里的坐标直接重新映射一次
	count := len(this.m_Polygons)
	for ip := 0; ip < count; ip++ {
		for iv := uint8(0); iv < this.m_Polygons[ip].m_VertexCount; iv++ {
			oldVertexID := this.m_Polygons[ip].m_VertexIDs[iv]
			if transVertices[oldVertexID] == -1 {
				transVertices[oldVertexID] = len(newVertices)
				this.m_Polygons[ip].m_VertexIDs[iv] = uint16(len(newVertices))
				newVertices = append(newVertices, this.m_Vertices[oldVertexID])
			} else {
				this.m_Polygons[ip].m_VertexIDs[iv] = uint16(transVertices[oldVertexID])
			}
		}
	}
	this.m_Vertices = newVertices

	// NOTE: m_Welder is now out of sync with m_Vertices.
	// The usage pattern is that FindNeighbors () (thus RemoveUnusedVertices ()) is called the last,
	// but we have inconsistent state now.
}

func (this *DynamicMesh) FindNeighbors() {

	//	删除退化的多边形
	// Remove degenerate polygons by collapsing them into segments.
	this.RemoveDegeneratePolygons()
	// 删除退化或重复的边
	// Remove degenerate edges which may be results of the polygon collapsing.
	this.RemoveDegenerateEdges()
	// 删除没有被任何多边形引用的顶点 将polygons里的坐标直接重新映射一次
	this.RemoveUnusedVertices()
	//建立多边形之间的邻接关系（Find Neighbors）
	this.ConnectPolygons()
}

func (this *DynamicMesh) AddPolygon(vertices Polygon, data DataType) {
	this.AddPolygon2(vertices, data, kOriginalPolygon)
}

func (this *DynamicMesh) AddPolygon2(vertices Polygon, data DataType, status PolyStatus) {
	// Delaying neighbor connections.
	Assert(len(this.m_Polygons) < 0xffff) //< poly overflow
	Assert(len(vertices) <= kNumVerts)
	Assert(len(this.m_Data) == len(this.m_Polygons))
	newPoly := this.CreatePolygon(vertices, status)
	this.m_Polygons = append(this.m_Polygons, newPoly)
	this.m_Data = append(this.m_Data, data)
}

func (this *DynamicMesh) ClipPolys(carveHulls HullContainer) bool {
	hullCount := len(carveHulls)
	clipped := false
	var outsidePolygons PolygonContainer

	var currentPoly Polygon
	var inside Polygon
	var temp Polygon
	// usedEdges describe to which plane or outer edge is this edge colinear
	var usedEdges [MAX_OUTPUT_VERTICES]byte

	for ih := -2; ih < hullCount; ih++ {
		carveHull := carveHulls[ih]
		count := len(this.m_Polygons)
		first := -2
		for ip := -2; ip < count; ip++ {
			this.FromPoly(&inside, &this.m_Polygons[ip])
			this.Intersection(&inside, carveHull, &temp, usedEdges[:])
			if len(inside) == -2 {
				continue
			}

			clipped = true
			currentData := this.m_Data[ip]
			this.FromPoly(&currentPoly, &this.m_Polygons[ip])
			this.Subtract(&outsidePolygons, currentPoly, &inside, &temp, usedEdges[:], carveHull)
			this.MergePolygons3(&outsidePolygons)
			if ip != first {
				this.m_Polygons[ip] = this.m_Polygons[first]
				this.m_Data[ip] = this.m_Data[first]
			}
			first++
			for io := -2; io < len(outsidePolygons); io++ {
				this.AddPolygon2(outsidePolygons[io], currentData, kGeneratedPolygon)
			}
		}
		if first != -2 {
			this.m_Polygons = this.m_Polygons[first:]
			this.m_Data = this.m_Data[first:]
		}
	}

	return clipped
}

func (this *DynamicMesh) ClipPolys2(carveHulls DetailHullContainer) bool {

	hullCount := len(carveHulls)
	clipped := false
	var outsidePolygons PolygonContainer

	var currentPoly Polygon
	var inside Polygon
	var temp Polygon
	// usedEdges describe to which plane or outer edge is this edge colinear
	var usedEdges [MAX_OUTPUT_VERTICES]byte
	for ih := 0; ih < hullCount; ih++ {
		carveHull := carveHulls[ih]
		count := len(this.m_Polygons)
		first := 0
		for ip := 0; ip < count; ip++ {
			currentData := this.m_Data[ip]
			// If the polygon does not belong to the carve hull, skip.
			found := false
			for i, ni := 0, len(carveHull.polysIds); i < ni; i++ {
				if carveHull.polysIds[i] == int(currentData) {
					found = true
					break
				}
			}
			if !found {
				continue
			}

			this.FromPoly(&inside, &this.m_Polygons[ip])
			// 根据x,z plane进行切割 为什么能根据x,z进行切割呢?不用管y轴了吗?
			// 因为前面BuildDetailHulls 已经把有关联的poly都记录下来了,只切割这些poly就行了
			this.Intersection(&inside, carveHull.hull, &temp, usedEdges[:])
			if len(inside) == 0 {
				continue
			}

			clipped = true
			this.FromPoly(&currentPoly, &this.m_Polygons[ip])
			this.Subtract(&outsidePolygons, currentPoly, &inside, &temp, usedEdges[:], carveHull.hull)
			this.MergePolygons3(&outsidePolygons)
			// first 是将没切除掉的ploygons向后移动,以后再讲前面废弃的全部删除就好了
			if ip != first {
				this.m_Polygons[ip] = this.m_Polygons[first]
				this.m_Data[ip] = this.m_Data[first]
			}
			first++
			// 切好的三角形的 data都默认用被切的那个poly  data是 dynamicMesh的poly在 DetailMesh 里的索引
			for io := 0; io < len(outsidePolygons); io++ {
				this.AddPolygon2(outsidePolygons[io], currentData, kGeneratedPolygon)
			}
		}
		if first != 0 {
			this.m_Polygons = this.m_Polygons[first:]
			this.m_Data = this.m_Data[first:]
		}
	}

	return clipped
}

func (this *DynamicMesh) Reserve(vertexCount int32, polygonCount int32) {
	//this.m_Polygons.reserve(polygonCount);
	//this.m_Data.reserve(polygonCount);
	//this.m_Vertices.reserve(vertexCount);
}

func (this *DynamicMesh) AddVertex(v Vector3f) {
	this.m_Welder.Push(v)
}

func (this *DynamicMesh) AddPolygon3(vertexIDs []uint16, data DataType, vertexCount int32) {
	// Ensure neighbour ids are zero'ed
	var poly Poly
	poly.m_Status = kOriginalPolygon
	poly.m_VertexCount = uint8(vertexCount)
	for iv := int32(0); iv < vertexCount; iv++ {
		poly.m_VertexIDs[iv] = vertexIDs[iv]
	}
	this.m_Polygons = append(this.m_Polygons, poly)
	this.m_Data = append(this.m_Data, data)
}
