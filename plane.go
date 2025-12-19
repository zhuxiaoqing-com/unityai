package unityai

// n dot x + d=0 n法向量  x空间中任意一点  d平面到原点的距离，注意符号
type Plane struct {
	normal   Vector3f
	distance float32
}

// 构建出来的平面是 一组plane;指向外面的;不是指向多边形里面的；
func (p *Plane) SetNormalAndPosition(normal Vector3f, position Vector3f) {
	p.normal = normal
	// 就是 ndotx = -d; d = -ndotx;
	p.distance = -DotVector3f(normal, position)
}

/*
*
n dot x + d=0  是x点距离平面的距离是0;
将x替换成指定的坐标点得出的结果就是距离平面的距离; >0 是在法向量方向, <0是在法向量相反方向;
*/
func (this *Plane) GetDistanceToPoint(inPt Vector3f) float32 {
	return DotVector3f(this.normal, inPt) + this.distance
}

func NewPlane(normal, position Vector3f) Plane {
	plane := Plane{}
	plane.SetNormalAndPosition(normal, position)
	return plane
}
