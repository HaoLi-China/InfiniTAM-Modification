#include <iostream>
#include "kdtree_search_eth.h"
#include "ETH_Kd_Tree/kdTree.h"

#define get_tree(x) ((kdtree::KdTree*)(x))


KdTreeSearch_ETH::KdTreeSearch_ETH()  {
	points_num_ = 0;
	tree_ = 0;
}


KdTreeSearch_ETH::~KdTreeSearch_ETH() {
    delete get_tree(tree_);
}


void KdTreeSearch_ETH::begin()  {
	vertices_.clear();

    delete get_tree(tree_);
	tree_ = 0;
}


void KdTreeSearch_ETH::end()  {
	points_num_ = vertices_.size();

	kdtree::Vector3D* points = new kdtree::Vector3D[points_num_];
	for(unsigned int i=0; i<points_num_; ++i) {
		const Vector3f& p = vertices_[i];
		points[i].x = p.x;
		points[i].y = p.y;
		points[i].z = p.z;
	}

	unsigned int maxBucketSize = 16 ;	// number of points per bucket
	tree_ = new kdtree::KdTree(points, points_num_, maxBucketSize );
	delete [] points;
}


void KdTreeSearch_ETH::add_point(Vector3f& v)  {
	vertices_.push_back(v);
}

void KdTreeSearch_ETH::add_vertex_set(Vector3f* vs, int points_num)  {
	for (int i = 0; i < points_num; i++){
		vertices_.push_back(vs[i]);
	}
}

//PointSet::Vertex* KdTreeSearch_ETH::find_closest_point(const vec3& p) const {
//	kdtree::Vector3D v3d( p.x, p.y, p.z );
//	get_tree(tree_)->setNOfNeighbours( 1 );
//	get_tree(tree_)->queryPosition( v3d );
//
//	unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
//	if (num == 1) {
//		return vertices_[ get_tree(tree_)->getNeighbourPositionIndex(0) ];
//	} else
//		return nil;
//}

//PointSet::Vertex* KdTreeSearch_ETH::find_closest_point(const vec3& p, double& squared_distance) const {
//	kdtree::Vector3D v3d( p.x, p.y, p.z );
//	get_tree(tree_)->setNOfNeighbours( 1 );
//	get_tree(tree_)->queryPosition( v3d );
//
//	unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
//	if (num == 1) {
//		squared_distance = get_tree(tree_)->getSquaredDistance(0);
//		return vertices_[ get_tree(tree_)->getNeighbourPositionIndex(0) ];
//	} else {
//		std::cerr << "no point found" << std::endl;
//		return nil;
//	}
//}
//
void KdTreeSearch_ETH::find_closest_K_points(
	const Vector3f& p, unsigned int k, std::vector<Vector3f>& neighbors
	)  const {
		kdtree::Vector3D v3d( p.x, p.y, p.z );
		get_tree(tree_)->setNOfNeighbours( k );
		get_tree(tree_)->queryPosition( v3d );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		if (num == k) {
			neighbors.resize(k);
			for (unsigned int i=0; i<k; ++i) {
				neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
			}		
		} else
			std::cerr << "less than " << k << " points found" << std::endl;
}

void KdTreeSearch_ETH::find_closest_K_points(
	const Vector3f& p, unsigned int k, std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances
	)  const {
		kdtree::Vector3D v3d( p.x, p.y, p.z );
		get_tree(tree_)->setNOfNeighbours( k );
		get_tree(tree_)->queryPosition( v3d );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		if (num == k) {
			neighbors.resize(k);
			squared_distances.resize(k);
			for (unsigned int i=0; i<k; ++i) {
				neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
				squared_distances[i] = get_tree(tree_)->getSquaredDistance(i);
			}		
		} else
			std::cerr << "less than " << k << " points found" << std::endl;
}



void KdTreeSearch_ETH::find_points_in_radius(
	const Vector3f& p, double squared_radius, std::vector<Vector3f>& neighbors
	)  const {
		kdtree::Vector3D v3d( p.x, p.y, p.z );
		get_tree(tree_)->queryRange( v3d, squared_radius, true );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		neighbors.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
		}	
}


void KdTreeSearch_ETH::find_points_in_radius(
	const Vector3f& p, double squared_radius, std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances
	)  const {
		kdtree::Vector3D v3d( p.x, p.y, p.z );
		get_tree(tree_)->queryRange( v3d, squared_radius, true );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		neighbors.resize(num);
		squared_distances.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
			squared_distances[i] = get_tree(tree_)->getSquaredDistance(i);
		}	
}


unsigned int KdTreeSearch_ETH::find_points_in_cylinder(
	const Vector3f& p1, const Vector3f& p2, double radius,
	std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances,
	bool bToLine
	) const {
		kdtree::Vector3D s( p1.x, p1.y, p1.z );
		kdtree::Vector3D t( p2.x, p2.y, p2.z );
		get_tree(tree_)->queryLineIntersection( s, t, radius, bToLine, true );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();

		neighbors.resize(num);
		squared_distances.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
			squared_distances[i] = get_tree(tree_)->getSquaredDistance(i);
		}	

		return num;
}

unsigned int KdTreeSearch_ETH::find_points_in_cylinder(
	const Vector3f& p1, const Vector3f& p2, double radius,
	std::vector<Vector3f>& neighbors,
	bool bToLine
	) const {
		kdtree::Vector3D s( p1.x, p1.y, p1.z );
		kdtree::Vector3D t( p2.x, p2.y, p2.z );
		get_tree(tree_)->queryLineIntersection( s, t, radius, bToLine, true );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		neighbors.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
		}

		return num;
}


unsigned int KdTreeSearch_ETH::find_points_in_cone(
	const Vector3f& eye, const Vector3f& p1, const Vector3f& p2, double angle_range,
	std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances,
	bool bToLine
	) const {
		kdtree::Vector3D eye3d( eye.x, eye.y, eye.z );
		kdtree::Vector3D s( p1.x, p1.y, p1.z );
		kdtree::Vector3D t( p2.x, p2.y, p2.z ); 
		get_tree(tree_)->queryConeIntersection( eye3d, s, t, angle_range, bToLine, true );

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		neighbors.resize(num);
		squared_distances.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
			squared_distances[i] = get_tree(tree_)->getSquaredDistance(i);
		}

		return num;
}

unsigned int KdTreeSearch_ETH::find_points_in_cone(
	const Vector3f& eye, const Vector3f& p1, const Vector3f& p2, double angle_range,
	std::vector<Vector3f>& neighbors,
	bool bToLine
	) const {
		kdtree::Vector3D eye3d( eye.x, eye.y, eye.z );
		kdtree::Vector3D s( p1.x, p1.y, p1.z );
		kdtree::Vector3D t( p2.x, p2.y, p2.z );
		get_tree(tree_)->queryConeIntersection( eye3d, s, t, angle_range, bToLine, true);

		unsigned int num = get_tree(tree_)->getNOfFoundNeighbours();
		neighbors.resize(num);
		for (unsigned int i=0; i<num; ++i) {
			neighbors[i] = vertices_[ get_tree(tree_)->getNeighbourPositionIndex(i) ];
		}

		return num;
}
