#ifndef __KDTREE_KDTREE_SEARCH_ETH__
#define __KDTREE_KDTREE_SEARCH_ETH__

#include "kdtree_search.h"

class KdTreeSearch_ETH : public KdTreeSearch  {
public:
	KdTreeSearch_ETH();
	~KdTreeSearch_ETH();

	//______________ tree construction __________________________

	void begin() ;
	void add_point(Vector3f& v);
	void add_vertex_set(Vector3f* vs, int points_num);
	void end() ;

	//________________ closest point ____________________________

	// NOTE: *squared* distance is returned
	//virtual PointSet::Vertex* find_closest_point(const vec3& p, double& squared_distance) const;
	//virtual PointSet::Vertex* find_closest_point(const vec3& p) const;

	//_________________ K-nearest neighbors ____________________

	// NOTE: *squared* distances are returned
	void find_closest_K_points(
		const Vector3f& p, unsigned int k, std::vector<unsigned int>& neighbor_indices
		) const;
	void find_closest_K_points(
		const Vector3f& p, unsigned int k,
		std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances
		) const ;

	void find_closest_K_points(
		const Vector3f& p, unsigned int k,
		std::vector<Vector3f>& neighbors
		) const ;

	//___________________ radius search __________________________

	// fixed-radius kNN	search. Search for all points in the range.
	// NOTE: *squared* radius of query ball
	void find_points_in_radius(const Vector3f& p, double squared_radius,
		std::vector<Vector3f>& neighbors
		) const ;

	void find_points_in_radius(const Vector3f& p, double squared_radius,
		std::vector<unsigned int>& neighbor_indices
		) const;

	void find_points_in_radius(const Vector3f& p, double squared_radius,
		std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances
		) const ;

	//____________________ cylinder range search _________________

	// Search for the nearest points whose distances to line segment $v1$-$v2$ are smaller 
	// than $radius$. If $bToLine$ is true, the points found are ordered by their distances 
	// to the line segment. Otherwise, they are ordered by their distances to $v1$.
	// NOTE: it is radius (instead of *squared* radius).
	unsigned int find_points_in_cylinder(
		const Vector3f& p1, const Vector3f& p2, double radius,
		std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances,
		bool bToLine = true
		) const ;

	unsigned int find_points_in_cylinder(
		const Vector3f& p1, const Vector3f& p2, double radius,
		std::vector<Vector3f>& neighbors,
		bool bToLine = true
		) const ;
	
	//_______________________ cone range search __________________

	// Search for the nearest points $P_i$ with an cone from $v1$ to $v2$ defined by v1 and v2. 
	// As a result, the angle between $v1$$P_i$ and $v1$$v2$ is smaller than $angle_range$.
	// Search for the nearest points P_i where the angle between $v1$-P_i and $v1$-$v2$ is 
	// smaller than $angle$.
	// NOTE: angle is in radian.
	unsigned int find_points_in_cone(
		const Vector3f& eye, const Vector3f& p1, const Vector3f& p2, double angle_range,
		std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances,
		bool bToLine = true
		) const ;

	unsigned int find_points_in_cone(
		const Vector3f& eye, const Vector3f& p1, const Vector3f& p2, double angle_range,
		std::vector<Vector3f>& neighbors,
		bool bToLine = true
		) const ;

protected:
	std::vector<Vector3f> vertices_;
	unsigned int	points_num_;

	void*	tree_;
} ;

#endif


