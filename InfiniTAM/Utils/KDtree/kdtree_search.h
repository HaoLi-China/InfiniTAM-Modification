
#ifndef __KDTREE_SPATIAL_SEARCH__
#define __KDTREE_SPATIAL_SEARCH__

#include <vector>
#include "../../ITMLib/Utils//ITMMath.h"

/***********************************************************************
 I have collected three different implementations: ANN, ETH, and FLANN,
 and tried to get the best performance of each implementation. Timing 
 tests (on Windows, using a Visual C++ release build) indicated the ETH 
 implementation has the best performance. Bellow is the statistic
 summary of the test on two point clouds.

 Build:  time for constructing the kd-tree.
 Single: time for querying closest vertex (for each point in the point cloud).
 KNN:    time for querying K(=16) closest vertex.
 Radius: time for querying closest vertex within a radius.
 
 Test 1: 362, 271 points (the Stanford bunny). squared_radius = 0.001 * 0.001
 --------------------------------------------------------------------------------------
       Build		 |		Single		   |		KNN			 |		Radius
 --------------------|---------------------|---------------------|---------------------
 ANN    ETH   FLANN  |  ANN   ETH   FLANN  |  ANN	 ETH   FLANN |  ANN	  ETH   FLANN  
 --------------------|---------------------|-------------------- |---------------------
 0.14   0.05   0.12  |  0.17  0.11   0.71  |  1.33   1.0   1.48  |  1.32  1.01  1.51
 --------------------------------------------------------------------------------------
 
 Test 2: 4, 116, 466 points (an noisy urban scan). squared_radius = 0.03 * 0.03
 --------------------------------------------------------------------------------------
       Build		 |		Single		   |		KNN			 |		Radius
 --------------------|---------------------|---------------------|---------------------
 ANN    ETH   FLANN  |  ANN   ETH   FLANN  |  ANN	 ETH   FLANN |  ANN	  ETH   FLANN  
 --------------------|---------------------|-------------------- |---------------------
 2.2   0.76   1.88   |  2.61  1.84  11.8   |  20.8   13.5  22.0  |  8.75  4.79  15.1
 --------------------------------------------------------------------------------------

************************************************************************/

class KdTreeSearch
{
public:
	KdTreeSearch();
	virtual ~KdTreeSearch();

	//______________ tree construction __________________________

	virtual void begin() = 0;
	virtual void add_point(Vector3f& v) = 0;
	virtual void add_vertex_set(Vector3f* vs, int points_num) = 0;
	virtual void end() = 0;

	//________________ closest point ____________________________

	// NOTE: *squared* distance is returned
	//virtual PointSet::Vertex* find_closest_point(const Vector3f& p, double& squared_distance) const = 0;
	//virtual PointSet::Vertex* find_closest_point(const Vector3f& p) const = 0;

	//_________________ K-nearest neighbors ____________________

	// NOTE: *squared* distances are returned
	virtual void find_closest_K_points(const Vector3f& p, unsigned int k, std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances) const = 0;
	virtual void find_closest_K_points(const Vector3f& p, unsigned int k, std::vector<Vector3f>& neighbors) const = 0;

	//___________________ radius search __________________________

	// fixed-radius kNN	search. Search for all points in the range.
	// NOTE: *squared* radius of query ball
	virtual void find_points_in_radius(const Vector3f& p, double squared_radius, std::vector<Vector3f>& neighbors) const = 0;
	virtual void find_points_in_radius(const Vector3f& p, double squared_radius, std::vector<unsigned int>& neighbor_indices) const = 0;

	virtual void find_points_in_radius(const Vector3f& p, double squared_radius, std::vector<Vector3f>& neighbors, std::vector<double>& squared_distances) const = 0;
};
#endif


