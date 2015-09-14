#ifndef POINTSIO_H
#define POINTSIO_H

#include <stdio.h>
#include <iostream>
#include <vector>

#include "rply.h"
#include "../../ITMLib/Utils/ITMMath.h"

class PointsIO
{
public:
	PointsIO();
	~PointsIO();

	static bool savePLYfile(const std::string& filename, const std::vector<Vector3f>& points, const std::vector<Vector3f>& normals, const std::vector<Vector3u>& colors);
	static bool savePLYfile(const std::string& filename, const std::vector<Vector3f>& points, const std::vector<Vector3f>& normals, const Vector3u &color);
};

#endif // POINTSIO_H
