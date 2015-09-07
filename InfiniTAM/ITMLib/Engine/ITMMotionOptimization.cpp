#include "ITMMotionAnalysis.h"
#include "../../../3rd_libLBFGS/lbfgs.h"

using namespace ITMLib::Engine;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motions information
struct MotionsData
{
	MotionsData(ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage);
	void updatePointsNormals(const std::vector<double>& x);  // using new warp transformations to compute the points and normals of canonical model
	void computeDpoints();  // compute corresponding points of depth image

	std::vector<Vector3f> points; // canonical model points
	std::vector<Vector3f> normals; // canonical model normals
	std::vector<Vector3f> dpoints; // depth image points
	std::vector<unsigned int> corriIndices; // pairs of canonical model points and depth image points
	std::vector<double> x0; // 6 * n warp transformations, n represents all nodes
	std::vector<bool> visibles; //n, visible nodes
	std::vector<std::vector<unsigned int>> neighborhood;  // n, neighbors of each node
	double lambda;
};

MotionsData::MotionsData(ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage)
{
	// need to fill
}

void MotionsData::updatePointsNormals(const std::vector<double>& x)
{
	// need to fill
}

void MotionsData::computeDpoints()
{
	// need to fill
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// non-linear optimization functions
static lbfgsfloatval_t motions_evaluate(
	void *data,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int N,
	const lbfgsfloatval_t step
	)
{
	MotionsData* d = (MotionsData*)data;
	const std::vector<Vector3f>& points = d->points;
	const std::vector<Vector3f>& normals = d->normals;
	const std::vector<Vector3f>& dpoints = d->dpoints;
	const std::vector<unsigned int>& corriIndices = d->corriIndices;
	const std::vector<double>& x0 = d->x0;
	const std::vector<bool>& visibles = d->visibles;
	const std::vector<std::vector<unsigned int>>& neighborhood = d->neighborhood;

	//////////////////////////////////////////////////////////////////////////
	// initialize
	lbfgsfloatval_t f = 0;
	for (int i = 0; i < N; ++i) {
		g[i] = 0;
	}

	// data term


	// reg term
	

	return f;
}

static int motions_progress(
	void *data,
	const lbfgsfloatval_t *x,
	const lbfgsfloatval_t *g,
	const lbfgsfloatval_t fx,
	const lbfgsfloatval_t xnorm,
	const lbfgsfloatval_t gnorm,
	const lbfgsfloatval_t step,
	int n,
	int k,
	int ls
	)
{
	//printf("Iteration %d\n", k);
	printf("Iteration %d:  fx = %f,  xnorm = %f, gnorm = %f, step = %f\n", k, fx, xnorm, gnorm, step);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// main optimization function
void ITMMotionAnalysis::optimizeEnergyFunction(ITMPointCloud &visiblePointClound, ITMFloatImage *newDepthImage)
{
	MotionsData data(visiblePointClound, newDepthImage);
	data.lambda = 1.0;

	int n = data.x0.size();
	lbfgsfloatval_t *x = lbfgs_malloc(n);
	if (!x) {
		std::cout << "L-BFGS failed to allocate a memory block for variables." << std::endl;
		return;
	}
	/* Initialize the variables. */
	for (int i = 0; i < n; ++i) {
		x[i] = data.x0[i];
	}

	lbfgs_parameter_t param;
	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	//----------------------------------------------------------------------------

	// Start the L-BFGS optimization; this will invoke the callback functions
	// evaluate() and progress() when necessary.
	lbfgsfloatval_t fx;
	int ret = lbfgs(n, x, &fx, motions_evaluate, motions_progress, &data, &param);

	/* Report the result. */
	printf("L-BFGS optimization terminated with status code = %d\n", ret);
	printf("  fx = %f\n", fx);

	// assign new warp transformation from x
	/*
	
	need to fill!!!!!!!!!!!!!!!!!!!!!!
	
	*/
}