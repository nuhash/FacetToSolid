#pragma once

#include "FeatureExtraction.h"
namespace SurfaceReconstructionAlgo
{

	class ReconstructedSurface
	{
	public:
	protected:
	private:
	};

	class ReconstructedObject : public vector<ReconstructedSurface>
	{
	public:
	protected:
	private:

	};

	ReconstructedObject EdgeFitNURBS(FeatureExtractionAlgo::ExtractedFeatures features);

};