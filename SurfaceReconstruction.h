#pragma once

#include "FeatureExtraction.h"
class SurfaceReconstructionAlgo
{
public:
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

	static ReconstructedObject EdgeFitNURBS(FeatureExtractionAlgo::ExtractedFeatures features);
protected:
private:
};