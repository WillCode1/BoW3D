#include "Frame.h"
#include <thread>

namespace BoW3D
{
    long unsigned int Frame::nNextId = 0;

    Frame::Frame(std::shared_ptr<LinK3D_Extractor> pLink3dExtractor, pcl::PointCloud<PointType>::Ptr pLaserCloudIn) : mpLink3dExtractor(pLink3dExtractor)
    {
        mnId = nNextId++;

        (*mpLink3dExtractor)(pLaserCloudIn, mvAggregationKeypoints, mDescriptors, mClusterEdgeKeypoints);
    }
}
