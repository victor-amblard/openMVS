/*
* SceneDensify.h
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#ifndef _MVS_SCENEDENSIFY_H_
#define _MVS_SCENEDENSIFY_H_


// I N C L U D E S /////////////////////////////////////////////////



// S T R U C T S ///////////////////////////////////////////////////


namespace MVS {
	
// Forward declarations
struct MVS_API gpsCoords{
    float longitude;
    float latitude;
    float altitude;
    float r_x; //roll
    float r_y; //pitch
    float r_z; //heading
};


  struct MVS_API relTransform{
 	 	Eigen::Matrix4f mSE3;
 	 	octomath::Pose6D p6D;
  };
const int WIDTH = 1242;
const int HEIGHT = 375;
const Eigen::Matrix4f transformImuLidar((Eigen::Matrix4f() << 9.999976e-01, 7.553071e-04 ,-2.035826e-03 , -8.086759e-01,
                                          -7.854027e-04 , 9.998898e-01, -1.482298e-02 , 3.195559e-01,
                                          2.024406e-03,  1.482454e-02,  9.998881e-01 , -7.997231e-01,
                                           0, 0 ,0,1).finished());

const Eigen::Matrix4f transformLidarCam((Eigen::Matrix4f() << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03 ,
                                          1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
                                          9.998621e-01, 7.523790e-03, 1.480755e-02,  -2.717806e-01,0, 0, 0, 1).finished());

const Eigen::Matrix4f R_rect_0((Eigen::Matrix4f() << 9.999239e-01, 9.837760e-03, -7.445048e-03,0,
                                 -9.869795e-03, 9.999421e-01, -4.278459e-03,0,
                                 7.402527e-03, 4.351614e-03, 9.999631e-01,0,
                                 0, 0 ,0, 1).finished());

const Eigen::Matrix<float,3,4> P_rect_2((Eigen::Matrix<float,3,4>() << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
                                           0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
                                            0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03).finished());

class MVS_API Scene;
	
// structure used to compute all depth-maps
class MVS_API DepthMapsData
{
public:
	DepthMapsData(Scene& _scene);
	~DepthMapsData();

	bool SelectViews(IIndexArr& images, IIndexArr& imagesMap, IIndexArr& neighborsMap);
	bool SelectViews(DepthData& depthData);
	bool InitViews(DepthData& depthData, IIndex idxNeighbor, IIndex numNeighbors);
	bool InitDepthMap(DepthData& depthData);
	bool EstimateDepthMap(IIndex idxImage);
	
    void addLidarData(const int& idx);
    void loadAllLidar();
    PointCloudXYZ::Ptr readLidarPcl(const int& idx);
    gpsCoords readIMUFile(const std::string& filename);
	Eigen::Matrix4f getRelativeTransform(const gpsCoords& curPose, const float& scale, Eigen::Matrix4f *Tr_0_inv);
	std::vector<LidarMap> backProjectLidarPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& all_pl,
                                         				const std::vector<Eigen::Matrix4f>& transforms);
	std::vector<relTransform> getTransformIMU(const std::vector<gpsCoords>& imu_poses);
    void mergeLidarPoints(std::vector<PointCloudXYZ::Ptr> allClouds, PointCloudXYZ::Ptr nCloud);
    pcl::PointCloud<pcl::XPointXYZ> checkDepthMaps(pcl::PointCloud<pcl::XPointXYZ> regPl);

    void ProjectLidarWorld(PointCloudXYZ::Ptr Pl, PointCloudXYZ::Ptr nPl, const int& idx);


    pcl::PointCloud<pcl::XPointXYZ> downsamplePcl(PointCloudXYZ::Ptr cloud, const float& leafSize);

    void testDensityPointCloud(pcl::PointCloud<pcl::XPointXYZ> pCloud);
	bool RemoveSmallSegments(DepthData& depthData);
	bool GapInterpolation(DepthData& depthData);

	bool FilterDepthMap(DepthData& depthData, const IIndexArr& idxNeighbors, bool bAdjust=true);
	void FuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal);

protected:
	static void* STCALL ScoreDepthMapTmp(void*);
	static void* STCALL EstimateDepthMapTmp(void*);
	static void* STCALL EndDepthMapTmp(void*);

public:
  Scene& scene;



  DepthDataArr arrDepthData;

	// used internally to estimate the depth-maps
	Image8U::Size prevDepthMapSize; // remember the size of the last estimated depth-map
	Image8U::Size prevDepthMapSizeTrg; // ... same for target image
	DepthEstimator::MapRefArr coords; // map pixel index to zigzag matrix coordinates
	DepthEstimator::MapRefArr coordsTrg; // ... same for target image
};
/*----------------------------------------------------------------*/

struct MVS_API DenseDepthMapData {
	Scene& scene;
	IIndexArr images;
	IIndexArr neighborsMap;
	DepthMapsData depthMaps;
	volatile Thread::safe_t idxImage;
	SEACAVE::EventQueue events; // internal events queue (processed by the working threads)
	Semaphore sem;
	CAutoPtr<Util::Progress> progress;

	DenseDepthMapData(Scene& _scene)
		: scene(_scene), depthMaps(_scene), idxImage(0), sem(1) {}

	void SignalCompleteDepthmapFilter() {
		ASSERT(idxImage > 0);
		if (Thread::safeDec(idxImage) == 0)
			sem.Signal((unsigned)images.GetSize()*2);
	}
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
