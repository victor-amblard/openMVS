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

class MVS_API Scene;

struct MyPlane{
	Point3 corners[4];
};

class MyLine{
    public:
        Eigen::Matrix4d pluckerMatrix;
        Eigen::Matrix<double,6,1> pluckerVector;

        Eigen::Vector3d getProjection(const Eigen::Matrix4d & projMat) const;

        MyLine(const Eigen::Vector3d pA, const Eigen::Vector3d pB){
            Eigen::Vector4d A = pA.homogeneous();
            Eigen::Vector4d B = pB.homogeneous();

            pluckerMatrix = A*B.transpose()-B*A.transpose();
            setVectorFromMatrix();

        }
		MyLine(const Point3& pA, const Point3& pB):MyLine(Eigen::Vector3d(pA.x,pA.y,pA.z), Eigen::Vector3d(pB.x, pB.y, pB.z)){}
    private:
        void setMatrixFromVector(void){
            pluckerMatrix = Eigen::Matrix4d::Zero();
            for(size_t i=1;i<4;++i){
                pluckerMatrix(i,0) = pluckerVector(i-1);
            }
            pluckerMatrix(2,1) = pluckerVector(3);
            pluckerMatrix(3,1) = pluckerVector(4);
            pluckerMatrix(3,2) = pluckerVector(5);

            for(size_t i=0;i<4;++i)
                for(size_t j=i;j<4;++j)
                    pluckerMatrix(i,j) = -pluckerMatrix(j,i);
        }
        void setVectorFromMatrix(void){
            pluckerVector(0) = pluckerMatrix(1,0);
            pluckerVector(1) = pluckerMatrix(2,0);
            pluckerVector(2) = pluckerMatrix(3,0);
            pluckerVector(3) = pluckerMatrix(2,1);
            pluckerVector(4) = pluckerMatrix(3,1);
            pluckerVector(5) = pluckerMatrix(3,2);
        }
    };

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
	
	Eigen::Matrix4f getRelativeTransform(const gpsCoords& curPose, const float& scale, Eigen::Matrix4f *Tr_0_inv);

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
