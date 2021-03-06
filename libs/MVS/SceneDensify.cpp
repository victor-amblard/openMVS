/*
* SceneDensify.cpp
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

#include "Common.h"
#include "Scene.h"
// MRF: view selection
#include "../Math/TRWS/MRFEnergy.h"
// CGAL: depth-map initialization
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Polygon_2.h>


#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/intersections.h>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define DENSE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

// Dense3D data.events
enum EVENT_TYPE {
    EVT_FAIL = 0,
    EVT_CLOSE,

    EVT_PROCESSIMAGE,

    EVT_ESTIMATEDEPTHMAP,
    EVT_OPTIMIZEDEPTHMAP,
    EVT_SAVEDEPTHMAP,

    EVT_FILTERDEPTHMAP,
    EVT_ADJUSTDEPTHMAP,
};

class EVTFail : public Event
{
public:
    EVTFail() : Event(EVT_FAIL) {}
};
class EVTClose : public Event
{
public:
    EVTClose() : Event(EVT_CLOSE) {}
};

class EVTProcessImage : public Event
{
public:
    IIndex idxImage;
    EVTProcessImage(IIndex _idxImage) : Event(EVT_PROCESSIMAGE), idxImage(_idxImage) {}
};

class EVTEstimateDepthMap : public Event
{
public:
    IIndex idxImage;
    EVTEstimateDepthMap(IIndex _idxImage) : Event(EVT_ESTIMATEDEPTHMAP), idxImage(_idxImage) {}
};
class EVTOptimizeDepthMap : public Event
{
public:
    IIndex idxImage;
    EVTOptimizeDepthMap(IIndex _idxImage) : Event(EVT_OPTIMIZEDEPTHMAP), idxImage(_idxImage) {}
};
class EVTSaveDepthMap : public Event
{
public:
    IIndex idxImage;
    EVTSaveDepthMap(IIndex _idxImage) : Event(EVT_SAVEDEPTHMAP), idxImage(_idxImage) {}
};

class EVTFilterDepthMap : public Event
{
public:
    IIndex idxImage;
    EVTFilterDepthMap(IIndex _idxImage) : Event(EVT_FILTERDEPTHMAP), idxImage(_idxImage) {}
};
class EVTAdjustDepthMap : public Event
{
public:
    IIndex idxImage;
    EVTAdjustDepthMap(IIndex _idxImage) : Event(EVT_ADJUSTDEPTHMAP), idxImage(_idxImage) {}
};
/*----------------------------------------------------------------*/


// convert the ZNCC score to a weight used to average the fused points
inline float Conf2Weight(float conf, Depth depth) {
    return 1.f/(MAXF(1.f-conf,0.03f)*depth*depth);
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////


DepthMapsData::DepthMapsData(Scene& _scene)
    :
    scene(_scene),
    arrDepthData(_scene.images.GetSize())
{
} // constructor

DepthMapsData::~DepthMapsData()
{
} // destructor
/*----------------------------------------------------------------*/


// globally choose the best target view for each image,
// trying in the same time the selected image pairs to cover the whole scene;
// the map of selected neighbors for each image is returned in neighborsMap.
// For each view a list of neighbor views ordered by number of shared sparse points and overlapped image area is given.
// Next a graph is formed such that the vertices are the views and two vertices are connected by an edge if the two views have each other as neighbors.
// For each vertex, a list of possible labels is created using the list of neighbor views and scored accordingly (the score is normalized by the average score).
// For each existing edge, the score is defined such that pairing the same two views for any two vertices is discouraged (a constant high penalty is applied for such edges).
// This primal-dual defined problem, even if NP hard, can be solved by a Belief Propagation like algorithm, obtaining in general a solution close enough to optimality.
bool DepthMapsData::SelectViews(IIndexArr& images, IIndexArr& imagesMap, IIndexArr& neighborsMap)
{
    // find all pair of images valid for dense reconstruction
    typedef std::unordered_map<uint64_t,float> PairAreaMap;
    PairAreaMap edges;
    double totScore(0);
    unsigned numScores(0);
    FOREACH(i, images) {
        const IIndex idx(images[i]);
        ASSERT(imagesMap[idx] != NO_ID);
        const ViewScoreArr& neighbors(arrDepthData[idx].neighbors);
        ASSERT(neighbors.GetSize() <= OPTDENSE::nMaxViews);
        // register edges
        FOREACHPTR(pNeighbor, neighbors) {
            const IIndex idx2(pNeighbor->idx.ID);
            ASSERT(imagesMap[idx2] != NO_ID);
            edges[MakePairIdx(idx,idx2)] = pNeighbor->idx.area;
            totScore += pNeighbor->score;
            ++numScores;
        }
    }
    if (edges.empty())
        return false;
    const float avgScore((float)(totScore/(double)numScores));

    // run global optimization
    const float fPairwiseMul = OPTDENSE::fPairwiseMul; // default 0.3
    const float fEmptyUnaryMult = 6.f;
    const float fEmptyPairwise = 8.f*OPTDENSE::fPairwiseMul;
    const float fSamePairwise = 24.f*OPTDENSE::fPairwiseMul;
    const IIndex _num_labels = OPTDENSE::nMaxViews+1; // N neighbors and an empty state
    const IIndex _num_nodes = images.GetSize();
    typedef MRFEnergy<TypeGeneral> MRFEnergyType;
    CAutoPtr<MRFEnergyType> energy(new MRFEnergyType(TypeGeneral::GlobalSize()));
    CAutoPtrArr<MRFEnergyType::NodeId> nodes(new MRFEnergyType::NodeId[_num_nodes]);
    typedef SEACAVE::cList<TypeGeneral::REAL, const TypeGeneral::REAL&, 0> EnergyCostArr;
    // unary costs: inverse proportional to the image pair score
    EnergyCostArr arrUnary(_num_labels);
    for (IIndex n=0; n<_num_nodes; ++n) {
        const ViewScoreArr& neighbors(arrDepthData[images[n]].neighbors);
        FOREACH(k, neighbors)
            arrUnary[k] = avgScore/neighbors[k].score; // use average score to normalize the values (not to depend so much on the number of features in the scene)
        arrUnary[neighbors.GetSize()] = fEmptyUnaryMult*(neighbors.IsEmpty()?avgScore*0.01f:arrUnary[neighbors.GetSize()-1]);
        nodes[n] = energy->AddNode(TypeGeneral::LocalSize(neighbors.GetSize()+1), TypeGeneral::NodeData(arrUnary.Begin()));
    }
    // pairwise costs: as ratios between the area to be covered and the area actually covered
    EnergyCostArr arrPairwise(_num_labels*_num_labels);
    for (PairAreaMap::const_reference edge: edges) {
        const PairIdx pair(edge.first);
        const float area(edge.second);
        const ViewScoreArr& neighborsI(arrDepthData[pair.i].neighbors);
        const ViewScoreArr& neighborsJ(arrDepthData[pair.j].neighbors);
        arrPairwise.Empty();
        FOREACHPTR(pNj, neighborsJ) {
            const IIndex i(pNj->idx.ID);
            const float areaJ(area/pNj->idx.area);
            FOREACHPTR(pNi, neighborsI) {
                const IIndex j(pNi->idx.ID);
                const float areaI(area/pNi->idx.area);
                arrPairwise.Insert(pair.i == i && pair.j == j ? fSamePairwise : fPairwiseMul*(areaI+areaJ));
            }
            arrPairwise.Insert(fEmptyPairwise+fPairwiseMul*areaJ);
        }
        for (const ViewScore& Ni: neighborsI) {
            const float areaI(area/Ni.idx.area);
            arrPairwise.Insert(fPairwiseMul*areaI+fEmptyPairwise);
        }
        arrPairwise.Insert(fEmptyPairwise*2);
        const IIndex nodeI(imagesMap[pair.i]);
        const IIndex nodeJ(imagesMap[pair.j]);
        energy->AddEdge(nodes[nodeI], nodes[nodeJ], TypeGeneral::EdgeData(TypeGeneral::GENERAL, arrPairwise.Begin()));
    }

    // minimize energy
    MRFEnergyType::Options options;
    options.m_eps = OPTDENSE::fOptimizerEps;
    options.m_iterMax = OPTDENSE::nOptimizerMaxIters;
    #ifndef _RELEASE
    options.m_printIter = 1;
    options.m_printMinIter = 1;
    #endif
    #if 1
    TypeGeneral::REAL energyVal, lowerBound;
    energy->Minimize_TRW_S(options, lowerBound, energyVal);
    #else
    TypeGeneral::REAL energyVal;
    energy->Minimize_BP(options, energyVal);
    #endif

    // extract optimized depth map
    neighborsMap.Resize(_num_nodes);
    for (IIndex n=0; n<_num_nodes; ++n) {
        const ViewScoreArr& neighbors(arrDepthData[images[n]].neighbors);
        IIndex& idxNeighbor = neighborsMap[n];
        const IIndex label((IIndex)energy->GetSolution(nodes[n]));
        ASSERT(label <= neighbors.GetSize());
        if (label == neighbors.GetSize()) {
            idxNeighbor = NO_ID; // empty
        } else {
            idxNeighbor = label;
            DEBUG_ULTIMATE("\treference image %3u paired with target image %3u (idx %2u)", images[n], neighbors[label].idx.ID, label);
        }
    }

    // remove all images with no valid neighbors
    RFOREACH(i, neighborsMap) {
        if (neighborsMap[i] == NO_ID) {
            // remove image with no neighbors
            for (IIndex& imageMap: imagesMap)
                if (imageMap != NO_ID && imageMap > i)
                    --imageMap;
            imagesMap[images[i]] = NO_ID;
            images.RemoveAtMove(i);
            neighborsMap.RemoveAtMove(i);
        }
    }
    return !images.IsEmpty();
} // SelectViews
/*----------------------------------------------------------------*/

// compute visibility for the reference image (the first image in "images")
// and select the best views for reconstructing the depth-map;
// extract also all 3D points seen by the reference image
bool DepthMapsData::SelectViews(DepthData& depthData)
{
    // find and sort valid neighbor views
    const IIndex idxImage((IIndex)(&depthData-arrDepthData.Begin()));
    ASSERT(depthData.neighbors.IsEmpty());
    ASSERT(scene.images[idxImage].neighbors.IsEmpty());
    if (!scene.SelectNeighborViews(idxImage, depthData.points, OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, FD2R(OPTDENSE::fOptimAngle)))
        return false;
    depthData.neighbors.CopyOf(scene.images[idxImage].neighbors);

    // remove invalid neighbor views
    const float fMinArea(OPTDENSE::fMinArea);
    const float fMinScale(0.2f), fMaxScale(3.2f);
    const float fMinAngle(FD2R(OPTDENSE::fMinAngle));
    const float fMaxAngle(FD2R(OPTDENSE::fMaxAngle));
    if (!Scene::FilterNeighborViews(depthData.neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, OPTDENSE::nMaxViews)) {
        DEBUG_EXTRA("error: reference image %3u has no good images in view", idxImage);
        return false;
    }
    return true;
} // SelectViews
/*----------------------------------------------------------------*/

// select target image for the reference image (the first image in "images")
// and initialize images data;
// if idxNeighbor is not NO_ID, only the reference image and the given neighbor are initialized;
// if numNeighbors is not 0, only the first numNeighbors neighbors are initialized;
// otherwise all are initialized;
// returns false if there are no good neighbors to estimate the depth-map
bool DepthMapsData::InitViews(DepthData& depthData, IIndex idxNeighbor, IIndex numNeighbors)
{
    const IIndex idxImage((IIndex)(&depthData-arrDepthData.Begin()));
    ASSERT(!depthData.neighbors.IsEmpty());
    ASSERT(depthData.images.IsEmpty());

    // set this image the first image in the array
    depthData.images.Reserve(depthData.neighbors.GetSize()+1);
    depthData.images.AddEmpty();

    if (idxNeighbor != NO_ID) {
        // set target image as the given neighbor
        const ViewScore& neighbor = depthData.neighbors[idxNeighbor];
        DepthData::ViewData& imageTrg = depthData.images.AddEmpty();
        imageTrg.pImageData = &scene.images[neighbor.idx.ID];
        imageTrg.scale = neighbor.idx.scale;
        imageTrg.camera = imageTrg.pImageData->camera;
        imageTrg.pImageData->image.toGray(imageTrg.image, cv::COLOR_BGR2GRAY, true);
        if (imageTrg.ScaleImage(imageTrg.image, imageTrg.image, imageTrg.scale))
            imageTrg.camera = imageTrg.pImageData->GetCamera(scene.platforms, imageTrg.image.size());
        DEBUG_EXTRA("Reference image %3u paired with image %3u", idxImage, neighbor.idx.ID);
    } else {
        // init all neighbor views too (global reconstruction is used)
        const float fMinScore(MAXF(depthData.neighbors.First().score*(OPTDENSE::fViewMinScoreRatio*0.1f), OPTDENSE::fViewMinScore));
        FOREACH(idx, depthData.neighbors) {
            const ViewScore& neighbor = depthData.neighbors[idx];
            if ((numNeighbors && depthData.images.GetSize() > numNeighbors) ||
                (neighbor.score < fMinScore))
                break;
            DepthData::ViewData& imageTrg = depthData.images.AddEmpty();
            imageTrg.pImageData = &scene.images[neighbor.idx.ID];
            imageTrg.scale = neighbor.idx.scale;
            imageTrg.camera = imageTrg.pImageData->camera;
            imageTrg.pImageData->image.toGray(imageTrg.image, cv::COLOR_BGR2GRAY, true);
            if (imageTrg.ScaleImage(imageTrg.image, imageTrg.image, imageTrg.scale))
                imageTrg.camera = imageTrg.pImageData->GetCamera(scene.platforms, imageTrg.image.size());
        }
        #if TD_VERBOSE != TD_VERBOSE_OFF
        // print selected views
        if (g_nVerbosityLevel > 2) {
            String msg;
            for (IIndex i=1; i<depthData.images.GetSize(); ++i)
                msg += String::FormatString(" %3u(%.2fscl)", depthData.images[i].GetID(), depthData.images[i].scale);
            VERBOSE("Reference image %3u paired with %u views:%s (%u shared points)", idxImage, depthData.images.GetSize()-1, msg.c_str(), depthData.points.GetSize());
        } else
        DEBUG_EXTRA("Reference image %3u paired with %u views", idxImage, depthData.images.GetSize()-1);
        #endif
    }
    if (depthData.images.GetSize() < 2) {
        depthData.images.Release();
        return false;
    }

    // init the first image as well
    DepthData::ViewData& imageRef = depthData.images.First();
    imageRef.scale = 1;
    imageRef.pImageData = &scene.images[idxImage];
    imageRef.pImageData->image.toGray(imageRef.image, cv::COLOR_BGR2GRAY, true);
    imageRef.camera = imageRef.pImageData->camera;
    return true;
} // InitViews
/*----------------------------------------------------------------*/

namespace CGAL {
typedef CGAL::Simple_cartesian<double> kernel_t;
typedef CGAL::Projection_traits_xy_3<kernel_t> Geometry;
typedef CGAL::Delaunay_triangulation_2<Geometry> Delaunay;
typedef CGAL::Delaunay::Face_circulator FaceCirculator;
typedef CGAL::Delaunay::Face_handle FaceHandle;
typedef CGAL::Delaunay::Vertex_circulator VertexCirculator;
typedef CGAL::Delaunay::Vertex_handle VertexHandle;
typedef kernel_t::Point_3 Point;
}

// triangulate in-view points, generating a 2D mesh
// return also the estimated depth boundaries (min and max depth)

bool checkCameraLidarConsistency(const int x, const int y, const int w, const int h, const DepthMap& xCam){
    return ISEQUAL(xCam(y,x), 0.f);
    /**
    const int neighbors(2);

    int minX(MAXF(0, x-neighbors));
    int minY(MAXF(0, y-neighbors));
    int maxX(MINF(x+neighbors, w-1));
    int maxY(MINF(y+neighbors, h-1));

    for(auto iX=minX;iX<=maxX;++iX)
        for (auto iY=minY;iY<=maxY;++iY)
            if (!ISEQUAL(xCam(iY, iX), 0.f))
                return false;

    return true;
    **/
}

//We can't really do raytracing because we don't have any surface
//Instead we do something simple and check whether the variance of the depth in the area
//is above a threshold (meaning that we have a potential occlusion)
// but we dont want to remove edges ==> we need to check variance in both directions

bool checkLidarConsistency(const Point3& p, const int w, const int h, const DepthMap& xLid){
    return ISEQUAL(xLid((int)p.y, (int)p.x), p.z);
    /**
    const int neighbors(5);

    const float maxDiffDepth(0.1);

    int minX(MAXF(0, int(p.x)-neighbors));
    int minY(MAXF(0, int(p.y)-neighbors));
    int maxX(MINF(int(p.x)+neighbors, w-1));
    int maxY(MINF(int(p.y)+neighbors, h-1));

    int count(0);
    int countX(0);
    int countY(0);
    float avgX(0);
    float avgY(0);
    float avg(0);

    for(auto iX=minX;iX<=maxX;++iX){
        for (auto iY=minY;iY<=maxY;++iY){
            if (!ISEQUAL(xLid(iY, iX), 0.f)){
                ++count;
                avg+=xLid(iY, iX);
            }
        }
        if (count){
            ++countY;
            avg /= count;
            avgY += avg;
            avg=0;
            count=0;
        }
    }

    for(auto iY=minY;iY<=maxY;++iY){
        for (auto iX=minX;iX<=maxX;++iX){
            if (!ISEQUAL(xLid(iY, iX), 0.f)){
                ++count;
                avg +=xLid(iY, iX);
            }
        }
        if (count){
            ++countX;
            avg /= count;
            avgX += avg;
            avg=0;
            count=0;
        }
    }


    if (!count)
        return true;

    avgX /= countX;
    avgY /= countY;

    if (xLid(p.y,p.x) > avgX+maxDiffDepth && xLid(p.y, p.x) > avgY + maxDiffDepth)
        return false;

    return true;
    **/
}

//Plane is an array (a,b,c,d) ax+by+cz+d=0
typedef CGAL::Exact_predicates_exact_constructions_kernel CGAL_K;

template <typename TYPE>
inline CGAL_K::Point_3 MVS2CGAL3(const TPoint3<TYPE>& p) {
    return CGAL_K::Point_3((CGAL_K::RT)p.x, (CGAL_K::RT)p.y, (CGAL_K::RT)p.z);
}
template <typename TYPE>
inline CGAL_K::Point_2 MVS2CGAL2(const TPoint2<TYPE>& p) {
    return CGAL_K::Point_2((CGAL_K::RT)p.x, (CGAL_K::RT)p.y);
}
std::pair<bool, double> getApproxDepthFromPlane(const Point2& pt, const CGAL_K::Plane_3& curPlane, const Camera& cam){

   const Point3 pixelDir(cam.RayPoint(pt));
   const CGAL_K::Ray_3 curRay(MVS2CGAL3<REAL>(cam.C), CGAL_K::Direction_3(pixelDir.x, pixelDir.y, pixelDir.z));

    auto result = CGAL::intersection(curPlane, curRay);
    if (result){
        if (const CGAL_K::Point_3* p = boost::get<CGAL_K::Point_3>(&*result)){
            const Point3& ptOnPlane = Point3(CGAL::to_double(p->x()), CGAL::to_double(p->y()), CGAL::to_double(p->z()));
            double depth(cam.ProjectPointP3(Point3(ptOnPlane)).z);
            return std::make_pair(true, depth);
        }else{
            const CGAL_K::Ray_3* r = boost::get<CGAL_K::Ray_3>(&*result);
            return std::make_pair(false, 0.);
        }
    }
    return std::make_pair(false, 0.);
}
bool isPointInsideProjectedPlane(const Point2& pt, CGAL_K::Point_2 *pgn_begin, CGAL_K::Point_2 *pgn_end, CGAL_K traits)
{
  CGAL_K::Point_2 curPt = MVS2CGAL2<REAL>(Point2(pt.x, pt.y));

  if(CGAL::bounded_side_2(pgn_begin, pgn_end,curPt, traits)==CGAL::ON_BOUNDED_SIDE)
    return true;
  else
    return false;
}


// A plane is seen iff one of its corners is inside the image
// Then we'd like to retrieve the polygon corresponding to the visible part of the plane
// We identify the visible corners and the lines corresponding to those corners 
std::pair<bool, std::vector<CGAL_K::Point_2>> isPlaneInsideImg(MyPlane plane, const DepthData::ViewData& image){

    bool visibility[4] = {false, false, false, false};
    int nVis(0);
    std::vector<CGAL_K::Point_2> pointsPolygon(4);

    for (int iLine = 0;iLine<4;++iLine){
        Point3 pCorner = image.camera.ProjectPointP3(plane.corners[iLine]);
        if (pCorner.z >=0 && pCorner.x >=0 && pCorner.y >=0 && pCorner.x/pCorner.z < image.image.width() && pCorner.y/pCorner.z < image.image.height()){
            visibility[iLine] = true;
            pointsPolygon.at(iLine) = MVS2CGAL2<REAL>(Point2(pCorner.x/pCorner.z, pCorner.y/pCorner.z));
            ++nVis;
        }
        else{
            visibility[iLine] = false;
        }
    }
    
    std::cerr << nVis << " corners in view" << std::endl;

    if (!nVis || nVis==1)
        return std::make_pair(false, pointsPolygon);

    
    CGAL_K::Point_2 topleftImage((CGAL_K::RT)(0), (CGAL_K::RT)(0));
    CGAL_K::Point_2 bottomRightImage((CGAL_K::RT)(image.image.width()), (CGAL_K::RT)(image.image.height()));

    CGAL_K::Iso_rectangle_2 imageBounds(topleftImage, bottomRightImage);
    Eigen::Matrix<double,3,4> projMat34 = Eigen::Matrix<double,3,4>::Zero();

    for(size_t i=0;i<3;++i)
        for(size_t j=0;j<4;++j)
            projMat34(i,j) = image.camera.P(i,j);

    Eigen::Matrix4d projMatrix = Eigen::Matrix4d::Identity();
    projMatrix.block(0,0,3,4) = projMat34;

    for (auto i=0;i<4;++i){
        int nxt = (i==3)?0:i+1;
        if ((visibility[i] || visibility[nxt]) && (visibility[i] != visibility[nxt])){ 
            //We have to come back to 3D infinite lines representation
            int iVisible = (visibility[i])?i:nxt;
            int notVisible = (!visibility[i])?i:nxt;
            Eigen::Matrix4d projPlucker = projMatrix * MyLine(plane.corners[i], plane.corners[nxt]).pluckerMatrix * projMatrix.transpose(); 
            Eigen::Vector3d lineProj(projPlucker(2,1), projPlucker(0,2), projPlucker(1,0));

            Point3 projNVisible = image.camera.ProjectPointP3(plane.corners[notVisible]);
            CGAL_K::Point_2 projVisible = MVS2CGAL2<REAL>(image.camera.ProjectPointP(plane.corners[iVisible]));


            if (projNVisible.z >= 0){
                CGAL_K::Ray_2 curRay  = CGAL_K::Ray_2(projVisible, MVS2CGAL2<REAL>(Point2(projNVisible.x/projNVisible.z, projNVisible.y/projNVisible.z)) );
                auto resultInter = CGAL::intersection(imageBounds, curRay);
                if (resultInter){
                    if (const CGAL_K::Point_2* p = boost::get<CGAL_K::Point_2>(&*resultInter)){
                        pointsPolygon.at(notVisible) = *p;
                    }else{
                        const CGAL_K::Segment_2* r = boost::get<CGAL_K::Segment_2>(&*resultInter);
                            pointsPolygon.at(notVisible) = r->target();
                        }
                }else{
                    std::cerr << "ERROR! Failed to estimate corner on boundary!" << std::endl;
                }

            }else{
                CGAL_K::Line_2 curLine = CGAL_K::Line_2((CGAL_K::RT)lineProj.x(), (CGAL_K::RT)lineProj.y(), (CGAL_K::RT)lineProj.z());
                auto resultInter = CGAL::intersection(imageBounds, curLine);
                if (resultInter){
                    if (const CGAL_K::Point_2* p = boost::get<CGAL_K::Point_2>(&*resultInter)){
                        pointsPolygon.at(notVisible) = *p;
                    }else{
                        std::cerr << "Visible: " << CGAL::to_double(projVisible.x()) << " " <<CGAL::to_double(projVisible.y()) << std::endl;
                        const CGAL_K::Segment_2* r = boost::get<CGAL_K::Segment_2>(&*resultInter);
                        std::cerr << CGAL::to_double(r->source().x()) << " " <<CGAL::to_double(r->source().y()) << std::endl;
                        std::cerr << CGAL::to_double(r->target().x()) << " " <<CGAL::to_double(r->target().y()) << std::endl;
                        if (CGAL::to_double(r->target().x()) - image.camera.K(0,2) > 0 && projNVisible.x - image.camera.K(0,2) > 0){
                            std::cerr << "-> target" << std::endl;
                            pointsPolygon.at(notVisible) = r->target();
                        }else{
                            pointsPolygon.at(notVisible) = r->source();
                            std::cerr << "-> source" << std::endl;

                        }
                    }
                }else{
                    std::cerr << "ERROR! Failed to estimate corner on boundary!" << std::endl;
                }
            }
        }
    }

    if (nVis <= 3){
        std::cerr << "Before adding points, the polygon has " << pointsPolygon.size() << " points!" << std::endl ;
        Point2 corners[4] = {Point2(0,0), Point2(image.image.width(),0), Point2(image.image.width(),image.image.height()), Point2(0,image.image.height())};

        for (auto i=0;i<4;++i){
            int nxt = (i==3)?0:i+1;
            if (!visibility[i] && !visibility[nxt]){ //We might need to add extra points to the polygon 
                int numLineA = (ISEQUAL(CGAL::to_double(pointsPolygon.at(i).x()), 0) << 1) | ((!ISEQUAL(CGAL::to_double(pointsPolygon.at(i).y()), 0)) << 0);
                int numLineB = (ISEQUAL(CGAL::to_double(pointsPolygon.at(nxt).x()), 0) << 1) | ((!ISEQUAL(CGAL::to_double(pointsPolygon.at(nxt).y()), 0)) << 0);
                std::cerr << numLineA << " " << numLineB << std::endl;
                if (numLineA != numLineB){ //We need to add at least 1 corner
                    // The easiest way to find out which corners we need to add is just to test both directions 
                    // and the convexity of the resulting polygon

                    std::vector<CGAL_K::Point_2> nPoints;
                    bool pickedLines[4] = {false, false, false, false};

                    for (auto j=0;j<=i;++j)
                        nPoints.push_back(pointsPolygon.at(j));
                
                    for(auto iCorner=std::min(numLineA, numLineB);iCorner<std::max(numLineA, numLineB);++iCorner){
                        nPoints.push_back(MVS2CGAL2<REAL>(corners[(iCorner+1)%4]));

                        pickedLines[iCorner]=true;
                    }
                    
                    for(auto j=i+1;j<4;++j)
                        nPoints.push_back(pointsPolygon.at(j));
                    
                    CGAL::Polygon_2<CGAL_K> pgn(&*nPoints.begin(), &*nPoints.end());

                    if(pgn.is_convex()){
                        pointsPolygon = nPoints;
                    }else{
                        nPoints.clear();
                        for (auto j=0;j<=i;++j)
                            nPoints.push_back(pointsPolygon.at(j));
                    
                        for(auto iCorner=0;iCorner<4;++iCorner)
                            if (!pickedLines[iCorner])
                                nPoints.push_back(MVS2CGAL2<REAL>(corners[(iCorner+1)%4]));
                        
                        for(auto j=i+1;j<4;++j)
                            nPoints.push_back(pointsPolygon.at(j));
                        
                        pgn = CGAL::Polygon_2<CGAL_K>(&*nPoints.begin(), &*nPoints.end());
                        pointsPolygon = nPoints;
                        if (!pgn.is_convex())
                            std::cerr << "Shouldn't happen!" << std::endl;
                    }
                }
            }
        }
    }
    // If only one point was initially visible, we still have one point to estimate
    return std::make_pair(true, pointsPolygon);
}
void fillDepthMapWPlanes(DepthMap& map2fill, const DepthData::ViewData& image, const std::vector<MyPlane>& allPlanes){
    const int imWidth = image.image.width();
    const int imHeight = image.image.height();
    const int gridSpacing(6);
    for(const auto& plane: allPlanes){
        CGAL_K::Plane_3 plane3d(MVS2CGAL3<REAL>(plane.corners[0]), MVS2CGAL3<REAL>(plane.corners[1]), MVS2CGAL3<REAL>(plane.corners[2]));
        Point3 curCameraLocation = image.camera.TransformPointC2W(Point3(0,0,0));
        //TODO: Check plane's orientation when creating the Plane_3
        auto resInside = isPlaneInsideImg(plane, image);
        if (resInside.first){ //check that the camera is facing the plane, not behind it
            std::cout << image.GetID() << std::endl;
            for(auto x=0;x<imWidth;++x){
                for (auto y=0;y<imHeight;++y){
                    auto curDepth = map2fill(y,x);
                    Point2 curPt(x,y);
                    if (x%gridSpacing==0 && y%gridSpacing==0 && isPointInsideProjectedPlane(curPt, &*resInside.second.begin(), &*resInside.second.end(), CGAL_K())){
                        std::pair<bool, double> resultDepth = getApproxDepthFromPlane(curPt, plane3d, image.camera);
                        if (resultDepth.first && (resultDepth.second < curDepth || ISEQUAL(curDepth, 0))){

                            map2fill(y,x) = resultDepth.second;
                        }
                    }
                }
            }
        }
    }
}
MyPlane createPlaneFrom4Points(Point3 * points){
  Eigen::Vector3d pA(points[0].x, points[0].y, points[0].z);
  Eigen::Vector3d pB(points[1].x, points[1].y, points[1].z);
  Eigen::Vector3d pC(points[2].x, points[2].y, points[2].z);
  Eigen::Vector3d pd(points[3].x, points[3].y, points[3].z);

  Eigen::Vector3d abc = (pB - pA).cross(pC-pA);
  double d = -abc(0)*points[0].x-abc(1)*points[0].y-abc(2)*points[0].z;
  Eigen::Vector3d pD = pd + (pd-pA).dot(abc)/abc.squaredNorm()*abc;

  MyPlane mp;
  mp.corners[0] = points[0];
  mp.corners[1] = points[1];
  mp.corners[2] = points[2];
  mp.corners[3] = Point3(pD.x(), pD.y(), pD.z());
  
  return mp;
  
}
std::pair<float,float> TriangulatePointsDelaunay(CGAL::Delaunay& delaunay, const Scene& scene, const DepthData::ViewData& image, const IndexArr& points, LidarMap& lPoints, uint8_t options)
{
    DEBUG_EXTRA("About to triangulate");

    ASSERT(sizeof(Point3) == sizeof(X3D));
    ASSERT(sizeof(Point3) == sizeof(CGAL::Point));

    uint8_t usePlanes = 1 << 2; //TODO: Add options in command line args
    uint8_t useLidar = 1 << 1;
    uint8_t useCamera = 1 << 0;

    std::pair<float,float> depthBounds(FLT_MAX, 0.f);
    const float eps = 0.0000001f;

    const int imWidth(image.image.width());
    const int imHeight(image.image.height());


    DepthMap xCam(imHeight, imWidth);
    //We first add LIDAR points, then visual points

    if (options & useCamera){

        for(auto x=0;x<imWidth;++x)
            for(auto y=0;y<imHeight;++y)
                xCam(y,x)=0;
        for (uint32_t idx: points) {
            const Point3f pt(image.camera.ProjectPointP3(scene.pointcloud.points[idx]));
            delaunay.insert(CGAL::Point(pt.x/pt.z, pt.y/pt.z, pt.z));
            int x,y;
            x = int(pt.x/pt.z);
            y = int(pt.y/pt.z);
           xCam(y,x)=pt.z;

           depthBounds.first = MINF(depthBounds.first, pt.z);
           depthBounds.second = MAXF(depthBounds.second, pt.z);

            }
        const Image8U3& im(image.pImageData->image);
        ExportOverlayedImage(ComposeDepthFilePath(image.GetID(), "raw_camera_points.png"), im, xCam, depthBounds.first, depthBounds.second, true);

    }

    if (options & useLidar){ //We assume that the LIDAR scan is aligned with the SfM cloud
         DepthMap xLid(imHeight, imWidth);
         std::vector<Point3> candidatePoints;

         for(auto x=0;x<imWidth;++x)
             for(auto y=0;y<imHeight;++y)
                 xLid(y,x)=0;

        for (auto it = lPoints->begin();it!=lPoints->end();){
            if (ISEQUAL(it->z, 0.f))
                continue;
            const Point3f pt(image.camera.ProjectPointP3(Point3d(it->x, it->y, it->z)));
            int x(pt.x/pt.z);
            int y(pt.y/pt.z);

            if (0 <= x && 0 <= y && x <  imWidth && y < imHeight){
                if (checkCameraLidarConsistency(x, y, imWidth, imHeight, xCam)){ //We just want to make sure we are not poluting a visual feature (more accurate)
                    //We delay the delaunay insertion because we need to do some post-processing to prevent spurious points
                    if (ISEQUAL(xLid(y,x),0.f) || pt.z < xLid(y,x)){
                        candidatePoints.push_back(Point3(pt.x/pt.z,pt.y/pt.z,pt.z));
                        xLid(y,x) = pt.z;
                    }

                    ++it;
                }else
                    it = lPoints->erase(it);

               }else{
                ++it;
            }
         }
        std::cerr << candidatePoints.size() << std::endl;

        for (Point3 curPoint: candidatePoints){
            if (checkLidarConsistency(curPoint, imWidth, imHeight, xLid)){

                delaunay.insert(CGAL::Point(curPoint.x, curPoint.y, curPoint.z));
                xCam(int(curPoint.y), int(curPoint.x)) = curPoint.z; //Just for visualization purposes
                depthBounds.first = MINF(depthBounds.first, static_cast<float>(curPoint.z));
                depthBounds.second = MAXF(depthBounds.second, static_cast<float>(curPoint.z));
              }
           }
         const Image8U3& im(image.pImageData->image);
         VERBOSE("%f, %f", depthBounds.first, depthBounds.second);
         ExportOverlayedImage(ComposeDepthFilePath(image.GetID(), "raw_lidar_points.png"), im, xCam, depthBounds.first, depthBounds.second, false);

    }
    if (options & usePlanes){
            DepthMap map2fill(imHeight, imWidth);
            for(auto x=0;x<imWidth;++x)
             for(auto y=0;y<imHeight;++y)
                 map2fill(y,x)=0;

            std::vector<MyPlane> planes;
            Point3 corners_1[4] = {Point3(-6.47,21.,-1.44), Point3(-6.66,20.6,7.73), Point3(-13.3,19.8,7.67), Point3(-13.1, 20.2, -0.84)};
            Point3 corners_2[4] = {Point3(-13.258301,20.378471,-0.662316), Point3(-13.740501,20.034218,10.402382), Point3(-17.336002,46.391182,11.441795),Point3(-17.234077,46.558113,-1.513632)};
            Point3 corners_3[4] = {Point3(6.267281,64.451820,11.510517), Point3(6.114540,64.451836,1.408474), Point3(8.864605,50.534073,-1.006508),Point3(8.991833,50.345917,10.174690)};

            planes.push_back(createPlaneFrom4Points(corners_1));
            planes.push_back(createPlaneFrom4Points(corners_2));
            planes.push_back(createPlaneFrom4Points(corners_3));
            fillDepthMapWPlanes(map2fill, image, planes);
            for(auto x=0;x<imWidth;++x)
                for(auto y=0;y<imHeight;++y)
                    if (!ISEQUAL(map2fill(y,x), 0.))
                        if (checkCameraLidarConsistency(x, y, imWidth, imHeight, xCam))
                            delaunay.insert(CGAL::Point(x,y,map2fill(y,x)));

            
            const Image8U3& im(image.pImageData->image);
            ExportOverlayedImage(ComposeDepthFilePath(image.GetID(), "raw_planes.png"), im, map2fill, depthBounds.first, depthBounds.second, false);
        }


    // if full size depth-map requested
    if (OPTDENSE::bAddCorners) {
        typedef TIndexScore<float,float> DepthDist;
        typedef CLISTDEF0(DepthDist) DepthDistArr;
        typedef Eigen::Map< Eigen::VectorXf, Eigen::Unaligned, Eigen::InnerStride<2> > FloatMap;
        // add the four image corners at the average depth
        const CGAL::VertexHandle vcorners[] = {
            delaunay.insert(CGAL::Point(0, 0, image.pImageData->avgDepth)),
            delaunay.insert(CGAL::Point(image.image.width(), 0, image.pImageData->avgDepth)),
            delaunay.insert(CGAL::Point(0, image.image.height(), image.pImageData->avgDepth)),
            delaunay.insert(CGAL::Point(image.image.width(), image.image.height(), image.pImageData->avgDepth))
        };
        // compute average depth from the closest 3 directly connected faces,
        // weighted by the distance
        const size_t numPoints = 3;
        for (int i=0; i<4; ++i) {
            const CGAL::VertexHandle vcorner = vcorners[i];
            CGAL::FaceCirculator cfc(delaunay.incident_faces(vcorner));
            if (cfc == 0)
                continue; // normally this should never happen
            const CGAL::FaceCirculator done(cfc);
            Point3d& poszA = reinterpret_cast<Point3d&>(vcorner->point());
            const Point2d& posA = reinterpret_cast<const Point2d&>(poszA);
            const Ray3d rayA(Point3d::ZERO, normalized(image.camera.TransformPointI2C(poszA)));
            DepthDistArr depths(0, numPoints);
            do {
                CGAL::FaceHandle fc(cfc->neighbor(cfc->index(vcorner)));
                if (fc == delaunay.infinite_face())
                    continue;
                for (int j=0; j<4; ++j)
                    if (fc->has_vertex(vcorners[j]))
                        goto Continue;
                // compute the depth as the intersection of the corner ray with
                // the plane defined by the face's vertices
                {
                const Point3d& poszB0 = reinterpret_cast<const Point3d&>(fc->vertex(0)->point());
                const Point3d& poszB1 = reinterpret_cast<const Point3d&>(fc->vertex(1)->point());
                const Point3d& poszB2 = reinterpret_cast<const Point3d&>(fc->vertex(2)->point());
                const Planed planeB(
                    image.camera.TransformPointI2C(poszB0),
                    image.camera.TransformPointI2C(poszB1),
                    image.camera.TransformPointI2C(poszB2)
                );
                const Point3d poszB(rayA.Intersects(planeB));
                if (poszB.z <= 0)
                    continue;
                const Point2d posB((
                    reinterpret_cast<const Point2d&>(poszB0)+
                    reinterpret_cast<const Point2d&>(poszB1)+
                    reinterpret_cast<const Point2d&>(poszB2))/3.f
                );
                const double dist(norm(posB-posA));
                depths.StoreTop<numPoints>(DepthDist(CLAMP((float)poszB.z,depthBounds.first,depthBounds.second), INVERT((float)dist)));
                }
                Continue:;
            } while (++cfc != done);
            if (depths.GetSize() != numPoints)
                continue; // normally this should never happen
            FloatMap vecDists(&depths[0].score, numPoints);
            vecDists *= 1.f/vecDists.sum();
            FloatMap vecDepths(&depths[0].idx, numPoints);
            poszA.z = vecDepths.dot(vecDists);
        }
    }
    return depthBounds;
}

// roughly estimate depth and normal maps by triangulating the sparse point cloud
// and interpolating normal and depth for all pixels
bool DepthMapsData::InitDepthMap(DepthData& depthData)
{
    TD_TIMER_STARTD();
    ASSERT(depthData.images.GetSize() > 1 && !depthData.points.IsEmpty());
    const DepthData::ViewData& image(depthData.images.First());
    ASSERT(!image.image.empty());
    DEBUG_EXTRA("Image has width %u and height %u", image.image.width(), image.image.height());
    // triangulate in-view points
    uint8_t useLC = 0;

    CGAL::Delaunay delaunay, delaunayCamera, delaunayLidar;

    useLC = (1 << 0); //| (1 << 2); //use planes
    if (depthData.lScan!=nullptr)
        useLC |= (1 << 1);
    const std::pair<float,float> thDepth(TriangulatePointsDelaunay(delaunay, scene, image, depthData.points, depthData.lScan, useLC));
    depthData.dMin = thDepth.first*0.9f;
    depthData.dMax = thDepth.second*1.1f;
    VERBOSE("Simultaneous depth map initialization");
    VERBOSE("%f,%f", depthData.dMin, depthData.dMax);
    
    // create rough depth-map by interpolating inside triangles
    const Camera& camera = image.camera;
    depthData.depthMap.create(image.image.size());
    depthData.normalMap.create(image.image.size());
    if (!OPTDENSE::bAddCorners) {
        depthData.depthMap.setTo(Depth(0));
        depthData.normalMap.setTo(0.f);
    }
    struct RasterDepthDataPlaneData {
        const Camera& P;
        DepthMap& depthMap;
        NormalMap& normalMap;
        Point3f normal;
        Point3f normalPlane;
        inline void operator()(const ImageRef& pt) {
            if (!depthMap.isInside(pt))
                return;
            const Depth z(INVERT(normalPlane.dot(P.TransformPointI2C(Point2f(pt)))));
            if (z <= 0) // due to numerical instability
                return;
            depthMap(pt) = z;
            normalMap(pt) = normal;
        }
    };            
    RasterDepthDataPlaneData data = {camera, depthData.depthMap, depthData.normalMap};

    for (CGAL::Delaunay::Face_iterator it=delaunay.faces_begin(); it!=delaunay.faces_end(); ++it) {

        const CGAL::Delaunay::Face& face = *it;
        const Point3f i0(reinterpret_cast<const Point3d&>(face.vertex(0)->point()));
        const Point3f i1(reinterpret_cast<const Point3d&>(face.vertex(1)->point()));
        const Point3f i2(reinterpret_cast<const Point3d&>(face.vertex(2)->point()));
        // compute the plane defined by the 3 points
        const Point3f c0(camera.TransformPointI2C(i0));
        const Point3f c1(camera.TransformPointI2C(i1));
        const Point3f c2(camera.TransformPointI2C(i2));
        const Point3f edge1(c1-c0);
        const Point3f edge2(c2-c0);
        data.normal = normalized(edge2.cross(edge1));
        data.normalPlane = data.normal * INVERT(data.normal.dot(c0));
        // draw triangle and for each pixel compute depth as the ray intersection with the plane
        Image8U::RasterizeTriangle(reinterpret_cast<const Point2f&>(i2), reinterpret_cast<const Point2f&>(i1), reinterpret_cast<const Point2f&>(i0), data);
    }
    ExportDepthMap(ComposeDepthFilePath(image.GetID(), "before.png"), depthData.depthMap);

    DEBUG_ULTIMATE("Depth-map %3u roughly estimated from %u sparse points: %dx%d (%s)", &depthData-arrDepthData.Begin(), depthData.points.GetSize(), image.image.width(), image.image.height(), TD_TIMER_GET_FMT().c_str());

    return true;
} // InitDepthMap
/*----------------------------------------------------------------*/


// initialize the confidence map (NCC score map) with the score of the current estimates
void* STCALL DepthMapsData::ScoreDepthMapTmp(void* arg)
{
    DepthEstimator& estimator = *((DepthEstimator*)arg);
    IDX idx;
    while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize()) {
        const ImageRef& x = estimator.coords[idx];
        if (!estimator.PreparePixelPatch(x) || !estimator.FillPixelPatch()) {
            estimator.depthMap0(x) = 0;
            estimator.normalMap0(x) = Normal::ZERO;
            estimator.confMap0(x) = 2.f;
            continue;
        }
        Depth& depth = estimator.depthMap0(x);
        Normal& normal = estimator.normalMap0(x);
        const Normal viewDir(Cast<float>(static_cast<const Point3&>(estimator.X0)));
        if (!ISINSIDE(depth, estimator.dMin, estimator.dMax)) {
            // init with random values
            depth = estimator.RandomDepth(estimator.dMinSqr, estimator.dMaxSqr);
            normal = estimator.RandomNormal(viewDir);
        } else if (normal.dot(viewDir) >= 0) {
            // replace invalid normal with random values
            normal = estimator.RandomNormal(viewDir);
        }
        estimator.confMap0(x) = estimator.ScorePixel(depth, normal);
    }
    return NULL;
}
// run propagation and random refinement cycles
void* STCALL DepthMapsData::EstimateDepthMapTmp(void* arg)
{
    DepthEstimator& estimator = *((DepthEstimator*)arg);
    IDX idx;
    while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize())
        estimator.ProcessPixel(idx);
    return NULL;
}
// remove all estimates with too big score and invert confidence map
void* STCALL DepthMapsData::EndDepthMapTmp(void* arg)
{
    DepthEstimator& estimator = *((DepthEstimator*)arg);
    IDX idx;
    const float fOptimAngle(FD2R(OPTDENSE::fOptimAngle));
    while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize()) {
        const ImageRef& x = estimator.coords[idx];
        ASSERT(estimator.depthMap0(x) >= 0);
        Depth& depth = estimator.depthMap0(x);
        float& conf = estimator.confMap0(x);
        // check if the score is good enough
        // and that the cross-estimates is close enough to the current estimate
        if (depth <= 0 || conf >= OPTDENSE::fNCCThresholdKeep) {
            #if 1 // used if gap-interpolation is active
            conf = 0;
            estimator.normalMap0(x) = Normal::ZERO;
            #endif
            depth = 0;
        } else {
            #if 1
            // converted ZNCC [0-2] score, where 0 is best, to [0-1] confidence, where 1 is best
            conf = conf>=1.f ? 0.f : 1.f-conf;
            #else
            #if 1
            FOREACH(i, estimator.images)
                estimator.scores[i] = ComputeAngle<REAL,float>(estimator.image0.camera.TransformPointI2W(Point3(x,depth)).ptr(), estimator.image0.camera.C.ptr(), estimator.images[i].view.camera.C.ptr());
            #if DENSE_AGGNCC == DENSE_AGGNCC_NTH
            const float fCosAngle(estimator.scores.GetNth(estimator.idxScore));
            #elif DENSE_AGGNCC == DENSE_AGGNCC_MEAN
            const float fCosAngle(estimator.scores.mean());
            #elif DENSE_AGGNCC == DENSE_AGGNCC_MIN
            const float fCosAngle(estimator.scores.minCoeff());
            #else
            const float fCosAngle(estimator.idxScore ?
                std::accumulate(estimator.scores.begin(), &estimator.scores.PartialSort(estimator.idxScore), 0.f) / estimator.idxScore :
                *std::min_element(estimator.scores.cbegin(), estimator.scores.cend()));
            #endif
            const float wAngle(MINF(POW(ACOS(fCosAngle)/fOptimAngle,1.5f),1.f));
            #else
            const float wAngle(1.f);
            #endif
            #if 1
            conf = wAngle/MAXF(conf,1e-2f);
            #else
            conf = wAngle/(depth*SQUARE(MAXF(conf,1e-2f)));
            #endif
            #endif
        }
    }
    return NULL;
}

Eigen::Vector3f getColorMap(const float& f){

    const float r[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9365079365079367f, 0.8571428571428572f, 0.7777777777777777f, 0.6984126984126986f, 0.6190476190476191f, 0.53968253968254f, 0.4603174603174605f, 0.3809523809523814f, 0.3015873015873018f, 0.2222222222222223f, 0.1428571428571432f, 0.06349206349206415f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03174603174603208f, 0.08465608465608465f, 0.1375661375661377f, 0.1904761904761907f, 0.2433862433862437f, 0.2962962962962963f, 0.3492063492063493f, 0.4021164021164023f, 0.4550264550264553f, 0.5079365079365079f, 0.5608465608465609f, 0.6137566137566139f, 0.666666666666667f};
    const float g[] = { 0, 0.03968253968253968f, 0.07936507936507936f, 0.119047619047619f, 0.1587301587301587f, 0.1984126984126984f, 0.2380952380952381f, 0.2777777777777778f, 0.3174603174603174f, 0.3571428571428571f, 0.3968253968253968f, 0.4365079365079365f, 0.4761904761904762f, 0.5158730158730158f, 0.5555555555555556f, 0.5952380952380952f, 0.6349206349206349f, 0.6746031746031745f, 0.7142857142857142f, 0.753968253968254f, 0.7936507936507936f, 0.8333333333333333f, 0.873015873015873f, 0.9126984126984127f, 0.9523809523809523f, 0.992063492063492f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9841269841269842f, 0.9047619047619047f, 0.8253968253968256f, 0.7460317460317465f, 0.666666666666667f, 0.587301587301587f, 0.5079365079365079f, 0.4285714285714288f, 0.3492063492063493f, 0.2698412698412698f, 0.1904761904761907f, 0.1111111111111116f, 0.03174603174603208f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    const float b[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01587301587301582f, 0.09523809523809534f, 0.1746031746031744f, 0.2539682539682535f, 0.333333333333333f, 0.412698412698413f, 0.4920634920634921f, 0.5714285714285712f, 0.6507936507936507f, 0.7301587301587302f, 0.8095238095238093f, 0.8888888888888884f, 0.9682539682539679f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    const int S = sizeof(r)/sizeof(r[0]);
    int idx = static_cast<int>(f*S);

    return Eigen::Vector3f(r[idx], g[idx], b[idx]);
}


// estimate depth-map using propagation and random refinement with NCC score
// as in: "Accurate Multiple View 3D Reconstruction Using Patch-Based Stereo for Large-Scale Scenes", S. Shen, 2013
// The implementations follows closely the paper, although there are some changes/additions.
// Given two views of the same scene, we note as the "reference image" the view for which a depth-map is reconstructed, and the "target image" the other view.
// As a first step, the whole depth-map is approximated by interpolating between the available sparse points.
// Next, the depth-map is passed from top/left to bottom/right corner and the opposite sens for each of the next steps.
// For each pixel, first the current depth estimate is replaced with its neighbor estimates if the NCC score is better.
// Second, the estimate is refined by trying random estimates around the current depth and normal values, keeping the one with the best score.
// The estimation can be stopped at any point, and usually 2-3 iterations are enough for convergence.
// For each pixel, the depth and normal are scored by computing the NCC score between the patch in the reference image and the wrapped patch in the target image, as dictated by the homography matrix defined by the current values to be estimate.
// In order to ensure some smoothness while locally estimating each pixel, a bonus is added to the NCC score if the estimate for this pixel is close to the estimates for the neighbor pixels.
// Optionally, the occluded pixels can be detected by extending the described iterations to the target image and removing the estimates that do not have similar values in both views.
bool DepthMapsData::EstimateDepthMap(IIndex idxImage)
{
    TD_TIMER_STARTD();

    // initialize depth and normal maps
    DepthData& depthData(arrDepthData[idxImage]);
    ASSERT(depthData.images.GetSize() > 1 && !depthData.points.IsEmpty());
    const DepthData::ViewData& image(depthData.images.First());
    ASSERT(!image.image.empty() && !depthData.images[1].image.empty());

    const Image8U::Size size(image.image.size());


    depthData.depthMap.create(size); depthData.depthMap.memset(0);
    depthData.normalMap.create(size);
    depthData.confMap.create(size);

    const unsigned nMaxThreads(scene.nMaxThreads);

    // initialize the depth-map

    if (OPTDENSE::nMinViewsTrustPoint < 2) {
        // compute depth range and initialize known depths
        VERBOSE("What are you doing here !");
            const int nPixelArea(2); // half windows size around a pixel to be initialize with the known depth
            const Camera& camera = depthData.images.First().camera;
            depthData.dMin = FLT_MAX;
            depthData.dMax = 0;
        FOREACHPTR(pPoint, depthData.points) {
            const PointCloud::Point& X = scene.pointcloud.points[*pPoint];
            const Point3 camX(camera.TransformPointW2C(Cast<REAL>(X)));
            const ImageRef x(ROUND2INT(camera.TransformPointC2I(camX)));
            const float d((float)camX.z);
            const ImageRef sx(MAXF(x.x-nPixelArea,0), MAXF(x.y-nPixelArea,0));
            const ImageRef ex(MINF(x.x+nPixelArea,size.width-1), MINF(x.y+nPixelArea,size.height-1));
            for (int y=sx.y; y<=ex.y; ++y) {
                for (int x=sx.x; x<=ex.x; ++x) {
                    depthData.depthMap(y,x) = d;
                    depthData.normalMap(y,x) = Normal::ZERO;
                }
            }
            if (depthData.dMin > d)
                depthData.dMin = d;
            if (depthData.dMax < d)
                depthData.dMax = d;
        }

        depthData.dMin *= 0.9f;
        depthData.dMax *= 1.1f;
    } else {
        // compute rough estimates using the sparse point-cloud
        InitDepthMap(depthData);
        #if TD_VERBOSE != TD_VERBOSE_OFF
        // save rough depth map as image
        if (g_nVerbosityLevel > 3) {
            ExportDepthMap(ComposeDepthFilePath(image.GetID(), "init.png"), depthData.depthMap);
            ExportNormalMap(ComposeDepthFilePath(image.GetID(), "init.normal.png"), depthData.normalMap);
            ExportPointCloud(ComposeDepthFilePath(image.GetID(), "init.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
        }
        #endif
    }

    // init integral images and index to image-ref map for the reference data
    #if DENSE_NCC == DENSE_NCC_WEIGHTED
    DepthEstimator::WeightMap weightMap0(size.area()-(size.width+1)*DepthEstimator::nSizeHalfWindow);
    #else
    Image64F imageSum0;
    cv::integral(image.image, imageSum0, CV_64F);
    #endif
    if (prevDepthMapSize != size) {
        prevDepthMapSize = size;
        BitMatrix mask;
        DepthEstimator::MapMatrix2ZigzagIdx(size, coords, mask, MAXF(64,(int)nMaxThreads*8));
    }

    // init threads
    ASSERT(nMaxThreads > 0);
    cList<DepthEstimator> estimators;
    estimators.Reserve(nMaxThreads);
    cList<SEACAVE::Thread> threads;
    if (nMaxThreads > 1)
        threads.Resize(nMaxThreads-1); // current thread is also used
    volatile Thread::safe_t idxPixel;

    // initialize the reference confidence map (NCC score map) with the score of the current estimates
    {
        // create working threads
        idxPixel = -1;
        ASSERT(estimators.IsEmpty());
        while (estimators.GetSize() < nMaxThreads)
            estimators.AddConstruct(0, depthData, idxPixel,
                #if DENSE_NCC == DENSE_NCC_WEIGHTED
                weightMap0,
                #else
                imageSum0,
                #endif
                coords);
        ASSERT(estimators.GetSize() == threads.GetSize()+1);
        FOREACH(i, threads)
            threads[i].start(ScoreDepthMapTmp, &estimators[i]);
        ScoreDepthMapTmp(&estimators.Last());
        // wait for the working threads to close
        FOREACHPTR(pThread, threads)
            pThread->join();
        estimators.Release();
        #if TD_VERBOSE != TD_VERBOSE_OFF
        // save rough depth map as image
        if (g_nVerbosityLevel > 4) {
            ExportDepthMap(ComposeDepthFilePath(image.GetID(), "rough.png"), depthData.depthMap);
            ExportNormalMap(ComposeDepthFilePath(image.GetID(), "rough.normal.png"), depthData.normalMap);
            ExportPointCloud(ComposeDepthFilePath(image.GetID(), "rough.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
        }
        #endif
    }

    // run propagation and random refinement cycles on the reference data
    for (unsigned iter=0; iter<OPTDENSE::nEstimationIters; ++iter) {
        // create working threads
        idxPixel = -1;
        ASSERT(estimators.IsEmpty());
        while (estimators.GetSize() < nMaxThreads)
            estimators.AddConstruct(iter, depthData, idxPixel,
                #if DENSE_NCC == DENSE_NCC_WEIGHTED
                weightMap0,
                #else
                imageSum0,
                #endif
                coords);
        ASSERT(estimators.GetSize() == threads.GetSize()+1);
        FOREACH(i, threads)
            threads[i].start(EstimateDepthMapTmp, &estimators[i]);
        EstimateDepthMapTmp(&estimators.Last());
        // wait for the working threads to close
        FOREACHPTR(pThread, threads)
            pThread->join();
        estimators.Release();
        #if 1 && TD_VERBOSE != TD_VERBOSE_OFF
        // save intermediate depth map as image
        if (g_nVerbosityLevel > 4) {
            const String path(ComposeDepthFilePath(image.GetID(), "iter")+String::ToString(iter));
            ExportDepthMap(path+".png", depthData.depthMap);
            ExportNormalMap(path+".normal.png", depthData.normalMap);
            ExportPointCloud(path+".ply", *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
        }
        #endif
    }

    // remove all estimates with too big score and invert confidence map
    {
        // create working threads
        idxPixel = -1;
        ASSERT(estimators.IsEmpty());
        while (estimators.GetSize() < nMaxThreads)
            estimators.AddConstruct(0, depthData, idxPixel,
                #if DENSE_NCC == DENSE_NCC_WEIGHTED
                weightMap0,
                #else
                imageSum0,
                #endif
                coords);
        ASSERT(estimators.GetSize() == threads.GetSize()+1);
        FOREACH(i, threads)
            threads[i].start(EndDepthMapTmp, &estimators[i]);
        EndDepthMapTmp(&estimators.Last());
        // wait for the working threads to close
        FOREACHPTR(pThread, threads)
            pThread->join();
        estimators.Release();
    }

    DEBUG_EXTRA("Depth-map for image %3u %s: %dx%d (%s)", image.GetID(),
        depthData.images.GetSize() > 2 ?
            String::FormatString("estimated using %2u images", depthData.images.GetSize()-1).c_str() :
            String::FormatString("with image %3u estimated", depthData.images[1].GetID()).c_str(),
        size.width, size.height, TD_TIMER_GET_FMT().c_str());
    return true;
} // EstimateDepthMap
/*----------------------------------------------------------------*/


// filter out small depth segments from the given depth map
bool DepthMapsData::RemoveSmallSegments(DepthData& depthData)
{
    const float fDepthDiffThreshold(OPTDENSE::fDepthDiffThreshold*0.7f);
    unsigned speckle_size = OPTDENSE::nSpeckleSize;
    DepthMap& depthMap = depthData.depthMap;
    NormalMap& normalMap = depthData.normalMap;
    ConfidenceMap& confMap = depthData.confMap;
    ASSERT(!depthMap.empty());
    const ImageRef size(depthMap.size());

    // allocate memory on heap for dynamic programming arrays
    TImage<bool> done_map(size, false);
    CAutoPtrArr<ImageRef> seg_list(new ImageRef[size.x*size.y]);
    unsigned seg_list_count;
    unsigned seg_list_curr;
    ImageRef neighbor[4];

    // for all pixels do
    for (int u=0; u<size.x; ++u) {
        for (int v=0; v<size.y; ++v) {
            // if the first pixel in this segment has been already processed => skip
            if (done_map(v,u))
                continue;

            // init segment list (add first element
            // and set it to be the next element to check)
            seg_list[0] = ImageRef(u,v);
            seg_list_count = 1;
            seg_list_curr  = 0;

            // add neighboring segments as long as there
            // are none-processed pixels in the seg_list;
            // none-processed means: seg_list_curr<seg_list_count
            while (seg_list_curr < seg_list_count) {
                // get address of current pixel in this segment
                const ImageRef addr_curr(seg_list[seg_list_curr]);
                const Depth& depth_curr = depthMap(addr_curr);

                if (depth_curr>0) {
                    // fill list with neighbor positions
                    neighbor[0] = ImageRef(addr_curr.x-1, addr_curr.y  );
                    neighbor[1] = ImageRef(addr_curr.x+1, addr_curr.y  );
                    neighbor[2] = ImageRef(addr_curr.x  , addr_curr.y-1);
                    neighbor[3] = ImageRef(addr_curr.x  , addr_curr.y+1);

                    // for all neighbors do
                    for (int i=0; i<4; ++i) {
                        // get neighbor pixel address
                        const ImageRef& addr_neighbor(neighbor[i]);
                        // check if neighbor is inside image
                        if (addr_neighbor.x>=0 && addr_neighbor.y>=0 && addr_neighbor.x<size.x && addr_neighbor.y<size.y) {
                            // check if neighbor has not been added yet
                            bool& done = done_map(addr_neighbor);
                            if (!done) {
                                // check if the neighbor is valid and similar to the current pixel
                                // (belonging to the current segment)
                                const Depth& depth_neighbor = depthMap(addr_neighbor);
                                if (depth_neighbor>0 && IsDepthSimilar(depth_curr, depth_neighbor, fDepthDiffThreshold)) {
                                    // add neighbor coordinates to segment list
                                    seg_list[seg_list_count++] = addr_neighbor;
                                    // set neighbor pixel in done_map to "done"
                                    // (otherwise a pixel may be added 2 times to the list, as
                                    //  neighbor of one pixel and as neighbor of another pixel)
                                    done = true;
                                }
                            }
                        }
                    }
                }

                // set current pixel in seg_list to "done"
                ++seg_list_curr;

                // set current pixel in done_map to "done"
                done_map(addr_curr) = true;
            } // end: while (seg_list_curr < seg_list_count)

            // if segment NOT large enough => invalidate pixels
            if (seg_list_count < speckle_size) {
                // for all pixels in current segment invalidate pixels
                for (unsigned i=0; i<seg_list_count; ++i) {
                    depthMap(seg_list[i]) = 0;
                    if (!normalMap.empty()) normalMap(seg_list[i]) = Normal::ZERO;
                    if (!confMap.empty()) confMap(seg_list[i]) = 0;
                }
            }
        }
    }

    return true;
} // RemoveSmallSegments
/*----------------------------------------------------------------*/

// try to fill small gaps in the depth map
bool DepthMapsData::GapInterpolation(DepthData& depthData)
{
    const float fDepthDiffThreshold(OPTDENSE::fDepthDiffThreshold*2.5f);
    unsigned nIpolGapSize = OPTDENSE::nIpolGapSize;
    DepthMap& depthMap = depthData.depthMap;
    NormalMap& normalMap = depthData.normalMap;
    ConfidenceMap& confMap = depthData.confMap;
    ASSERT(!depthMap.empty());
    const ImageRef size(depthMap.size());

    // 1. Row-wise:
    // for each row do
    for (int v=0; v<size.y; ++v) {
        // init counter
        unsigned count = 0;

        // for each element of the row do
        for (int u=0; u<size.x; ++u) {
            // get depth of this location
            const Depth& depth = depthMap(v,u);

            // if depth not valid => count and skip it
            if (depth <= 0) {
                ++count;
                continue;
            }
            if (count == 0)
                continue;

            // check if speckle is small enough
            // and value in range
            if (count <= nIpolGapSize && (unsigned)u > count) {
                // first value index for interpolation
                int u_curr(u-count);
                const int u_first(u_curr-1);
                // compute mean depth
                const Depth& depthFirst = depthMap(v,u_first);
                if (IsDepthSimilar(depthFirst, depth, fDepthDiffThreshold)) {
                    #if 0
                    // set all values with the average
                    const Depth avg((depthFirst+depth)*0.5f);
                    do {
                        depthMap(v,u_curr) = avg;
                    } while (++u_curr<u);
                    #else
                    // interpolate values
                    const Depth diff((depth-depthFirst)/(count+1));
                    Depth d(depthFirst);
                    const float c(confMap.empty() ? 0.f : MINF(confMap(v,u_first), confMap(v,u)));
                    if (normalMap.empty()) {
                        do {
                            depthMap(v,u_curr) = (d+=diff);
                            if (!confMap.empty()) confMap(v,u_curr) = c;
                        } while (++u_curr<u);
                    } else {
                        Point2f dir1, dir2;
                        Normal2Dir(normalMap(v,u_first), dir1);
                        Normal2Dir(normalMap(v,u), dir2);
                        const Point2f dirDiff((dir2-dir1)/float(count+1));
                        do {
                            depthMap(v,u_curr) = (d+=diff);
                            dir1 += dirDiff;
                            Dir2Normal(dir1, normalMap(v,u_curr));
                            if (!confMap.empty()) confMap(v,u_curr) = c;
                        } while (++u_curr<u);
                    }
                    #endif
                }
            }

            // reset counter
            count = 0;
        }
    }

    // 2. Column-wise:
    // for each column do
    for (int u=0; u<size.x; ++u) {

        // init counter
        unsigned count = 0;

        // for each element of the column do
        for (int v=0; v<size.y; ++v) {
            // get depth of this location
            const Depth& depth = depthMap(v,u);

            // if depth not valid => count and skip it
            if (depth <= 0) {
                ++count;
                continue;
            }
            if (count == 0)
                continue;

            // check if gap is small enough
            // and value in range
            if (count <= nIpolGapSize && (unsigned)v > count) {
                // first value index for interpolation
                int v_curr(v-count);
                const int v_first(v_curr-1);
                // compute mean depth
                const Depth& depthFirst = depthMap(v_first,u);
                if (IsDepthSimilar(depthFirst, depth, fDepthDiffThreshold)) {
                    #if 0
                    // set all values with the average
                    const Depth avg((depthFirst+depth)*0.5f);
                    do {
                        depthMap(v_curr,u) = avg;
                    } while (++v_curr<v);
                    #else
                    // interpolate values
                    const Depth diff((depth-depthFirst)/(count+1));
                    Depth d(depthFirst);
                    const float c(confMap.empty() ? 0.f : MINF(confMap(v_first,u), confMap(v,u)));
                    if (normalMap.empty()) {
                        do {
                            depthMap(v_curr,u) = (d+=diff);
                            if (!confMap.empty()) confMap(v_curr,u) = c;
                        } while (++v_curr<v);
                    } else {
                        Point2f dir1, dir2;
                        Normal2Dir(normalMap(v_first,u), dir1);
                        Normal2Dir(normalMap(v,u), dir2);
                        const Point2f dirDiff((dir2-dir1)/float(count+1));
                        do {
                            depthMap(v_curr,u) = (d+=diff);
                            dir1 += dirDiff;
                            Dir2Normal(dir1, normalMap(v_curr,u));
                            if (!confMap.empty()) confMap(v_curr,u) = c;
                        } while (++v_curr<v);
                    }
                    #endif
                }
            }

            // reset counter
            count = 0;
        }
    }
    return true;
} // GapInterpolation
/*----------------------------------------------------------------*/


// filter depth-map, one pixel at a time, using confidence based fusion or neighbor pixels
bool DepthMapsData::FilterDepthMap(DepthData& depthDataRef, const IIndexArr& idxNeighbors, bool bAdjust)
{
    TD_TIMER_STARTD();

    // count valid neighbor depth-maps
    ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty());
    const IIndex N = idxNeighbors.GetSize();
    ASSERT(OPTDENSE::nMinViewsFilter > 0 && scene.nCalibratedImages > 1);
    const IIndex nMinViews(MINF(OPTDENSE::nMinViewsFilter,scene.nCalibratedImages-1));
    const IIndex nMinViewsAdjust(MINF(OPTDENSE::nMinViewsFilterAdjust,scene.nCalibratedImages-1));
    if (N < nMinViews || N < nMinViewsAdjust) {
        DEBUG("error: depth map %3u can not be filtered", depthDataRef.GetView().GetID());
        return false;
    }

    // project all neighbor depth-maps to this image
    const DepthData::ViewData& imageRef = depthDataRef.images.First();
    const Image8U::Size sizeRef(depthDataRef.depthMap.size());
    const Camera& cameraRef = imageRef.camera;
    DepthMapArr depthMaps(N);
    ConfidenceMapArr confMaps(N);
    FOREACH(n, depthMaps) {
        DepthMap& depthMap = depthMaps[n];
        depthMap.create(sizeRef);
        depthMap.memset(0);
        ConfidenceMap& confMap = confMaps[n];
        if (bAdjust) {
            confMap.create(sizeRef);
            confMap.memset(0);
        }
        const IIndex idxView = depthDataRef.neighbors[idxNeighbors[(IIndex)n]].idx.ID;
        const DepthData& depthData = arrDepthData[idxView];
        const Camera& camera = depthData.images.First().camera;
        const Image8U::Size size(depthData.depthMap.size());
        for (int i=0; i<size.height; ++i) {
            for (int j=0; j<size.width; ++j) {
                const ImageRef x(j,i);
                const Depth depth(depthData.depthMap(x));
                if (depth == 0)
                    continue;
                ASSERT(depth > 0);
                const Point3 X(camera.TransformPointI2W(Point3(x.x,x.y,depth)));
                const Point3 camX(cameraRef.TransformPointW2C(X));
                if (camX.z <= 0)
                    continue;
                #if 0
                // set depth on the rounded image projection only
                const ImageRef xRef(ROUND2INT(cameraRef.TransformPointC2I(camX)));
                if (!depthMap.isInside(xRef))
                    continue;
                Depth& depthRef(depthMap(xRef));
                if (depthRef != 0 && depthRef < camX.z)
                    continue;
                depthRef = camX.z;
                if (bAdjust)
                    confMap(xRef) = depthData.confMap(x);
                #else
                // set depth on the 4 pixels around the image projection
                const Point2 imgX(cameraRef.TransformPointC2I(camX));
                const ImageRef xRefs[4] = {
                    ImageRef(FLOOR2INT(imgX.x), FLOOR2INT(imgX.y)),
                    ImageRef(FLOOR2INT(imgX.x), CEIL2INT(imgX.y)),
                    ImageRef(CEIL2INT(imgX.x), FLOOR2INT(imgX.y)),
                    ImageRef(CEIL2INT(imgX.x), CEIL2INT(imgX.y))
                };
                for (int p=0; p<4; ++p) {
                    const ImageRef& xRef = xRefs[p];
                    if (!depthMap.isInside(xRef))
                        continue;
                    Depth& depthRef(depthMap(xRef));
                    if (depthRef != 0 && depthRef < (Depth)camX.z)
                        continue;
                    depthRef = (Depth)camX.z;
                    if (bAdjust)
                        confMap(xRef) = depthData.confMap(x);
                }
                #endif
            }
        }
        #if TD_VERBOSE != TD_VERBOSE_OFF
        if (g_nVerbosityLevel > 3)
            ExportDepthMap(MAKE_PATH(String::FormatString("depthRender%04u.%04u.png", depthDataRef.GetView().GetID(), idxView)), depthMap);
        #endif
    }

    const float thDepthDiff(OPTDENSE::fDepthDiffThreshold*1.2f);
    DepthMap newDepthMap(sizeRef);
    ConfidenceMap newConfMap(sizeRef);
    #if TD_VERBOSE != TD_VERBOSE_OFF
    size_t nProcessed(0), nDiscarded(0);
    #endif
    if (bAdjust) {
        // average similar depths, and decrease confidence if depths do not agree
        // (inspired by: "Real-Time Visibility-Based Fusion of Depth Maps", Merrell, 2007)
        for (int i=0; i<sizeRef.height; ++i) {
            for (int j=0; j<sizeRef.width; ++j) {
                const ImageRef xRef(j,i);
                const Depth depth(depthDataRef.depthMap(xRef));
                if (depth == 0) {
                    newDepthMap(xRef) = 0;
                    newConfMap(xRef) = 0;
                    continue;
                }
                ASSERT(depth > 0);
                #if TD_VERBOSE != TD_VERBOSE_OFF
                ++nProcessed;
                #endif
                // update best depth and confidence estimate with all estimates
                float posConf(depthDataRef.confMap(xRef)), negConf(0);
                Depth avgDepth(depth*posConf);
                unsigned nPosViews(0), nNegViews(0);
                unsigned n(N);
                do {
                    const Depth d(depthMaps[--n](xRef));
                    if (d == 0) {
                        if (nPosViews + nNegViews + n < nMinViews)
                            goto DiscardDepth;
                        continue;
                    }
                    ASSERT(d > 0);
                    if (IsDepthSimilar(depth, d, thDepthDiff)) {
                        // average similar depths
                        const float c(confMaps[n](xRef));
                        avgDepth += d*c;
                        posConf += c;
                        ++nPosViews;
                    } else {
                        // penalize confidence
                        if (depth > d) {
                            // occlusion
                            negConf += confMaps[n](xRef);
                        } else {
                            // free-space violation
                            const DepthData& depthData = arrDepthData[depthDataRef.neighbors[idxNeighbors[n]].idx.ID];
                            const Camera& camera = depthData.images.First().camera;
                            const Point3 X(cameraRef.TransformPointI2W(Point3(xRef.x,xRef.y,depth)));
                            const ImageRef x(ROUND2INT(camera.TransformPointW2I(X)));
                            if (depthData.confMap.isInside(x)) {
                                const float c(depthData.confMap(x));
                                negConf += (c > 0 ? c : confMaps[n](xRef));
                            } else
                                negConf += confMaps[n](xRef);
                        }
                        ++nNegViews;
                    }
                } while (n);
                ASSERT(nPosViews+nNegViews >= nMinViews);
                // if enough good views and positive confidence...
                if (nPosViews >= nMinViewsAdjust && posConf > negConf && ISINSIDE(avgDepth/=posConf, depthDataRef.dMin, depthDataRef.dMax)) {
                    // consider this pixel an inlier
                    newDepthMap(xRef) = avgDepth;
                    newConfMap(xRef) = posConf - negConf;
                } else {
                    // consider this pixel an outlier
                    DiscardDepth:
                    newDepthMap(xRef) = 0;
                    newConfMap(xRef) = 0;
                    #if TD_VERBOSE != TD_VERBOSE_OFF
                    ++nDiscarded;
                    #endif
                }
            }
        }
    } else {
        // remove depth if it does not agree with enough neighbors
        const float thDepthDiffStrict(OPTDENSE::fDepthDiffThreshold*0.8f);
        const unsigned nMinGoodViewsProc(75), nMinGoodViewsDeltaProc(65);
        const unsigned nDeltas(4);
        const unsigned nMinViewsDelta(nMinViews*(nDeltas-2));
        const ImageRef xDs[nDeltas] = { ImageRef(-1,0), ImageRef(1,0), ImageRef(0,-1), ImageRef(0,1) };
        for (int i=0; i<sizeRef.height; ++i) {
            for (int j=0; j<sizeRef.width; ++j) {
                const ImageRef xRef(j,i);
                const Depth depth(depthDataRef.depthMap(xRef));
                if (depth == 0) {
                    newDepthMap(xRef) = 0;
                    newConfMap(xRef) = 0;
                    continue;
                }
                ASSERT(depth > 0);
                #if TD_VERBOSE != TD_VERBOSE_OFF
                ++nProcessed;
                #endif
                // check if very similar with the neighbors projected to this pixel
                {
                    unsigned nGoodViews(0);
                    unsigned nViews(0);
                    unsigned n(N);
                    do {
                        const Depth d(depthMaps[--n](xRef));
                        if (d > 0) {
                            // valid view
                            ++nViews;
                            if (IsDepthSimilar(depth, d, thDepthDiffStrict)) {
                                // agrees with this neighbor
                                ++nGoodViews;
                            }
                        }
                    } while (n);
                    if (nGoodViews < nMinViews || nGoodViews < nViews*nMinGoodViewsProc/100) {
                        #if TD_VERBOSE != TD_VERBOSE_OFF
                        ++nDiscarded;
                        #endif
                        newDepthMap(xRef) = 0;
                        newConfMap(xRef) = 0;
                        continue;
                    }
                }
                // check if similar with the neighbors projected around this pixel
                {
                    unsigned nGoodViews(0);
                    unsigned nViews(0);
                    for (unsigned d=0; d<nDeltas; ++d) {
                        const ImageRef xDRef(xRef+xDs[d]);
                        unsigned n(N);
                        do {
                            const Depth d(depthMaps[--n](xDRef));
                            if (d > 0) {
                                // valid view
                                ++nViews;
                                if (IsDepthSimilar(depth, d, thDepthDiff)) {
                                    // agrees with this neighbor
                                    ++nGoodViews;
                                }
                            }
                        } while (n);
                    }
                    if (nGoodViews < nMinViewsDelta || nGoodViews < nViews*nMinGoodViewsDeltaProc/100) {
                        #if TD_VERBOSE != TD_VERBOSE_OFF
                        ++nDiscarded;
                        #endif
                        newDepthMap(xRef) = 0;
                        newConfMap(xRef) = 0;
                        continue;
                    }
                }
                // enough good views, keep it
                newDepthMap(xRef) = depth;
                newConfMap(xRef) = depthDataRef.confMap(xRef);
            }
        }
    }
    if (!SaveDepthMap(ComposeDepthFilePath(imageRef.GetID(), "filtered.dmap"), newDepthMap) ||
        !SaveConfidenceMap(ComposeDepthFilePath(imageRef.GetID(), "filtered.cmap"), newConfMap))
        return false;

    #if TD_VERBOSE != TD_VERBOSE_OFF
    DEBUG("Depth map %3u filtered using %u other images: %u/%u depths discarded (%s)", imageRef.GetID(), N, nDiscarded, nProcessed, TD_TIMER_GET_FMT().c_str());
    #endif

    return true;
} // FilterDepthMap
/*----------------------------------------------------------------*/

// fuse all valid depth-maps in the same 3D point cloud;
// join points very likely to represent the same 3D point and
// filter out points blocking the view
void DepthMapsData::FuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal)
{
    TD_TIMER_STARTD();

    struct Proj {
        union {
            uint32_t idxPixel;
            struct {
                uint16_t x, y; // image pixel coordinates
            };
        };
        inline Proj() {}
        inline Proj(uint32_t _idxPixel) : idxPixel(_idxPixel) {}
        inline Proj(const ImageRef& ir) : x(ir.x), y(ir.y) {}
        inline ImageRef GetCoord() const { return ImageRef(x,y); }
    };
    typedef SEACAVE::cList<Proj,const Proj&,0,4,uint32_t> ProjArr;
    typedef SEACAVE::cList<ProjArr,const ProjArr&,1,65536> ProjsArr;

    // find best connected images
    IndexScoreArr connections(0, scene.images.GetSize());
    size_t nPointsEstimate(0);
    FOREACH(i, scene.images) {
        DepthData& depthData = arrDepthData[i];
        if (!depthData.IsValid())
            continue;
        if (depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap")) == 0)
            return;
        ASSERT(!depthData.IsEmpty());
        IndexScore& connection = connections.AddEmpty();
        connection.idx = i;
        connection.score = (float)scene.images[i].neighbors.GetSize();
        nPointsEstimate += ROUND2INT(depthData.depthMap.area()*(0.5f/*valid*/*0.3f/*new*/));
    }
    connections.Sort();

    // fuse all depth-maps, processing the best connected images first
    const unsigned nMinViewsFuse(MINF(OPTDENSE::nMinViewsFuse, scene.images.GetSize()));
    const float normalError(COS(FD2R(OPTDENSE::fNormalDiffThreshold)));
    CLISTDEF0(Depth*) invalidDepths(0, 32);
    size_t nDepths(0);
    typedef TImage<cuint32_t> DepthIndex;
    typedef cList<DepthIndex> DepthIndexArr;
    DepthIndexArr arrDepthIdx(scene.images.GetSize());
    ProjsArr projs(0, nPointsEstimate);
    pointcloud.points.Reserve(nPointsEstimate);
    pointcloud.pointViews.Reserve(nPointsEstimate);
    pointcloud.pointWeights.Reserve(nPointsEstimate);
    if (bEstimateColor)
        pointcloud.colors.Reserve(nPointsEstimate);
    if (bEstimateNormal)
        pointcloud.normals.Reserve(nPointsEstimate);
    Util::Progress progress(_T("Fused depth-maps"), connections.GetSize());
    GET_LOGCONSOLE().Pause();
    FOREACHPTR(pConnection, connections) {
        TD_TIMER_STARTD();
        const uint32_t idxImage(pConnection->idx);
        const DepthData& depthData(arrDepthData[idxImage]);
        ASSERT(!depthData.images.IsEmpty() && !depthData.neighbors.IsEmpty());
        for (const ViewScore& neighbor: depthData.neighbors) {
            DepthIndex& depthIdxs = arrDepthIdx[neighbor.idx.ID];
            if (!depthIdxs.empty())
                continue;
            const DepthData& depthDataB(arrDepthData[neighbor.idx.ID]);
            if (depthDataB.IsEmpty())
                continue;
            depthIdxs.create(depthDataB.depthMap.size());
            depthIdxs.memset((uint8_t)NO_ID);
        }
        ASSERT(!depthData.IsEmpty());
        const Image8U::Size sizeMap(depthData.depthMap.size());
        const Image& imageData = *depthData.images.First().pImageData;
        ASSERT(&imageData-scene.images.Begin() == idxImage);
        DepthIndex& depthIdxs = arrDepthIdx[idxImage];
        if (depthIdxs.empty()) {
            depthIdxs.create(Image8U::Size(imageData.width, imageData.height));
            depthIdxs.memset((uint8_t)NO_ID);
        }
        const size_t nNumPointsPrev(pointcloud.points.GetSize());
        for (int i=0; i<sizeMap.height; ++i) {
            for (int j=0; j<sizeMap.width; ++j) {
                const ImageRef x(j,i);
                const Depth depth(depthData.depthMap(x));
                if (depth == 0)
                    continue;
                ++nDepths;
                ASSERT(ISINSIDE(depth, depthData.dMin, depthData.dMax));
                uint32_t& idxPoint = depthIdxs(x);
                if (idxPoint != NO_ID)
                    continue;
                // create the corresponding 3D point
                idxPoint = (uint32_t)pointcloud.points.GetSize();
                PointCloud::Point& point = pointcloud.points.AddEmpty();
                point = imageData.camera.TransformPointI2W(Point3(Point2f(x),depth));
                PointCloud::ViewArr& views = pointcloud.pointViews.AddEmpty();
                views.Insert(idxImage);
                PointCloud::WeightArr& weights = pointcloud.pointWeights.AddEmpty();
                REAL confidence(weights.emplace_back(Conf2Weight(depthData.confMap(x),depth)));
                ProjArr& pointProjs = projs.AddEmpty();
                pointProjs.Insert(Proj(x));
                const PointCloud::Normal normal(imageData.camera.R.t()*Cast<REAL>(depthData.normalMap(x)));
                ASSERT(ISEQUAL(norm(normal), 1.f));
                // check the projection in the neighbor depth-maps
                Point3 X(point*confidence);
                Pixel32F C(Cast<float>(imageData.image(x))*confidence);
                PointCloud::Normal N(normal*confidence);
                invalidDepths.Empty();
                FOREACHPTR(pNeighbor, depthData.neighbors) {
                    const IIndex idxImageB(pNeighbor->idx.ID);
                    DepthData& depthDataB = arrDepthData[idxImageB];
                    if (depthDataB.IsEmpty())
                        continue;
                    const Image& imageDataB = scene.images[idxImageB];
                    const Point3f pt(imageDataB.camera.ProjectPointP3(point));
                    if (pt.z <= 0)
                        continue;
                    const ImageRef xB(ROUND2INT(pt.x/pt.z), ROUND2INT(pt.y/pt.z));
                    DepthMap& depthMapB = depthDataB.depthMap;
                    if (!depthMapB.isInside(xB))
                        continue;
                    Depth& depthB = depthMapB(xB);
                    if (depthB == 0)
                        continue;
                    uint32_t& idxPointB = arrDepthIdx[idxImageB](xB);
                    if (idxPointB != NO_ID)
                        continue;
                    if (IsDepthSimilar(pt.z, depthB, OPTDENSE::fDepthDiffThreshold)) {
                        // check if normals agree
                        const PointCloud::Normal normalB(imageDataB.camera.R.t()*Cast<REAL>(depthDataB.normalMap(xB)));
                        ASSERT(ISEQUAL(norm(normalB), 1.f));
                        if (normal.dot(normalB) > normalError) {
                            // add view to the 3D point
                            ASSERT(views.FindFirst(idxImageB) == PointCloud::ViewArr::NO_INDEX);
                            const float confidenceB(Conf2Weight(depthDataB.confMap(xB),depthB));
                            const IIndex idx(views.InsertSort(idxImageB));
                            weights.InsertAt(idx, confidenceB);
                            pointProjs.InsertAt(idx, Proj(xB));
                            idxPointB = idxPoint;
                            X += imageDataB.camera.TransformPointI2W(Point3(Point2f(xB),depthB))*REAL(confidenceB);
                            if (bEstimateColor)
                                C += Cast<float>(imageDataB.image(xB))*confidenceB;
                            if (bEstimateNormal)
                                N += normalB*confidenceB;
                            confidence += confidenceB;
                            continue;
                        }
                    }
                    if (pt.z < depthB) {
                        // discard depth
                        invalidDepths.Insert(&depthB);
                    }
                }
                if (views.GetSize() < nMinViewsFuse) {
                    // remove point
                    FOREACH(v, views) {
                        const IIndex idxImageB(views[v]);
                        const ImageRef x(pointProjs[v].GetCoord());
                        ASSERT(arrDepthIdx[idxImageB].isInside(x) && arrDepthIdx[idxImageB](x).idx != NO_ID);
                        arrDepthIdx[idxImageB](x).idx = NO_ID;
                    }
                    projs.RemoveLast();
                    pointcloud.pointWeights.RemoveLast();
                    pointcloud.pointViews.RemoveLast();
                    pointcloud.points.RemoveLast();
                } else {
                    // this point is valid, store it
                    const REAL nrm(REAL(1)/confidence);
                    point = X*nrm;
                    ASSERT(ISFINITE(point));
                    if (bEstimateColor)
                        pointcloud.colors.AddConstruct((C*(float)nrm).cast<uint8_t>());
                    if (bEstimateNormal)
                        pointcloud.normals.AddConstruct(normalized(N*(float)nrm));
                    // invalidate all neighbor depths that do not agree with it
                    for (Depth* pDepth: invalidDepths)
                        *pDepth = 0;
                }
            }
        }
        ASSERT(pointcloud.points.GetSize() == pointcloud.pointViews.GetSize() && pointcloud.points.GetSize() == pointcloud.pointWeights.GetSize() && pointcloud.points.GetSize() == projs.GetSize());
        DEBUG_ULTIMATE("Depths map for reference image %3u fused using %u depths maps: %u new points (%s)", idxImage, depthData.images.GetSize()-1, pointcloud.points.GetSize()-nNumPointsPrev, TD_TIMER_GET_FMT().c_str());
        progress.display(pConnection-connections.Begin());
    }
    GET_LOGCONSOLE().Play();
    progress.close();
    arrDepthIdx.Release();

    DEBUG_EXTRA("Depth-maps fused and filtered: %u depth-maps, %u depths, %u points (%d%%%%) (%s)", connections.GetSize(), nDepths, pointcloud.points.GetSize(), ROUND2INT((100.f*pointcloud.points.GetSize())/nDepths), TD_TIMER_GET_FMT().c_str());

    if (bEstimateNormal && !pointcloud.points.IsEmpty() && pointcloud.normals.IsEmpty()) {
        // estimate normal also if requested (quite expensive if normal-maps not available)
        TD_TIMER_STARTD();
        pointcloud.normals.Resize(pointcloud.points.GetSize());
        const int64_t nPoints((int64_t)pointcloud.points.GetSize());
        #ifdef DENSE_USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int64_t i=0; i<nPoints; ++i) {
            PointCloud::WeightArr& weights = pointcloud.pointWeights[i];
            ASSERT(!weights.IsEmpty());
            IIndex idxView(0);
            float bestWeight = weights.First();
            for (IIndex idx=1; idx<weights.GetSize(); ++idx) {
                const PointCloud::Weight& weight = weights[idx];
                if (bestWeight < weight) {
                    bestWeight = weight;
                    idxView = idx;
                }
            }
            const DepthData& depthData(arrDepthData[pointcloud.pointViews[i][idxView]]);
            ASSERT(depthData.IsValid() && !depthData.IsEmpty());
            depthData.GetNormal(projs[i][idxView].GetCoord(), pointcloud.normals[i]);
        }
        DEBUG_EXTRA("Normals estimated for the dense point-cloud: %u normals (%s)", pointcloud.points.GetSize(), TD_TIMER_GET_FMT().c_str());
    }

    // release all depth-maps
    FOREACHPTR(pDepthData, arrDepthData) {
        if (pDepthData->IsValid())
            pDepthData->DecRef();
    }
} // FuseDepthMaps
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

static void* DenseReconstructionEstimateTmp(void*);
static void* DenseReconstructionFilterTmp(void*);

bool Scene::DenseReconstruction()
{
    DenseDepthMapData data(*this);

    VERBOSE("nPointsTrust %u", OPTDENSE::nMinViewsTrustPoint);
    // estimate depth-maps
    if (!ComputeDepthMaps(data))
        return false;

    LidarMap lScan(data.depthMaps.arrDepthData[0].lScan);
    LidarMap nCloud(new pcl::PointCloud<pcl::XPointXYZ>);
    
    for (auto it = lScan->begin();it!=lScan->end();++it){
        if (ISEQUAL(it->z, 0.f))
            continue;
        bool valid = true;
        FOREACH(i, images) {
            const DepthData& depthData(data.depthMaps.arrDepthData[i]);
            if (depthData.IsValid() && !depthData.IsEmpty()){
                const DepthMap& curDMap(depthData.depthMap);
                const Image8U::Size sizeMap(curDMap.size());            
                FOREACH(j, depthData.images){
                    const Point3f pt(depthData.images[0].camera.ProjectPointP3(Point3d(it->x, it->y, it->z)));
                    int x = ROUND2INT(pt.x/pt.z);
                    int y = ROUND2INT(pt.y/pt.z);
                    if (0 <= x && 0 <= y && x <  depthData.images[0].image.width() && y < depthData.images[0].image.height()){
                        if (!ISEQUAL(curDMap(y,x), 0.f)){
                            valid = false;
                            break;
                        }
                    }
                }
            }
        }
        if (valid){
            nCloud->push_back(*it);
        }
    
    }
    std::cerr << nCloud->points.size() << std::endl;
    for(auto idx=0;idx<data.images.GetSize();++idx){
        DepthData& depthData(data.depthMaps.arrDepthData[idx]);
        depthData.ReleaseImages();
        depthData.Release();

    }
    
    pcl::io::savePCDFile("/home/victor/Downloads/filtered_cloud.pcd", *nCloud);


    // fuse all depth-maps
    pointcloud.Release();
    data.depthMaps.FuseDepthMaps(pointcloud, OPTDENSE::nEstimateColors == 2, OPTDENSE::nEstimateNormals == 2);
    #if TD_VERBOSE != TD_VERBOSE_OFF
    if (g_nVerbosityLevel > 2) {
        // print number of points with 3+ views
        size_t nPoints1m(0), nPoints2(0), nPoints3p(0);
        FOREACHPTR(pViews, pointcloud.pointViews) {
            switch (pViews->GetSize())
            {
            case 0:
            case 1:
                ++nPoints1m;
                break;
            case 2:
                ++nPoints2;
                break;
            default:
                ++nPoints3p;
            }
        }
        VERBOSE("Dense point-cloud composed of:\n\t%u points with 1- views\n\t%u points with 2 views\n\t%u points with 3+ views", nPoints1m, nPoints2, nPoints3p);
    }
    #endif

    if (!pointcloud.IsEmpty()) {
        if (pointcloud.colors.IsEmpty() && OPTDENSE::nEstimateColors == 1)
            EstimatePointColors(images, pointcloud);
        if (pointcloud.normals.IsEmpty() && OPTDENSE::nEstimateNormals == 1)
            EstimatePointNormals(images, pointcloud);
    }
    return true;
} // DenseReconstruction
/*----------------------------------------------------------------*/

// do first half of dense reconstruction: depth map computation
// results are saved to "data"
bool Scene::ComputeDepthMaps(DenseDepthMapData& data)
{
    {
    // maps global view indices to our list of views to be processed
    IIndexArr imagesMap;

    // prepare images for dense reconstruction (load if needed)
    {
        TD_TIMER_START();
        data.images.Reserve(images.GetSize());
        imagesMap.Resize(images.GetSize());
        #ifdef DENSE_USE_OPENMP
        bool bAbort(false);
        #pragma omp parallel for shared(data, bAbort)
        for (int_t ID=0; ID<(int_t)images.GetSize(); ++ID) {
            #pragma omp flush (bAbort)
            if (bAbort)
                continue;
            const IIndex idxImage((IIndex)ID);
        #else
        FOREACH(idxImage, images) {
        #endif
            // skip invalid, uncalibrated or discarded images
            Image& imageData = images[idxImage];
            if (!imageData.IsValid()) {
                #ifdef DENSE_USE_OPENMP
                #pragma omp critical
                #endif
                imagesMap[idxImage] = NO_ID;
                continue;
            }
            // map image index
            #ifdef DENSE_USE_OPENMP
            #pragma omp critical
            #endif
            {
                imagesMap[idxImage] = data.images.GetSize();
                data.images.Insert(idxImage);
            }
            // reload image at the appropriate resolution
            const unsigned nMaxResolution(imageData.RecomputeMaxResolution(OPTDENSE::nResolutionLevel, OPTDENSE::nMinResolution, OPTDENSE::nMaxResolution));
            if (!imageData.ReloadImage(nMaxResolution)) {
                #ifdef DENSE_USE_OPENMP
                bAbort = true;
                #pragma omp flush (bAbort)
                continue;
                #else
                return false;
                #endif
            }
            imageData.UpdateCamera(platforms);
            // print image camera
//            DEBUG_ULTIMATE("K%d = \n%s", idxImage, cvMat2String(imageData.camera.K).c_str());
//            DEBUG_LEVEL(3, "R%d = \n%s", idxImage, cvMat2String(imageData.camera.R).c_str());
//            DEBUG_LEVEL(3, "C%d = \n%s", idxImage, cvMat2String(imageData.camera.C).c_str());
        }
        #ifdef DENSE_USE_OPENMP
        if (bAbort || data.images.IsEmpty()) {
        #else
        if (data.images.IsEmpty()) {
        #endif
            VERBOSE("error: preparing images for dense reconstruction failed (errors loading images)");
            return false;
        }
        VERBOSE("Preparing images for dense reconstruction completed: %d images (%s)", images.GetSize(), TD_TIMER_GET_FMT().c_str());
    }


    // select images to be used for dense reconstruction
    {
        TD_TIMER_START();
        // for each image, find all useful neighbor views
        IIndexArr invalidIDs;
        #ifdef DENSE_USE_OPENMP
        #pragma omp parallel for shared(data, invalidIDs)
        for (int_t ID=0; ID<(int_t)data.images.GetSize(); ++ID) {
            const IIndex idx((IIndex)ID);
        #else
        FOREACH(idx, data.images) {
        #endif
            const IIndex idxImage(data.images[idx]);
            ASSERT(imagesMap[idxImage] != NO_ID);
            DepthData& depthData(data.depthMaps.arrDepthData[idxImage]);
            if (!data.depthMaps.SelectViews(depthData)) {
                #ifdef DENSE_USE_OPENMP
                #pragma omp critical
                #endif
                invalidIDs.InsertSort(idx);
            }
        }
        RFOREACH(i, invalidIDs) {
            const IIndex idx(invalidIDs[i]);
            imagesMap[data.images.Last()] = idx;
            imagesMap[data.images[idx]] = NO_ID;
            data.images.RemoveAt(idx);
        }
        // globally select a target view for each reference image
        if (OPTDENSE::nNumViews == 1 && !data.depthMaps.SelectViews(data.images, imagesMap, data.neighborsMap)) {
            VERBOSE("error: no valid images to be dense reconstructed");
            return false;
        }
        ASSERT(!data.images.IsEmpty());
        VERBOSE("Selecting images for dense reconstruction completed: %d images (%s)", data.images.GetSize(), TD_TIMER_GET_FMT().c_str());
    }
    }

    // initialize the queue of images to be processed
    data.idxImage = 0;
    ASSERT(data.events.IsEmpty());
    data.events.AddEvent(new EVTProcessImage(0));
    // start working threads
    data.progress = new Util::Progress("Estimated depth-maps", data.images.GetSize());
    GET_LOGCONSOLE().Pause();
    if (nMaxThreads > 1) {
        // multi-thread execution
        cList<SEACAVE::Thread> threads(2);
        FOREACHPTR(pThread, threads)
            pThread->start(DenseReconstructionEstimateTmp, (void*)&data);
        FOREACHPTR(pThread, threads)
            pThread->join();
    } else {
        // single-thread execution
        DenseReconstructionEstimate((void*)&data);
    }
    GET_LOGCONSOLE().Play();
    if (!data.events.IsEmpty())
        return false;
    data.progress.Release();

    if ((OPTDENSE::nOptimize & OPTDENSE::ADJUST_FILTER) != 0) {
        // initialize the queue of depth-maps to be filtered
        data.sem.Clear();
        data.idxImage = data.images.GetSize();
        ASSERT(data.events.IsEmpty());
        FOREACH(i, data.images)
            data.events.AddEvent(new EVTFilterDepthMap(i));
        // start working threads
        data.progress = new Util::Progress("Filtered depth-maps", data.images.GetSize());
        GET_LOGCONSOLE().Pause();
        if (nMaxThreads > 1) {
            // multi-thread execution
            cList<SEACAVE::Thread> threads(MINF(nMaxThreads, (unsigned)data.images.GetSize()));
            FOREACHPTR(pThread, threads)
                pThread->start(DenseReconstructionFilterTmp, (void*)&data);
            FOREACHPTR(pThread, threads)
                pThread->join();
        } else {
            // single-thread execution
            DenseReconstructionFilter((void*)&data);
        }
        GET_LOGCONSOLE().Play();
        if (!data.events.IsEmpty())
            return false;
        data.progress.Release();
    }
    return true;
} // ComputeDepthMaps
/*----------------------------------------------------------------*/

void* DenseReconstructionEstimateTmp(void* arg) {
    const DenseDepthMapData& dataThreads = *((const DenseDepthMapData*)arg);
    dataThreads.scene.DenseReconstructionEstimate(arg);
    return NULL;
}

// initialize the dense reconstruction with the sparse point cloud
void Scene::DenseReconstructionEstimate(void* pData)
{
    DenseDepthMapData& data = *((DenseDepthMapData*)pData);
    while (true) {
        CAutoPtr<Event> evt(data.events.GetEvent());
        switch (evt->GetID()) {
        case EVT_PROCESSIMAGE: {
            const EVTProcessImage& evtImage = *((EVTProcessImage*)(Event*)evt);
            if (evtImage.idxImage >= data.images.GetSize()) {
                if (nMaxThreads > 1) {
                    // close working threads
                    data.events.AddEvent(new EVTClose);
                }
                return;
            }
            // select views to reconstruct the depth-map for this image
            const IIndex idx = data.images[evtImage.idxImage];
            DepthData& depthData(data.depthMaps.arrDepthData[idx]);
            // init images pair: reference image and the best neighbor view
            ASSERT(data.neighborsMap.IsEmpty() || data.neighborsMap[evtImage.idxImage] != NO_ID);
            if (!data.depthMaps.InitViews(depthData, data.neighborsMap.IsEmpty()?NO_ID:data.neighborsMap[evtImage.idxImage], OPTDENSE::nNumViews)) {
                // process next image
                data.events.AddEvent(new EVTProcessImage((IIndex)Thread::safeInc(data.idxImage)));
                break;
            }
            VERBOSE("ADD LIDAR DATA for image %u", idx);
            depthData.lScan = lidarCloud;

            // try to load already compute depth-map for this image
            if (File::access(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"))) {
                if (OPTDENSE::nOptimize & OPTDENSE::OPTIMIZE) {
                    if (!depthData.Load(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"))) {
                        VERBOSE("error: invalid depth-map '%s'", ComposeDepthFilePath(depthData.GetView().GetID(), "dmap").c_str());
                        exit(EXIT_FAILURE);
                    }
                    // optimize depth-map
                    data.events.AddEventFirst(new EVTOptimizeDepthMap(evtImage.idxImage));
                }
                // process next image
                data.events.AddEvent(new EVTProcessImage((uint32_t)Thread::safeInc(data.idxImage)));
            } else {
                // estimate depth-map
                data.events.AddEventFirst(new EVTEstimateDepthMap(evtImage.idxImage));
            }
            break; }

        case EVT_ESTIMATEDEPTHMAP: {
            const EVTEstimateDepthMap& evtImage = *((EVTEstimateDepthMap*)(Event*)evt);
            // request next image initialization to be performed while computing this depth-map
            data.events.AddEvent(new EVTProcessImage((uint32_t)Thread::safeInc(data.idxImage)));
            // extract depth map
            data.sem.Wait();

            data.depthMaps.EstimateDepthMap(data.images[evtImage.idxImage]);
            data.sem.Signal();
            if (OPTDENSE::nOptimize & OPTDENSE::OPTIMIZE) {
                // optimize depth-map
                data.events.AddEventFirst(new EVTOptimizeDepthMap(evtImage.idxImage));
            } else {
                // save depth-map
                data.events.AddEventFirst(new EVTSaveDepthMap(evtImage.idxImage));
            }
            break; }

        case EVT_OPTIMIZEDEPTHMAP: {
            const EVTOptimizeDepthMap& evtImage = *((EVTOptimizeDepthMap*)(Event*)evt);
            const IIndex idx = data.images[evtImage.idxImage];
            DepthData& depthData(data.depthMaps.arrDepthData[idx]);
            #if TD_VERBOSE != TD_VERBOSE_OFF
            // save depth map as image
            if (g_nVerbosityLevel > 3)
                ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "raw.png"), depthData.depthMap);
            #endif
            // apply filters
            if (OPTDENSE::nOptimize & (OPTDENSE::REMOVE_SPECKLES)) {
                TD_TIMER_START();
                if (data.depthMaps.RemoveSmallSegments(depthData)) {
                    DEBUG_ULTIMATE("Depth-map %3u filtered: remove small segments (%s)", depthData.GetView().GetID(), TD_TIMER_GET_FMT().c_str());
                }
            }
            if (OPTDENSE::nOptimize & (OPTDENSE::FILL_GAPS)) {
                TD_TIMER_START();
                if (data.depthMaps.GapInterpolation(depthData)) {
                    DEBUG_ULTIMATE("Depth-map %3u filtered: gap interpolation (%s)", depthData.GetView().GetID(), TD_TIMER_GET_FMT().c_str());
                }
            }
            // save depth-map
            data.events.AddEventFirst(new EVTSaveDepthMap(evtImage.idxImage));
            break; }

        case EVT_SAVEDEPTHMAP: {
            const EVTSaveDepthMap& evtImage = *((EVTSaveDepthMap*)(Event*)evt);
            const IIndex idx = data.images[evtImage.idxImage];
            DepthData& depthData(data.depthMaps.arrDepthData[idx]);
            #if TD_VERBOSE != TD_VERBOSE_OFF
            // save depth map as image
            if (g_nVerbosityLevel > 2) {
                ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "png"), depthData.depthMap);
                ExportConfidenceMap(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.png"), depthData.confMap);
                ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
                if (g_nVerbosityLevel > 4) {
                    ExportNormalMap(ComposeDepthFilePath(depthData.GetView().GetID(), "normal.png"), depthData.normalMap);
                    depthData.confMap.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.pfm"));
                }
            }
            #endif
            // save compute depth-map for this image
            depthData.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
//            depthData.Release();
            data.progress->operator++();
            break; }

        case EVT_CLOSE: {
//            data.depthMaps.loadAllLidar();



            return; }

        default:
            ASSERT("Should not happen!" == NULL);
        }
    }

} // DenseReconstructionEstimate
/*----------------------------------------------------------------*/

void* DenseReconstructionFilterTmp(void* arg) {
    DenseDepthMapData& dataThreads = *((DenseDepthMapData*)arg);
    dataThreads.scene.DenseReconstructionFilter(arg);
    return NULL;
}

// filter estimated depth-maps
void Scene::DenseReconstructionFilter(void* pData)
{
    DenseDepthMapData& data = *((DenseDepthMapData*)pData);
    CAutoPtr<Event> evt;
    while ((evt=data.events.GetEvent(0)) != NULL) {
        switch (evt->GetID()) {
        case EVT_FILTERDEPTHMAP: {
            const EVTFilterDepthMap& evtImage = *((EVTFilterDepthMap*)(Event*)evt);
            const IIndex idx = data.images[evtImage.idxImage];
            DepthData& depthData(data.depthMaps.arrDepthData[idx]);
            if (!depthData.IsValid()) {
                data.SignalCompleteDepthmapFilter();
                break;
            }
            // make sure all depth-maps are loaded
            depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
            const unsigned numMaxNeighbors(8);
            IIndexArr idxNeighbors(0, depthData.neighbors.GetSize());
            FOREACH(n, depthData.neighbors) {
                const IIndex idxView = depthData.neighbors[n].idx.ID;
                DepthData& depthDataPair = data.depthMaps.arrDepthData[idxView];
                if (!depthDataPair.IsValid())
                    continue;
                if (depthDataPair.IncRef(ComposeDepthFilePath(depthDataPair.GetView().GetID(), "dmap")) == 0) {
                    // signal error and terminate
                    data.events.AddEventFirst(new EVTFail);
                    return;
                }
                idxNeighbors.Insert(n);
                if (idxNeighbors.GetSize() == numMaxNeighbors)
                    break;
            }
            // filter the depth-map for this image
            if (data.depthMaps.FilterDepthMap(depthData, idxNeighbors, OPTDENSE::bFilterAdjust)) {
                // load the filtered maps after all depth-maps were filtered
                data.events.AddEvent(new EVTAdjustDepthMap(evtImage.idxImage));
            }
            // unload referenced depth-maps
            FOREACHPTR(pIdxNeighbor, idxNeighbors) {
                const IIndex idxView = depthData.neighbors[*pIdxNeighbor].idx.ID;
                DepthData& depthDataPair = data.depthMaps.arrDepthData[idxView];
                depthDataPair.DecRef();
            }
            depthData.DecRef();
            data.SignalCompleteDepthmapFilter();
            break; }

        case EVT_ADJUSTDEPTHMAP: {
            const EVTAdjustDepthMap& evtImage = *((EVTAdjustDepthMap*)(Event*)evt);
            const IIndex idx = data.images[evtImage.idxImage];
            DepthData& depthData(data.depthMaps.arrDepthData[idx]);
            ASSERT(depthData.IsValid());
            data.sem.Wait();
            // load filtered maps
            if (depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap")) == 0 ||
                !LoadDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.dmap"), depthData.depthMap) ||
                !LoadConfidenceMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.cmap"), depthData.confMap))
            {
                // signal error and terminate
                data.events.AddEventFirst(new EVTFail);
                return;
            }
            ASSERT(depthData.GetRef() == 1);
            File::deleteFile(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.dmap").c_str());
            File::deleteFile(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.cmap").c_str());
            #if TD_VERBOSE != TD_VERBOSE_OFF
            // save depth map as image
            if (g_nVerbosityLevel > 2) {
                ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.png"), depthData.depthMap);
                ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
            }
            #endif
            // save filtered depth-map for this image
            depthData.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
            depthData.DecRef();
            data.progress->operator++();
            break; }

        case EVT_FAIL: {
            data.events.AddEventFirst(new EVTFail);
            return; }

        default:
            ASSERT("Should not happen!" == NULL);
        }
    }
} // DenseReconstructionFilter
/*----------------------------------------------------------------*/

// filter point-cloud based on camera-point visibility intersections
void Scene::PointCloudFilter(int thRemove)
{
    TD_TIMER_STARTD();

    typedef TOctree<PointCloud::PointArr,PointCloud::Point::Type,3,uint32_t,128> Octree;
    struct Collector {
        typedef Octree::IDX_TYPE IDX;
        typedef PointCloud::Point::Type Real;
        typedef TCone<Real,3> Cone;
        typedef TSphere<Real,3> Sphere;
        typedef TConeIntersect<Real,3> ConeIntersect;

        Cone cone;
        const ConeIntersect coneIntersect;
        const PointCloud& pointcloud;
        IntArr& visibility;
        PointCloud::Index idxPoint;
        Real distance;
        int weight;
        #ifdef DENSE_USE_OPENMP
        uint8_t pcs[sizeof(CriticalSection)];
        #endif

        Collector(const Cone::RAY& ray, Real angle, const PointCloud& _pointcloud, IntArr& _visibility)
            : cone(ray, angle), coneIntersect(cone), pointcloud(_pointcloud), visibility(_visibility)
        #ifdef DENSE_USE_OPENMP
        { new(pcs) CriticalSection; }
        ~Collector() { reinterpret_cast<CriticalSection*>(pcs)->~CriticalSection(); }
        inline CriticalSection& GetCS() { return *reinterpret_cast<CriticalSection*>(pcs); }
        #else
        {}
        #endif
        inline void Init(PointCloud::Index _idxPoint, const PointCloud::Point& X, int _weight) {
            const Real thMaxDepth(1.02f);
            idxPoint =_idxPoint;
            const PointCloud::Point::EVec D((PointCloud::Point::EVec&)X-cone.ray.m_pOrig);
            distance = D.norm();
            cone.ray.m_vDir = D/distance;
            cone.maxHeight = MaxDepthDifference(distance, thMaxDepth);
            weight = _weight;
        }
        inline bool Intersects(const Octree::POINT_TYPE& center, Octree::Type radius) const {
            return coneIntersect(Sphere(center, radius*Real(SQRT_3)));
        }
        inline void operator () (const IDX* idices, IDX size) {
            const Real thSimilar(0.01f);
            Real dist;
            FOREACHRAWPTR(pIdx, idices, size) {
                const PointCloud::Index idx(*pIdx);
                if (coneIntersect.Classify(pointcloud.points[idx], dist) == VISIBLE && !IsDepthSimilar(distance, dist, thSimilar)) {
                    if (dist > distance)
                        visibility[idx] += pointcloud.pointViews[idx].size();
                    else
                        visibility[idx] -= weight;
                }
            }
        }
    };
    typedef CLISTDEF2(Collector) Collectors;

    // create octree to speed-up search
    Octree octree(pointcloud.points);
    IntArr visibility(pointcloud.GetSize()); visibility.Memset(0);
    Collectors collectors; collectors.reserve(images.size());
    FOREACH(idxView, images) {
        const Image& image = images[idxView];
        const Ray3f ray(Cast<float>(image.camera.C), Cast<float>(image.camera.Direction()));
        const float angle(float(image.ComputeFOV(0)/image.width));
        collectors.emplace_back(ray, angle, pointcloud, visibility);
    }

    // run all camera-point visibility intersections
    Util::Progress progress(_T("Point visibility checks"), pointcloud.GetSize());
    #ifdef DENSE_USE_OPENMP
    #pragma omp parallel for //schedule(dynamic)
    for (int64_t i=0; i<(int64_t)pointcloud.GetSize(); ++i) {
        const PointCloud::Index idxPoint((PointCloud::Index)i);
    #else
    FOREACH(idxPoint, pointcloud.points) {
    #endif
        const PointCloud::Point& X = pointcloud.points[idxPoint];
        const PointCloud::ViewArr& views = pointcloud.pointViews[idxPoint];
        for (PointCloud::View idxView: views) {
            Collector& collector = collectors[idxView];
            #ifdef DENSE_USE_OPENMP
            Lock l(collector.GetCS());
            #endif
            collector.Init(idxPoint, X, (int)views.size());
            octree.Collect(collector, collector);
        }
        ++progress;
    }
    progress.close();

    #if TD_VERBOSE != TD_VERBOSE_OFF
    if (g_nVerbosityLevel > 2) {
        // print visibility stats
        UnsignedArr counts(0, 64);
        for (int views: visibility) {
            if (views > 0)
                continue;
            while (counts.size() <= IDX(-views))
                counts.push_back(0);
            ++counts[-views];
        }
        String msg;
        msg.reserve(64*counts.size());
        FOREACH(c, counts)
            if (counts[c])
                msg += String::FormatString("\n\t% 3u - % 9u", c, counts[c]);
        VERBOSE("Visibility lengths (%u points):%s", pointcloud.GetSize(), msg.c_str());
        // save outlier points
        PointCloud pc;
        RFOREACH(idxPoint, pointcloud.points) {
            if (visibility[idxPoint] <= thRemove) {
                pc.points.push_back(pointcloud.points[idxPoint]);
                pc.colors.push_back(pointcloud.colors[idxPoint]);
            }
        }
        pc.Save(MAKE_PATH("scene_dense_outliers.ply"));
    }
    #endif

    // filter points
    const size_t numInitPoints(pointcloud.GetSize());
    RFOREACH(idxPoint, pointcloud.points) {
        if (visibility[idxPoint] <= thRemove)
            pointcloud.RemovePoint(idxPoint);
    }

    DEBUG_EXTRA("Point-cloud filtered: %u/%u points (%d%%%%) (%s)", pointcloud.points.size(), numInitPoints, ROUND2INT((100.f*pointcloud.points.GetSize())/numInitPoints), TD_TIMER_GET_FMT().c_str());
} // PointCloudFilter
/*----------------------------------------------------------------*/
