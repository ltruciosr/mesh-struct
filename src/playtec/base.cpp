#include "playtec/base.h"

#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>

// vtk headers
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkVersion.h>

namespace ltr_base
{
float dist2D(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    float dist = (p1.x - p2.x) * (p1.x - p2.x) +
                 (p1.y - p2.y) * (p1.x - p2.y);
    return dist;
}

void interpolatePoints(pcl::PointXYZ p, pcl::PointXYZ q, float minD_sq, std::stack<pcl::PointXYZ>& S)
{
    int number_points = ceil(sqrt(dist2D(p, q) / minD_sq));
    pcl::PointXYZ samplePoint;
    for (int i = 1; i < number_points; ++i)
    {
        samplePoint.getArray3fMap() = p.getArray3fMap() +
                                      (q.getArray3fMap() - p.getArray3fMap()) * (float(i) / float(number_points));
        S.push(samplePoint);
    }
}

void GetBaseFromBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                       float minZ, float minD)
{
    // Initialize Output border setup
    int border_num_pts = input->points.size();
    // Initialize Output surface setup
    float x = 0, y = 0;
    for (int i = 0; i < border_num_pts; i++)
    {
        input->points[i].z = minZ;
        x += input->points[i].x;
        y += input->points[i].y;
    }
    pcl::PointXYZ centroid;
    centroid.x = x / border_num_pts;
    centroid.y = y / border_num_pts;
    centroid.z = minZ;

    float minD_sq = minD * minD;
    std::stack<pcl::PointXYZ> S;
    for (pcl::PointXYZ p : input->points)
    {
        interpolatePoints(centroid, p, minD_sq, S);
    }

    // Make resample
    output->width = S.size();
    output->height = 1;
    output->points.resize(output->width * output->height);
    int count = 0;
    while (!S.empty())
    {
        output->points[count] = S.top();
        S.pop();
        ++count;
    }
}

// GET PCL FROM MESH

double uniform_deviate(int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

void randomPointTriangle(float a1, float a2, float a3,
                         float b1, float b2, float b3,
                         float c1, float c2, float c3,
                         float r1, float r2, Eigen::Vector3f& p)
{
    float r1sqr = std::sqrt(r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
}

void randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas,
                  double totalArea, Eigen::Vector3f& p)
{
    float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
    vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType* ptIds = nullptr;
    polydata->GetCellPoints(el, npts, ptIds);
    polydata->GetPoint(ptIds[0], A);
    polydata->GetPoint(ptIds[1], B);
    polydata->GetPoint(ptIds[2], C);

    float r1 = static_cast<float>(uniform_deviate(rand()));
    float r2 = static_cast<float>(uniform_deviate(rand()));

    randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
                        float(B[0]), float(B[1]), float(B[2]),
                        float(C[0]), float(C[1]), float(C[2]), r1, r2, p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples,
                      pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    polydata->BuildCells();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

    double p1[3], p2[3], p3[3];
    double totalArea = 0;
    std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
    vtkIdType npts = 0, *ptIds = nullptr;
    std::size_t cellId = 0;
    for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); cellId++)
    {
        polydata->GetPoint(ptIds[0], p1);
        polydata->GetPoint(ptIds[1], p2);
        polydata->GetPoint(ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
        cumulativeAreas[cellId] = totalArea;
    }

    cloud_out.points.resize(n_samples);
    cloud_out.width = static_cast<std::uint32_t>(n_samples);
    cloud_out.height = 1;

    for (std::size_t i = 0; i < n_samples; i++)
    {
        Eigen::Vector3f p;
        randPSurface(polydata, &cumulativeAreas, totalArea, p);
        cloud_out[i].x = p[0];
        cloud_out[i].y = p[1];
        cloud_out[i].z = p[2];
    }
}

void GetCloudFromMesh(pcl::PolygonMesh mesh,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    const int number_samples = 50000;
    const float leaf_size = 0.2f;

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh, polydata);
    //make sure that the polygons are triangles!
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polydata);
    triangleFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata = triangleMapper->GetInput();

    // fill the new pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    uniform_sampling(polydata, number_samples, *input);

    // Voxelgrid
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(input);
    voxel.filter(*output);
}
}  // namespace ltr_base