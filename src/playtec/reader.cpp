#include "playtec/reader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <string>

#include "playtec/ply_utils.hpp"
#include "playtec/rapidcsv.h"

using namespace tinyply;

namespace ltr_reader
{
int readPLYFile(const std::string& filepath,
                pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                pcl::PointCloud<pcl::Normal>::Ptr normal,
                std::vector<pcl::Vertices>& pcl_vert,
                const bool preload_into_memory)
{
    std::cout << "........................................................................\n";
    std::cout << "NOW READING: " << filepath << std::endl;

    std::unique_ptr<std::istream> file_stream;
    std::vector<uint8_t> byte_buffer;

    try
    {
        // For most files < 1gb, pre-loading the entire file upfront and wrapping it into a
        // stream is a net win for parsing speed, about 40% faster.
        if (preload_into_memory)
        {
            byte_buffer = read_file_binary(filepath);
            file_stream.reset(new memory_stream((char*)byte_buffer.data(), byte_buffer.size()));
        }
        else
        {
            file_stream.reset(new std::ifstream(filepath, std::ios::binary));
        }

        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + filepath);

        file_stream->seekg(0, std::ios::end);
        const float size_mb = file_stream->tellg() * float(1e-6);
        file_stream->seekg(0, std::ios::beg);

        PlyFile file;
        file.parse_header(*file_stream);

        std::shared_ptr<PlyData> vertices, normals, faces;

        try
        {
            vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
        }
        catch (const std::exception& e)
        {
            std::cerr << "tinyply exception: " << e.what() << std::endl;
        }

        try
        {
            normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz"});
        }
        catch (const std::exception& e)
        {
            std::cerr << "tinyply exception: " << e.what() << std::endl;
        }

        try
        {
            faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
        }
        catch (const std::exception& e)
        {
            std::cerr << "tinyply exception: " << e.what() << std::endl;
        }

        manual_timer read_timer;

        read_timer.start();
        file.read(*file_stream);
        read_timer.stop();

        const float parsing_time = read_timer.get() / 1000.f;
        std::cout << "\tparsing " << size_mb << "mb in " << parsing_time << " seconds [" << (size_mb / parsing_time) << " MBps]" << std::endl;

        if (vertices) std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
        if (normals) std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
        if (faces) std::cout << "\tRead " << faces->count << " total faces (triangles) " << std::endl;

        // Call PCL object
        read_timer.start();

        const size_t numPointsBytes = vertices->buffer.size_bytes();
        const size_t numFacesBytes = faces->buffer.size_bytes();
        // Set up input cloud
        input->width = vertices->count;
        input->height = 1;
        input->points.resize(input->width * input->height);
        std::vector<float3> verts(vertices->count);
        std::memcpy(verts.data(), vertices->buffer.get(), numPointsBytes);

        // Set up normal cloud
        normal->width = normals->count;
        normal->height = 1;
        normal->points.resize(normal->width * normal->height);
        std::vector<float3> normls(vertices->count);
        std::memcpy(normls.data(), normals->buffer.get(), numPointsBytes);

        for (int idx = 0; idx < verts.size(); ++idx)
        {
            input->points[idx].x = verts[idx].x;
            input->points[idx].y = verts[idx].y;
            input->points[idx].z = verts[idx].z;
            normal->points[idx].normal_x = normls[idx].x;
            normal->points[idx].normal_y = normls[idx].y;
            normal->points[idx].normal_z = normls[idx].z;
        }

        // Set up normal cloud
        std::vector<uint3> triangles(faces->count);
        std::memcpy(triangles.data(), faces->buffer.get(), numFacesBytes);
        pcl_vert.resize(faces->count);

        for (int idx = 0; idx < triangles.size(); ++idx)
        {
            pcl_vert[idx].vertices.push_back(triangles[idx].x);
            pcl_vert[idx].vertices.push_back(triangles[idx].y);
            pcl_vert[idx].vertices.push_back(triangles[idx].z);
        }

        read_timer.stop();

        const float memory_time = read_timer.get() / 1000.f;
        std::cout << "\tReading " << size_mb << "mb in " << memory_time << " seconds [" << (size_mb / memory_time) << " MBps]" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
        return -1;
    }
    std::cout << "........................................................................\n";
    return 0;
}

int readCSVFile(const std::string& filepath,
                pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    std::cout << "........................................................................\n";
    std::cout << "\t!NOW READING!" << std::endl;
    manual_timer read_timer;
    read_timer.start();

    try
    {
        rapidcsv::Document doc(filepath, rapidcsv::LabelParams(-1, -1));
        int num_rows = doc.GetRowCount();
        int num_cols = doc.GetColumnCount();

        // Setup input pointcloud
        input->width = num_rows;
        input->height = 1;
        input->points.resize(input->width * input->height);

        std::vector<float> x_row = doc.GetColumn<float>(1);
        std::vector<float> y_row = doc.GetColumn<float>(2);
        std::vector<float> z_row = doc.GetColumn<float>(3);

        for (int idx = 0; idx < num_rows; ++idx)
        {
            input->points[idx].x = x_row[idx];
            input->points[idx].y = y_row[idx];
            input->points[idx].z = z_row[idx];
        }
        read_timer.stop();
        const float parsing_time = read_timer.get() / 1000.f;
        std::cout << "\tParsing " << num_rows << " points in " << parsing_time << " seconds" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    std::cout << "........................................................................\n";
    return 0;
}
}  // namespace ltr_reader