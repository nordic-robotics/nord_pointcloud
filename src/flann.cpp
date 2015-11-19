#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <nord_messages/Classification.h>
#include <nord_messages/FlannSrv.h>
#include <iostream>
#include <fstream>
#include <cassert>

// tweakables
const int num_neighbours = 3;
const int checks = 512;

typedef std::pair<std::string, std::vector<float> > vfh_model;

std::vector<std::string> names;
flann::Index<flann::ChiSquareDistance<float> >* indices;
std::vector<vfh_model> classes;

void nearest_search(std::vector<float>& histogram, flann::Matrix<int>& k_indices,
                    flann::Matrix<float>& k_distances)
{
    flann::Matrix<float> p = flann::Matrix<float>(histogram.data(), 1, 308);
    indices->knnSearch(p, k_indices, k_distances, num_neighbours,
                       flann::SearchParams(checks));
}

std::string elect(const flann::Matrix<int>& k_indices, const flann::Matrix<float>& k_distances)
{
    std::map<std::string, int> counts;
    for (size_t i = 0; i < num_neighbours; i++)
    {
        uint ix = k_indices[0][i];
        if (0 < ix && ix < names.size())
            counts[names[k_indices[0][i]]]++;
    }
    
    std::pair<std::string, int> max;
    for (std::map<std::string, int>::iterator it = counts.begin(); it != counts.end(); ++it)
    {
        if (it->second >= max.second)
            max = *it;
    }
    return max.first;
}

bool classify_shape(nord_messages::FlannSrv::Request& req,
                    nord_messages::FlannSrv::Response& res)
{
    flann::Matrix<int> k_indices(new int[num_neighbours], 1, num_neighbours);
    flann::Matrix<float> k_distances(new float[num_neighbours], 1, num_neighbours);

    std::map<std::string, int> counts;
    for (size_t i = 0; i < req.data.size(); i++)
    {
        nearest_search(req.data[i].vfh, k_indices, k_distances);
        std::string name = elect(k_indices, k_distances);
        counts[name]++;
    }

    for (std::map<std::string, int>::iterator it = counts.begin(); it != counts.end(); ++it)
    {
        std_msgs::String name;
        name.data = it->first;
        res.names.push_back(name);
        res.counts.push_back(it->second);
    }

    delete[] k_indices.ptr();
    delete[] k_distances.ptr();

    return true;
}

bool file_exists(const std::string& filename)
{
    std::ifstream infile(filename.c_str());
    return infile.good();
}

std::vector<float> load_single_pcd(const std::string& filename)
{
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type;
    unsigned int idx;
    r.readHeader(filename, cloud, origin, orientation, version, type, idx);
    int vfh_idx = pcl::getFieldIndex(cloud, "vfh");

    assert(vfh_idx != -1);
    assert(cloud.width * cloud.height == 1);

    pcl::PointCloud<pcl::VFHSignature308> point;
    pcl::io::loadPCDFile(filename, point);

    std::vector<pcl::PCLPointField> fields;
    getFieldIndex(point, "vfh", fields);

    std::vector<float> vfh;
    vfh.reserve(308);
    for (size_t i = 0; i < fields[vfh_idx].count; i++)
    {
        vfh.push_back(point.points[0].histogram[i]);
    }

    return vfh;
}

std::vector<vfh_model> load_examples(const std::vector<std::string>& names)
{
    std::vector<std::pair<std::string, std::vector<float> > > result;
    std::string folder = ros::package::getPath("nord_pointcloud") + "/data/";
    for (size_t i = 0; i < names.size(); i++)
    {
        std::string named = folder + names[i];

        int j = 0;
        while (true)
        {
            std::stringstream indexed;
            indexed << named;
            indexed << j;
            indexed << ".pcd";
            if (!file_exists(indexed.str()))
                break;

            result.push_back(std::pair<std::string, std::vector<float> >(
                names[i], load_single_pcd(indexed.str())));
            j++;
        }
    }

    return result;
}

std::vector<std::string> load_names(const std::string& filename)
{
    std::vector<std::string> results;
    std::ifstream fs;
    fs.open(filename.c_str());

    std::string line;
    while (!fs.eof())
    {
        std::getline(fs, line);
        if (line.empty())
            continue;
        results.push_back(line);
    }
    fs.close();
    return results;
}

void build()
{
    flann::Matrix<float> data(new float[classes.size() * 308], classes.size(), 308);  
    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; j++)
            data[i][j] = classes[i].second[j];

    flann::save_to_file(data, ros::package::getPath("nord_pointcloud")
                              + "/data/training_data.h5", "training_data");

    std::ofstream fs;
    fs.open((ros::package::getPath("nord_pointcloud") + "/data/training_data.list").c_str());
    for (size_t i = 0; i < classes.size (); ++i)
        fs << classes[i].first << "\n";
    fs.close();

    flann::Index<flann::ChiSquareDistance<float> > indices_out(data,
                                                               flann::LinearIndexParams());
    //flann::Index<flann::ChiSquareDistance<float> > index(data, flann::KDTreeIndexParams(4));
    indices_out.buildIndex();
    indices_out.save(ros::package::getPath("nord_pointcloud") + "/data/kdtree.idx");

    delete[] data.ptr();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "flann");
    ros::NodeHandle n;

    names.push_back("redcube");
    names.push_back("redcylinder");

    classes = load_examples(names);

    if (argc > 1 && std::string(argv[1]) == "build")
    {
        std::cout << "building..." << std::flush;
        build();
        std::cout << "done!" << std::endl;
    }

    names = load_names(ros::package::getPath("nord_pointcloud")
                       + "/data/training_data.list");

    flann::Matrix<float> data;
    flann::load_from_file(data, ros::package::getPath("nord_pointcloud")
                                + "/data/training_data.h5", "training_data");

    indices = new flann::Index<flann::ChiSquareDistance<float> >(data,
        flann::SavedIndexParams(ros::package::getPath("nord_pointcloud")
                                + "/data/kdtree.idx"));
    indices->buildIndex();

    ros::ServiceServer srv = n.advertiseService("/nord/pointcloud/shape_classification_service",
                                                classify_shape);
    ros::spin();

    return 0;
}