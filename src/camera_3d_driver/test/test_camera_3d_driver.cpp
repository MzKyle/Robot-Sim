#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>
#include <algorithm>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr PointMap2CloudPoint_mock(int width, int height, const std::vector<float>& data)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = data[i * 3];
        cloud->points[i].y = data[i * 3 + 1];
        cloud->points[i].z = data[i * 3 + 2];
    }
    return cloud;
}

void transformCloudToTCP_mock(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                const tf2::Transform& tcp_camera) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const tf2::Matrix3x3& R = tcp_camera.getBasis();
    const tf2::Vector3& p = tcp_camera.getOrigin();

    T.linear() <<
        R[0][0], R[0][1], R[0][2],
        R[1][0], R[1][1], R[1][2],
        R[2][0], R[2][1], R[2][2];

    T.translation() << p.x(), p.y(), p.z();

    Eigen::Matrix4d matrix = T.matrix().cast<double>();

    pcl::transformPointCloud(*cloud_in, *cloud_out, matrix);
}

bool filter_point_mock(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out, double value_min, double value_max)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(source);
    pass.setFilterFieldName(name);
    pass.setFilterLimits(value_min, value_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(*out);
    return true;
}

void filter_by_percentile_mock(std::vector<double>& in_values, std::vector<double>& out_values, 
                                double percentile_low, double percentile_high)
{
    if (percentile_low >= percentile_high) {
        return;
    }
    if (percentile_low < 0.0 || percentile_low > 1.0 || percentile_high < 0.0 || percentile_high > 1.0) {
        return;
    }

    if (in_values.empty()) {
        return;
    }

    out_values.clear();

    std::vector<double> sorted_values = in_values;
    std::sort(sorted_values.begin(), sorted_values.end());
    size_t n = sorted_values.size();
    size_t index_low = static_cast<size_t>(n * percentile_low);
    size_t index_high = static_cast<size_t>(n * percentile_high);

    if (index_low >= n) index_low = n - 1;
    if (index_high >= n) index_high = n - 1;
    double value_low = sorted_values[index_low];
    double value_high = sorted_values[index_high];

    for (const auto& v : in_values) {
        if (v >= value_low && v <= value_high) {
            out_values.push_back(v);
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB> generateChooseColoredCloud_mock(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

    colored_cloud.width = cloud.width;
    colored_cloud.height = cloud.height;
    colored_cloud.is_dense = cloud.is_dense;
    colored_cloud.points.resize(cloud.points.size());

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        colored_cloud.points[i].x = cloud.points[i].x;
        colored_cloud.points[i].y = cloud.points[i].y;
        colored_cloud.points[i].z = cloud.points[i].z;
        colored_cloud.points[i].r = 255;
    }
    return colored_cloud;
}

TEST(FilterByPercentileTest, NormalCase)
{
    std::vector<double> in_values = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    std::vector<double> out_values;
    
    filter_by_percentile_mock(in_values, out_values, 0.2, 0.8);
    
    EXPECT_GE(out_values.size(), 4);
    EXPECT_LE(out_values.size(), 8);
}

TEST(FilterByPercentileTest, EmptyInput)
{
    std::vector<double> in_values;
    std::vector<double> out_values;
    
    filter_by_percentile_mock(in_values, out_values, 0.2, 0.8);
    
    EXPECT_EQ(out_values.size(), 0);
}

TEST(FilterByPercentileTest, InvalidPercentile)
{
    std::vector<double> in_values = {1.0, 2.0, 3.0};
    std::vector<double> out_values;
    
    filter_by_percentile_mock(in_values, out_values, 0.8, 0.2);
    
    EXPECT_EQ(out_values.size(), 0);
}

TEST(FilterByPercentileTest, SingleElement)
{
    std::vector<double> in_values = {5.0};
    std::vector<double> out_values;
    
    filter_by_percentile_mock(in_values, out_values, 0.0, 1.0);
    
    EXPECT_EQ(out_values.size(), 1);
    EXPECT_DOUBLE_EQ(out_values[0], 5.0);
}

TEST(FilterPointTest, FilterByZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = 3;
    cloud->height = 1;
    cloud->points.resize(3);
    cloud->points[0] = pcl::PointXYZ(0, 0, 0);
    cloud->points[1] = pcl::PointXYZ(1, 1, 0.5);
    cloud->points[2] = pcl::PointXYZ(2, 2, 1.0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    
    bool result = filter_point_mock("z", cloud, filtered, 0.3, 0.8);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(filtered->points.size(), 1);
    EXPECT_DOUBLE_EQ(filtered->points[0].z, 0.5);
}

TEST(FilterPointTest, FilterByY)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = 3;
    cloud->height = 1;
    cloud->points.resize(3);
    cloud->points[0] = pcl::PointXYZ(0, 0, 0);
    cloud->points[1] = pcl::PointXYZ(1, 5, 0);
    cloud->points[2] = pcl::PointXYZ(2, 10, 0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    
    bool result = filter_point_mock("y", cloud, filtered, 3, 8);
    
    EXPECT_TRUE(result);
    EXPECT_EQ(filtered->points.size(), 1);
    EXPECT_DOUBLE_EQ(filtered->points[0].y, 5);
}

TEST(TransformCloudToTCPTest, IdentityTransform)
{
    std::vector<float> data = {
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f
    };
    auto cloud_in = PointMap2CloudPoint_mock(2, 1, data);
    
    tf2::Transform tcp_camera;
    tcp_camera.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tcp_camera.setRotation(q);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    transformCloudToTCP_mock(cloud_in, cloud_out, tcp_camera);
    
    EXPECT_EQ(cloud_out->points.size(), 2);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].y, 2.0);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].z, 3.0);
}

TEST(TransformCloudToTCPTest, TranslationTransform)
{
    std::vector<float> data = {
        1.0f, 2.0f, 3.0f
    };
    auto cloud_in = PointMap2CloudPoint_mock(1, 1, data);
    
    tf2::Transform tcp_camera;
    tcp_camera.setOrigin(tf2::Vector3(10.0, 20.0, 30.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tcp_camera.setRotation(q);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    transformCloudToTCP_mock(cloud_in, cloud_out, tcp_camera);
    
    EXPECT_EQ(cloud_out->points.size(), 1);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].x, 11.0);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].y, 22.0);
    EXPECT_DOUBLE_EQ(cloud_out->points[0].z, 33.0);
}

TEST(TransformCloudToTCPTest, RotationTransform)
{
    std::vector<float> data = {
        1.0f, 0.0f, 0.0f
    };
    auto cloud_in = PointMap2CloudPoint_mock(1, 1, data);
    
    tf2::Transform tcp_camera;
    tcp_camera.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI_2);
    tcp_camera.setRotation(q);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    transformCloudToTCP_mock(cloud_in, cloud_out, tcp_camera);
    
    EXPECT_EQ(cloud_out->points.size(), 1);
    EXPECT_NEAR(cloud_out->points[0].x, 0.0, 0.001);
    EXPECT_NEAR(cloud_out->points[0].y, 1.0, 0.001);
    EXPECT_NEAR(cloud_out->points[0].z, 0.0, 0.001);
}

TEST(GenerateColoredCloudTest, BasicConversion)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 2;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(2);
    cloud.points[0] = pcl::PointXYZ(1.0, 2.0, 3.0);
    cloud.points[1] = pcl::PointXYZ(4.0, 5.0, 6.0);
    
    auto colored_cloud = generateChooseColoredCloud_mock(cloud);
    
    EXPECT_EQ(colored_cloud.width, cloud.width);
    EXPECT_EQ(colored_cloud.height, cloud.height);
    EXPECT_EQ(colored_cloud.points.size(), 2);
    
    EXPECT_DOUBLE_EQ(colored_cloud.points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(colored_cloud.points[0].y, 2.0);
    EXPECT_DOUBLE_EQ(colored_cloud.points[0].z, 3.0);
    EXPECT_EQ(colored_cloud.points[0].r, 255);
    EXPECT_EQ(colored_cloud.points[0].g, 0);
    EXPECT_EQ(colored_cloud.points[0].b, 0);
    
    EXPECT_DOUBLE_EQ(colored_cloud.points[1].x, 4.0);
    EXPECT_DOUBLE_EQ(colored_cloud.points[1].y, 5.0);
    EXPECT_DOUBLE_EQ(colored_cloud.points[1].z, 6.0);
}

TEST(GenerateColoredCloudTest, EmptyCloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.is_dense = true;
    
    auto colored_cloud = generateChooseColoredCloud_mock(cloud);
    
    EXPECT_EQ(colored_cloud.points.size(), 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
