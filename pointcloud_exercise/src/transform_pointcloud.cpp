#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "cnpy.h"
#include <vector>

class TransformPointCloudNode : public rclcpp::Node
{
public:
    TransformPointCloudNode() : Node("transform_pointcloud_node")
    {
        original_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/original_pointcloud_data", 10);
        transformed_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_pointcloud_data", 10);

        // Read the .npy file on initialization
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (loadPointCloudFromNpy("/path/to/pointcloud.npy", cloud)) {    // Path to Point Cloud File
            // Publish original point cloud
            publishOriginalPointCloud(cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr flat_regions(new pcl::PointCloud<pcl::PointXYZ>());

            // Process the loaded point cloud
            removeBackgroundPlane(cloud, cloud_filtered);
            identifyFlatRegions(cloud_filtered, flat_regions);

            // Convert to ROS message and publish transformed point cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*flat_regions, transformed_cloud_msg);
            transformed_cloud_msg.header.frame_id = "world";
            transformed_cloud_msg.header.stamp = this->get_clock()->now();

            transformed_pointcloud_publisher_->publish(transformed_cloud_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from .npy file.");
        }
    }

private:
    bool loadPointCloudFromNpy(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        try {
            // Load the .npy file
            cnpy::NpyArray arr = cnpy::npy_load(file_path);
            float* data = arr.data<float>();

            // Get the number of points
            size_t num_points = arr.shape[1]; 

            cloud->width = num_points;
            cloud->height = 1;
            cloud->is_dense = false;
            cloud->points.resize(num_points);

            // Populate the point cloud with x, y, and z data
            for (size_t i = 0; i < num_points; ++i) {
                cloud->points[i].x = data[i];                  // x values in the first row
                cloud->points[i].y = data[num_points + i];    // y values in the second row
                cloud->points[i].z = data[2 * num_points + i]; // z values in the third row

                if (i < 10) {  
                    RCLCPP_INFO(this->get_logger(), "Point %lu: x=%.3f, y=%.3f, z=%.3f", i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Loaded %lu points from %s", cloud->size(), file_path.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading .npy file: %s", e.what());
            return false;
        }
    }

    void publishOriginalPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        sensor_msgs::msg::PointCloud2 original_cloud_msg;
        pcl::toROSMsg(*cloud, original_cloud_msg);
        original_cloud_msg.header.frame_id = "world";
        original_cloud_msg.header.stamp = this->get_clock()->now();

        original_pointcloud_publisher_->publish(original_cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published original point cloud with %lu points.", cloud->size());
    }

    void removeBackgroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);

        // HW3-related
        //...
        // - Set the model type for detecting planes
        seg.setModelType (pcl::SACMODEL_PLANE);
        // - Set the RAMSAC method for model identification.
        seg.setMethodType (pcl::SAC_RANSAC);
        // - Set a distance threshold to group the points on the same plane.
        seg.setDistanceThreshold (0.01);
        //...

        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No plane found.");
            *cloud_out = *cloud_in;
            return;
        }

        // Extract points not part of the plane
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true); // Remove the plane
        extract.filter(*cloud_out);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Background plane removed, %lu points left.", cloud_out->size());
    }

    void identifyFlatRegions(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& flat_regions_out)
    {
        // Normal Estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());

        // HW3-related
        //...
        //  Task: Calculate normals for the point cloud.
        //  Tips:
        // - The input point cloud is cloud_in 
        ne.setInputCloud (cloud_in);
        // - You will need to use the search method "tree"
        ne.setSearchMethod (tree);
        // - You will need to set the sphere radius for calculating normals on the object surface.
        ne.setRadiusSearch (0.03);
        // - output the result to "cloud_normals"
        ne.compute (*cloud_normals);
        //...


        // Filter flat regions based on normals
        pcl::PointIndices::Ptr flat_region_indices(new pcl::PointIndices());

        
         // HW3-related
        //...
        // - Info: In your setup the camera looks down to a conveyor belt.
        // - Task: Create a for loop to the identify point cloud indices that has normals with a high "z" components and store them in flot_region_indices
        // - Tips:
        //      - cloud_normals->points.size() gives you the number of points in the pointcloud
        //      - cloud_normals->points[i].normal_z gives you the z component of the normals. This value is normalized i.e. if the surface has a normal directly pointing to the camera, than the measured z component becomes "1".
        //      - you can populate the idenfied indices using the following command: flat_region_indices->indices.push_back(static_cast<int>(i));
        //...

        // The code filters the point cloud based in the indices that you have identified above so that only the identified pointcloud remains in the "flat_regions_out" data.
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(flat_region_indices);
        extract.filter(*flat_regions_out);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Flat regions identified: %lu points.", flat_regions_out->size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pointcloud_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformPointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
