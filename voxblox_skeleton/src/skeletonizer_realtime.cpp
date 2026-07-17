// #include <ros/wall_timer_options.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/mesh_pcl.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

#include "voxblox_skeleton/io/skeleton_io.h"
#include "voxblox_skeleton/ros/skeleton_vis.h"
#include "voxblox_skeleton/skeleton_generator.h"

namespace voxblox
{

class SkeletonizerNode
{
  public:
    SkeletonizerNode(rclcpp::Node::SharedPtr node_ptr) :
        node_ptr_(node_ptr),
        frame_id_("map_elevated"),
        esdf_server_(node_ptr.get()),
        min_separation_angle_(0.785f),
        generate_by_layer_neighbors_(false),
        num_neighbors_for_edge_(18),
        min_gvd_distance_(0.4f),
        update_esdf_(false),
        vertex_distance_threshold_(0.8f)
    {
        skeleton_pub_ =
            node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
                std::string(node_ptr_->get_name()) + "/skeleton",
                rclcpp::QoS(1).transient_local());

        sparse_graph_pub_ =
            node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
                std::string(node_ptr_->get_name()) + "/sparse_graph",
                rclcpp::QoS(1).transient_local());
    }

    // Initialize the node.
    void init();

    // Update ESDF
    void updateEsdf();

    // Start skeleton generation
    void generateSkeleton();

    // Make a skeletor!!!
    void skeletonize(Layer<EsdfVoxel>    *esdf_layer,
                     voxblox::Pointcloud *skeleton,
                     std::vector<float>  *distances);

  private:
    rclcpp::Node::SharedPtr node_ptr_;

    std::string frame_id_;

    // ros::Publisher skeleton_pub_;
    // ros::Publisher sparse_graph_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr skeleton_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        sparse_graph_pub_;

    EsdfServer esdf_server_;

    // ros params
    float       min_separation_angle_;
    bool        generate_by_layer_neighbors_;
    int         num_neighbors_for_edge_;
    float       min_gvd_distance_;
    bool        update_esdf_;
    std::string input_filepath_, output_filepath_, sparse_graph_filepath_;
    float       vertex_distance_threshold_;

    rclcpp::TimerBase::SharedPtr skeleton_generator_timer_;
};

void SkeletonizerNode::init()
{
    skeleton_generator_timer_ = node_ptr_->create_wall_timer(
        std::chrono::duration<double>(5.0),
        std::bind(&SkeletonizerNode::generateSkeleton, this));
}

void SkeletonizerNode::generateSkeleton()
{
    // Skeletonize????
    voxblox::Pointcloud pointcloud;
    std::vector<float>  distances;
    skeletonize(esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr(),
                &pointcloud,
                &distances);

    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 ptcloud_msg;
    pcl::toROSMsg(ptcloud_pcl, ptcloud_msg);
    skeleton_pub_->publish(ptcloud_msg);
}

void SkeletonizerNode::skeletonize(Layer<EsdfVoxel>    *esdf_layer,
                                   voxblox::Pointcloud *pointcloud,
                                   std::vector<float>  *distances)
{
    SkeletonGenerator skeleton_generator;

    if (!node_ptr_->has_parameter("input_filepath"))
    {
        node_ptr_->declare_parameter("input_filepath", input_filepath_);
    }

    if (!node_ptr_->has_parameter("output_filepath"))
    {
        node_ptr_->declare_parameter("output_filepath", output_filepath_);
    }
    if (!node_ptr_->has_parameter("sparse_graph_filepath"))
    {
        node_ptr_->declare_parameter("sparse_graph_filepath",
                                     sparse_graph_filepath_);
    }
    if (!node_ptr_->has_parameter("frame_id"))
    {
        node_ptr_->declare_parameter("frame_id", frame_id_);
    }

    node_ptr_->get_parameter("input_filepath", input_filepath_);
    node_ptr_->get_parameter("output_filepath", output_filepath_);
    node_ptr_->get_parameter("sparse_graph_filepath", sparse_graph_filepath_);
    node_ptr_->get_parameter("frame_id", frame_id_);

    RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                       "Input filepath: " << input_filepath_);
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                       "Output filepath: " << output_filepath_);
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                       "Sparse graph filepath: " << sparse_graph_filepath_);
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Frame ID: " << frame_id_);

    update_esdf_ = false;

    if (!node_ptr_->has_parameter("update_esdf"))
    {
        node_ptr_->declare_parameter("update_esdf", update_esdf_);
    }
    if (!node_ptr_->has_parameter("vertex_distance_threshold"))
    {
        node_ptr_->declare_parameter("vertex_distance_threshold",
                                     vertex_distance_threshold_);
    }
    node_ptr_->get_parameter("update_esdf", update_esdf_);
    node_ptr_->get_parameter("vertex_distance_threshold",
                             vertex_distance_threshold_);

    min_separation_angle_ = skeleton_generator.getMinSeparationAngle();

    if (!node_ptr_->has_parameter("min_separation_angle"))
    {
        node_ptr_->declare_parameter("min_separation_angle",
                                     min_separation_angle_);
    }
    node_ptr_->get_parameter("min_separation_angle", min_separation_angle_);
    skeleton_generator.setMinSeparationAngle(min_separation_angle_);

    generate_by_layer_neighbors_ =
        skeleton_generator.getGenerateByLayerNeighbors();

    if (!node_ptr_->has_parameter("generate_by_layer_neighbors"))
    {
        node_ptr_->declare_parameter("generate_by_layer_neighbors",
                                     generate_by_layer_neighbors_);
    }
    node_ptr_->get_parameter("generate_by_layer_neighbors",
                             generate_by_layer_neighbors_);
    skeleton_generator.setGenerateByLayerNeighbors(
        generate_by_layer_neighbors_);

    num_neighbors_for_edge_ = skeleton_generator.getNumNeighborsForEdge();

    if (!node_ptr_->has_parameter("num_neighbors_for_edge"))
    {
        node_ptr_->declare_parameter("num_neighbors_for_edge",
                                     num_neighbors_for_edge_);
    }
    node_ptr_->get_parameter("num_neighbors_for_edge", num_neighbors_for_edge_);
    skeleton_generator.setNumNeighborsForEdge(num_neighbors_for_edge_);

    min_gvd_distance_ = skeleton_generator.getMinGvdDistance();

    if (!node_ptr_->has_parameter("min_gvd_distance"))
    {
        node_ptr_->declare_parameter("min_gvd_distance", min_gvd_distance_);
    }
    node_ptr_->get_parameter("min_gvd_distance", min_gvd_distance_);
    skeleton_generator.setMinGvdDistance(min_gvd_distance_);

    skeleton_generator.setEsdfLayer(esdf_layer);
    skeleton_generator.generateSkeleton();
    skeleton_generator.getSkeleton().getEdgePointcloudWithDistances(pointcloud,
                                                                    distances);
    RCLCPP_INFO(node_ptr_->get_logger(), "Finished generating skeleton.");

    skeleton_generator.generateSparseGraph();
    RCLCPP_INFO(node_ptr_->get_logger(), "Finished generating sparse graph.");
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(),
                       "Total Timings: " << std::endl
                                         << timing::Timing::Print());

    // Now visualize the graph.
    const SparseSkeletonGraph &graph = skeleton_generator.getSparseGraph();

    std::vector<int64_t> vertexIds;
    std::vector<int64_t> edgeIds;

    graph.getAllVertexIds(&vertexIds);

    graph.getAllEdgeIds(&edgeIds);

    const std::size_t esdfBlockCount = esdf_layer->getNumberOfAllocatedBlocks();

    RCLCPP_INFO(node_ptr_->get_logger(),
                "Skeleton update: ESDF blocks=%zu, "
                "dense edge points=%zu, "
                "sparse vertices=%zu, "
                "sparse edges=%zu, "
                "min_gvd_distance=%.3f, "
                "vertex_distance_threshold=%.3f.",
                esdfBlockCount,
                pointcloud->size(),
                vertexIds.size(),
                edgeIds.size(),
                static_cast<double>(min_gvd_distance_),
                static_cast<double>(vertex_distance_threshold_));

    visualization_msgs::msg::MarkerArray marker_array;
    visualizeSkeletonGraph(graph,
                           frame_id_,
                           &marker_array,
                           vertex_distance_threshold_);
    sparse_graph_pub_->publish(marker_array);
}

} // namespace voxblox

int main(int argc, char **argv)
{
    // Let gflags re-parse later if needed (optional)
    // gflags::AllowCommandLineReparsing();

    // Init logging first (so FLAGS_* affect glog)
    google::InitGoogleLogging("-v=1");

    // Parse only non-help flags and REMOVE recognized ones from argv
    // so the remaining argv is clean for rclcpp.
    // gflags::ParseCommandLineNonHelpFlags(&argc, &argv,
    // /*remove_flags=*/true);

    // Now ROS 2 sees only its own args (e.g., --ros-args --params-file …)
    rclcpp::init(argc, argv);

    FLAGS_alsologtostderr = true;

    auto nh = std::make_shared<rclcpp::Node>("voxblox_skeletonizer");
    voxblox::SkeletonizerNode node(nh);
    node.init();
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
