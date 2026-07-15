#include "voxblox_ros/esdf_server.h"

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox
{

static std::string generate_private_name(const rclcpp::Node *node_ptr,
                                         const std::string  &name)
{
    if (name.empty() || name.front() == '/' || name.front() == '~')
    {
        return name;
    }
    return std::string(node_ptr->get_name()) + "/" + name;
}

EsdfServer::EsdfServer(rclcpp::Node *node_ptr) :
    EsdfServer(node_ptr,
               getEsdfMapConfigFromRosParam(node_ptr),
               getEsdfIntegratorConfigFromRosParam(node_ptr),
               getTsdfMapConfigFromRosParam(node_ptr),
               getTsdfIntegratorConfigFromRosParam(node_ptr),
               getMeshIntegratorConfigFromRosParam(node_ptr))
{}

EsdfServer::EsdfServer(rclcpp::Node                     *node_ptr,
                       const EsdfMap::Config            &esdf_config,
                       const EsdfIntegrator::Config     &esdf_integrator_config,
                       const TsdfMap::Config            &tsdf_config,
                       const TsdfIntegratorBase::Config &tsdf_integrator_config,
                       const MeshIntegratorConfig       &mesh_config) :
    TsdfServer(node_ptr, tsdf_config, tsdf_integrator_config, mesh_config),
    clear_sphere_for_planning_(false),
    publish_esdf_map_(false),
    publish_traversable_(false),
    traversability_radius_(1.0),
    incremental_update_(true),
    num_subscribers_esdf_map_(0)
{
    // Set up map and integrator.
    esdf_map_.reset(new EsdfMap(esdf_config));
    esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                              tsdf_map_->getTsdfLayerPtr(),
                                              esdf_map_->getEsdfLayerPtr()));

    setupRos();
}

void EsdfServer::setupRos()
{
    /* ---------------------------------------------------------------------- *
     * PUBLISHERS
     * ---------------------------------------------------------------------- */

    esdf_pointcloud_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
            generate_private_name(node_ptr_, "esdf_pointcloud"),
            1);

    esdf_slice_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
            generate_private_name(node_ptr_, "esdf_slice"),
            1);

    traversable_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
            generate_private_name(node_ptr_, "traversable"),
            1);

    esdf_map_pub_ = node_ptr_->create_publisher<voxblox_msgs::msg::Layer>(
        generate_private_name(node_ptr_, "esdf_map_out"),
        1);

    /* ---------------------------------------------------------------------- *
     * SUBSCRIBERS
     * ---------------------------------------------------------------------- */

    esdf_map_sub_ = node_ptr_->create_subscription<voxblox_msgs::msg::Layer>(
        generate_private_name(node_ptr_, "esdf_map_in"),
        1,
        std::bind(&EsdfServer::esdfMapCallback, this, std::placeholders::_1));

    /* ---------------------------------------------------------------------- *
     * EXTRACT PARAMETERS
     * ---------------------------------------------------------------------- */

    /*!
     * Check Standard Parameters
     */

    if (!node_ptr_->has_parameter("clear_sphere_for_planning"))
    {
        node_ptr_->declare_parameter("clear_sphere_for_planning",
                                     clear_sphere_for_planning_);
    }

    if (!node_ptr_->has_parameter("publish_esdf_map"))
    {
        node_ptr_->declare_parameter("publish_esdf_map", publish_esdf_map_);
    }

    if (!node_ptr_->has_parameter("publish_traversable"))
    {
        node_ptr_->declare_parameter("publish_traversable",
                                     publish_traversable_);
    }

    if (!node_ptr_->has_parameter("traversability_radius"))
    {
        node_ptr_->declare_parameter("traversability_radius",
                                     traversability_radius_);
    }

    double update_esdf_every_n_sec = 1.0;
    if (!node_ptr_->has_parameter("update_esdf_every_n_sec"))
    {
        node_ptr_->declare_parameter("update_esdf_every_n_sec",
                                     update_esdf_every_n_sec);
    }

    if (update_esdf_every_n_sec > 0.0)
    {
        update_esdf_timer_ = node_ptr_->create_wall_timer(
            std::chrono::duration<double>(update_esdf_every_n_sec),
            std::bind(&EsdfServer::updateEsdfEvent, this));
    }

    if (!node_ptr_->has_parameter("publish_slices"))
    {
        node_ptr_->declare_parameter("publish_slices", publish_slices_);
    }

    /*!
     * Declared Parameters
     */

    node_ptr_->get_parameter("clear_sphere_for_planning",
                             clear_sphere_for_planning_);
    node_ptr_->get_parameter("publish_esdf_map", publish_esdf_map_);
    node_ptr_->get_parameter("publish_traversable", publish_traversable_);
    node_ptr_->get_parameter("traversability_radius", traversability_radius_);
    node_ptr_->get_parameter("update_esdf_every_n_sec",
                             update_esdf_every_n_sec);
    node_ptr_->get_parameter("publish_slices", publish_slices_);
}

void EsdfServer::publishAllUpdatedEsdfVoxels()
{
    // Create a pointcloud with distance = intensity.
    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                          &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(pointcloud, pointcloud_msg);
    esdf_pointcloud_pub_->publish(pointcloud_msg);
    // HERE
}

void EsdfServer::publishSlices()
{
    TsdfServer::publishSlices();

    pcl::PointCloud<pcl::PointXYZI> pointcloud;

    constexpr int kZAxisIndex = 2;
    createDistancePointcloudFromEsdfLayerSlice(esdf_map_->getEsdfLayer(),
                                               kZAxisIndex,
                                               slice_level_,
                                               &pointcloud);

    pointcloud.header.frame_id = world_frame_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(pointcloud, *pointcloud_msg);
    esdf_slice_pub_->publish(*pointcloud_msg);
}

void EsdfServer::updateEsdfEvent()
{
    updateEsdf();
}

void EsdfServer::publishPointclouds()
{
    /* Publish ESDF Point Cloud */
    publishAllUpdatedEsdfVoxels();

    /* Publish ESDF Slice */
    if (publish_slices_)
    {
        publishSlices();
    }

    /* Publish Traversible */
    if (publish_traversable_)
    {
        publishTraversable();
    }

    /* Publish TSDF Point Cloud */
    TsdfServer::publishPointclouds();
}

void EsdfServer::publishTraversable()
{
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                      traversability_radius_,
                                      &pointcloud);
    pointcloud.header.frame_id = world_frame_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(pointcloud, pointcloud_msg);
    traversable_pub_->publish(pointcloud_msg);
}

void EsdfServer::publishMap(bool reset_remote_map)
{
    /* This is controlled from the ros parameter launch file. */
    if (!publish_esdf_map_)
    {
        return;
    }

    /* Find the number of subscribers wanting the esdf */
    int subscribers = this->esdf_map_pub_->get_subscription_count();

    /* If no subscribers, then skips publishing to have better performance */
    if (subscribers > 0)
    {
        /*!
         * If the current number of subscribers increases, update map.
         *
         * Note:    Always reset the remote map and send all when a new
         *          subscriber subscribes. A bit of overhead for other
         *          subscribers, but better than inconsistent map states.
         */
        if (num_subscribers_esdf_map_ < subscribers)
        {
            reset_remote_map = true;
        }

        /* Start publish timer for esdf layer */
        timing::Timer publish_map_timer("map/publish_esdf");

        /* Set flag on weather to serialize only updated blocks */
        const bool serialize_only_updated_blocks = !reset_remote_map;

        /* Serialize the esdf layer */
        voxblox_msgs::msg::Layer layer_msg;
        serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                       serialize_only_updated_blocks,
                                       &layer_msg);

        /* Create msg to publish */
        if (reset_remote_map)
        {
            layer_msg.action =
                static_cast<uint8_t>(MapDerializationAction::kReset);
        }

        /* Publish layer */
        this->esdf_map_pub_->publish(layer_msg);

        /* Stop recording the time since last publish*/
        publish_map_timer.Stop();
    }

    /* Set number of subscribers so can check if they change in future */
    num_subscribers_esdf_map_ = subscribers;

    /* Publish TsdfServer map. (Works the same as ESDF) */
    TsdfServer::publishMap(true);
}

bool EsdfServer::saveMap(const std::string &file_path)
{
    // Output TSDF map first, then ESDF.
    const bool success = TsdfServer::saveMap(file_path);

    constexpr bool kClearFile = false;
    return success &&
           io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfServer::loadMap(const std::string &file_path)
{
    // Load in the same order: TSDF first, then ESDF.
    bool success = TsdfServer::loadMap(file_path);

    constexpr bool kMultipleLayerSupport = true;
    return success && io::LoadBlocksFromFile(
                          file_path,
                          Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
                          kMultipleLayerSupport,
                          esdf_map_->getEsdfLayerPtr());
}

void EsdfServer::updateEsdf()
{
    /* Check if there are block allocations waiting to udpate the map */
    if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0)
    {
        /* Set flag to indicate to clear tsdf once update complete */
        const bool clear_updated_flag_esdf = true;

        /* Update the esdf if there is updated blocks in the tsddf */
        esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
    }

    /* Publish the ESDF */
    publishMap(true);
}

void EsdfServer::updateEsdfBatch(bool full_euclidean)
{
    if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0)
    {
        esdf_integrator_->setFullEuclidean(full_euclidean);
        esdf_integrator_->updateFromTsdfLayerBatch();
    }
}

float EsdfServer::getEsdfMaxDistance() const
{
    return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfServer::setEsdfMaxDistance(float max_distance)
{
    esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfServer::getTraversabilityRadius() const
{
    return traversability_radius_;
}

void EsdfServer::setTraversabilityRadius(float traversability_radius)
{
    traversability_radius_ = traversability_radius;
}

void EsdfServer::newPoseCallback(const Transformation &T_G_C)
{
    if (clear_sphere_for_planning_)
    {
        esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
    }

    timing::Timer block_remove_timer("remove_distant_blocks");
    esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
        T_G_C.getPosition(),
        max_block_distance_from_body_);
    block_remove_timer.Stop();
}

void EsdfServer::esdfMapCallback(const voxblox_msgs::msg::Layer &layer_msg)
{
    timing::Timer receive_map_timer("map/receive_esdf");

    bool success =
        deserializeMsgToLayer<EsdfVoxel>(layer_msg,
                                         esdf_map_->getEsdfLayerPtr());

    if (!success)
    {
        RCLCPP_ERROR_THROTTLE(node_ptr_->get_logger(),
                              *node_ptr_->get_clock(),
                              10000,
                              "Got an invalid ESDF map message!");
    }
    else
    {
        RCLCPP_INFO_ONCE(node_ptr_->get_logger(),
                         "Got an ESDF map from ROS topic!");

        if (publish_pointclouds_)
        {
            publishPointclouds();
        }
    }
}

void EsdfServer::clear()
{
    esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
    esdf_integrator_->clear();
    CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

    TsdfServer::clear();

    // Publish a message to reset the map to all subscribers.
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
}

} // namespace voxblox
