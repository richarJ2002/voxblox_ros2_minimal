#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_

#include <memory>
#include <string>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_msgs/msg/layer.hpp>

#include "voxblox_ros/tsdf_server.h"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace voxblox
{

class EsdfServer : public TsdfServer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* ---------------------------------------------------------------------- *
     * ESDF SERVER
     * ---------------------------------------------------------------------- */

    /*!
     * @brief:      TODO
     */
    EsdfServer(rclcpp::Node *node_ptr);

    /*!
     * @brief:      TODO
     */
    EsdfServer(rclcpp::Node                     *node_ptr,
               const EsdfMap::Config            &esdf_config,
               const EsdfIntegrator::Config     &esdf_integrator_config,
               const TsdfMap::Config            &tsdf_config,
               const TsdfIntegratorBase::Config &tsdf_integrator_config,
               const MeshIntegratorConfig       &mesh_config);

    /*!
     * @brief:      TODO
     */
    virtual ~EsdfServer() {}

    /* ---------------------------------------------------------------------- *
     * CALLBACKS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief:      TODO
     */
    void generateEsdfCallback(
        const std_srvs::srv::Empty::Request::SharedPtr request, // NOLINT
        std_srvs::srv::Empty::Response::SharedPtr      response);    // NOLINT

    /*!
     * @brief:      TODO
     */
    void publishAllUpdatedEsdfVoxels();

    /*!
     * @brief:      TODO
     */
    virtual void publishSlices();

    /*!
     * @brief:      TODO
     */
    void publishTraversable();

    /*!
     * @brief:      TODO
     */
    virtual void publishPointclouds();

    /*!
     * @brief:      TODO
     */
    virtual void newPoseCallback(const Transformation &T_G_C);

    /*!
     * @brief:      If a new subscriber joins, or `reset_remote_map` is set to
     *              true, then will update the ESDF map with block from the TSDF
     *              and publish it along the TSDF.
     *
     * @param[in]   reset_remonte_map
     *              Flag to indicate if the map should be reset. This overrides
     *              the functionality to reset the map by updating it if a new
     *              subscriber connects to the node.
     *
     *              In general, should be false for performance, and true for
     *              debugging.
     */
    virtual void publishMap(bool reset_remote_map = false);

    /* ---------------------------------------------------------------------- *
     * METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief:      TODO
     */
    virtual bool saveMap(const std::string &file_path);

    /*!
     * @brief:      TODO
     */
    virtual bool loadMap(const std::string &file_path);

    /*!
     * @brief:      callback function which is called on a walltimer basis.
     *              The method is called with a frequency set by:
     *                  `update_esdf_every_n_sec`
     *
     *              This method only calls the updateEsdf(). The reason this
     *              method is here is so that it is compatiable with the ros
     *              framework. (If not then its because someone was lazy and
     *              using claud code too much).
     */
    void updateEsdfEvent();

    /*!
     * @brief       Call this to update the ESDF based on latest state of the
     *              TSDF map, considering only the newly updated parts of the
     *              TSDF map (checked with the ESDF updated bit in
     *              Update::Status).
     */
    void updateEsdf();

    /*!
     * @brief:     Update the ESDF all at once; clear the existing map.
     */
    void updateEsdfBatch(bool full_euclidean = false);

    /*!
     * @brief:     Overwrites the layer with what's coming from the topic!
     */
    void esdfMapCallback(const voxblox_msgs::msg::Layer &layer_msg);

    /*!
     * @brief:      TODO
     */
    inline std::shared_ptr<EsdfMap> getEsdfMapPtr()
    {
        return esdf_map_;
    }

    /*!
     * @brief:      TODO
     *
     * TODO:        Move function declaration to `esdf_server.cc`
     */
    inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const
    {
        return esdf_map_;
    }

    /*!
     * @brief:      TODO
     */
    bool getClearSphere() const
    {
        return clear_sphere_for_planning_;
    }

    /*!
     * @brief:      TODO
     *
     * TODO:       Move function declaration to `esdf_server.c`.
     */
    void setClearSphere(bool clear_sphere_for_planning)
    {
        clear_sphere_for_planning_ = clear_sphere_for_planning;
    }

    /*!
     * @brief:      TODO
     */
    float getEsdfMaxDistance() const;

    /*!
     * @brief:      TODO
     */
    void setEsdfMaxDistance(float max_distance);

    /*!
     * @brief:      TODO
     */
    float getTraversabilityRadius() const;

    /*!
     * @brief:      TODO
     */
    void setTraversabilityRadius(float traversability_radius);

    /*!
     * @brief:      These are for enabling or disabling incremental update of
     *              the ESDF. Use carefully.
     */
    void disableIncrementalUpdate()
    {
        incremental_update_ = false;
    }

    /*!
     * @brief:     TODO
     */
    void enableIncrementalUpdate()
    {
        incremental_update_ = true;
    }

    /*!
     * @brief:     TODO
     */
    virtual void clear();

  protected:
    /*!
     * @brief:     Sets up publishing and subscribing. Should only be called
     *             from constructor.
     */
    void setupRos();

    /* ---------------------------------------------------------------------- *
     * MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief:      TODO
     */
    bool clear_sphere_for_planning_;

    /*!
     * @brief:      TODO
     */
    bool publish_esdf_map_;

    /*!
     * @brief:      TODO
     */
    bool publish_traversable_;

    /*!
     * @brief:      TODO
     */
    float traversability_radius_;

    /*!
     * @brief:      TODO
     */
    bool incremental_update_;

    /*!
     * @brief:      TODO
     */
    int num_subscribers_esdf_map_;

    /*!
     * @brief:      TODO
     */
    std::shared_ptr<EsdfMap> esdf_map_;

    /*!
     * @brief:      TODO
     */
    std::unique_ptr<EsdfIntegrator> esdf_integrator_;

    /*!
     * @brief:     Publish markers for visualization.
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        esdf_pointcloud_pub_;

    /*!
     * @brief:     TODO
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_slice_pub_;

    /*!
     * @brief:     TODO
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        traversable_pub_;

    /*!
     * @brief:     Publish the complete map for other nodes to consume.
     */
    rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_pub_;

    /*!
     * @brief:     Subscriber to subscribe to another node generating the map.
     */
    rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr esdf_map_sub_;

    /*!
     * @brief:     TODO
     */
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_esdf_srv_;

    /*!
     * @brief:     TODO
     */
    rclcpp::TimerBase::SharedPtr update_esdf_timer_;
};

} // namespace voxblox

#endif // VOXBLOX_ROS_ESDF_SERVER_H_
