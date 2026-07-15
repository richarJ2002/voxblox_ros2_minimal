#ifndef VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_
#define VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/bucket_queue.h"
#include "voxblox/utils/neighbor_tools.h"
#include "voxblox/utils/timing.h"

namespace voxblox
{

/*!
 * Builds an ESDF layer out of a given TSDF layer. For a description of this
 * algorithm, please see: https://arxiv.org/abs/1611.03631
 */
class EsdfIntegrator
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Config
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
         *  @brief:     Whether to use full euclidean distance (true) or
         *              quasi-euclidean (false). Full euclidean is slightly more
         *              accurate (up to 8% in the worst case) but slower.
         */
        bool full_euclidean_distance = false;

        /*!
         *  @brief:     Maximum distance to calculate the actual distance to.
         *              Any values above this will be set to default_distance_m.
         */
        float max_distance_m = 2.0;

        /*!
         *  @brief:     Should mirror (or be smaller than) truncation distance
         *              in tsdf integrator.
         */
        float min_distance_m = 0.2;

        /*!
         *  @brief:     Default distance set for unknown values and when:
         *                  values > max_distance_m.
         */
        float default_distance_m = 2.0;

        /*!
         *  @brief:     For cheaper but less accurate map updates: the minimum
         *              difference in a voxel distance, before the change is
         *              propagated.
         */
        float min_diff_m = 0.001;

        /*!
         * @brief:      Minimum weight to consider a TSDF value seen at.
         */
        float min_weight = 1e-6;

        /*!
         * @brief:      Number of buckets for the bucketed priority queue.
         */
        int num_buckets = 20;

        /*!
         *  @brief:     Whether to push stuff to the open queue multiple
         *              times, with updated distances.
         */
        bool multi_queue = false;

        /*!
         *  @brief:     Whether to add an outside layer of occupied voxels.
         *              Basically just sets all unknown voxels in the allocated
         *              blocks to occupied. Try to only use this for batch
         *              processing, otherwise look into addNewRobotPosition
         *              below, which uses clear spheres.
         */
        bool add_occupied_crust = false;

        /*!
         *  @brief:     For marking unknown space around a robot as free or
         *              occupied, these are the radiuses used around each robot
         *              position.
         */
        float clear_sphere_radius    = 1.5;
        float occupied_sphere_radius = 5.0;
    };

    /*!
     * @brief:      TODO
     */
    EsdfIntegrator(const Config     &config,
                   Layer<TsdfVoxel> *tsdf_layer,
                   Layer<EsdfVoxel> *esdf_layer);

    /*!
     * @brief:      Used for planning - allocates sphere around as observed but
     *              occupied, and clears space in a smaller sphere around
     *              current position. Points added this way are marked as
     *              "hallucinated," and can subsequently be cleared based on
     *              this.
     */
    void addNewRobotPosition(const Point &position);

    /*!
     * @brief:      Update from a TSDF layer in batch, clearing the current ESDF
     *              layer in the process.
     */
    void updateFromTsdfLayerBatch();
    /*!
     * @brief:      Incrementally update from the TSDF layer, optionally
     *              clearing the updated flag of all changed TSDF voxels.
     */
    void updateFromTsdfLayer(bool clear_updated_flag);

    /*!
     * @brief:      Short-cut for pushing neighbors (i.e., incremental update)
     *              by default. Not necessary in batch.
     *
     * @param[in]   tsdf_blocks
     *              Reference of the tsdf blocks to be used to update the esdf.
     *
     * @param[in]   incrimental
     *              Flag to indicate whether to get voxel by linear index when
     *              tsdf_voxel.weight < config_.min_weight or to skip the block.
     */
    void updateFromTsdfBlocks(const BlockIndexList &tsdf_blocks,
                              bool                  incremental = false);

    /*!
     * @brief:      For incremental updates, the raise set contains all fixed
     *              voxels whose distances have INCREASED since last iteration.
     *              This means that all voxels that have these voxels as parents
     *              need to be invalidated and assigned new values. The raise
     *              set is always empty in batch operations.
     */
    void processRaiseSet();

    /*!
     * @brief:      The core of ESDF updates: processes a queue of voxels to get
     *              the minimum possible distance value in each voxel, by
     *              checking the values of its neighbors and distance to
     *              neighbors. The update is done once the open set is empty.
     */
    void processOpenSet();

    /*!
     * @brief:      For new voxels, etc. -- update its value from its neighbors.
     *              Sort of the inverse of what the open set does (pushes the
     *              value of voxels *TO* its neighbors). Used mostly for adding
     *              new voxels in.
     */
    bool updateVoxelFromNeighbors(const GlobalIndex &global_index);

    /*!
     * @brief:        Convenience functions.
     */
    inline bool isFixed(float dist_m) const
    {
        return std::abs(dist_m) < config_.min_distance_m;
    }

    /*!
     * @brief:      Clears the state of the integrator, in case robot pose
     *              clearance is used.
     */
    void clear()
    {
        updated_blocks_.clear();
        open_.clear();
        raise_ = AlignedQueue<GlobalIndex>();
    }

    /*!
     * @brief:      Update some specific settings.
     */
    float getEsdfMaxDistance() const
    {
        return config_.max_distance_m;
    }

    /*!
     * @brief:      TODO
     */
    void setEsdfMaxDistance(float max_distance)
    {
        config_.max_distance_m = max_distance;
        if (config_.default_distance_m < max_distance)
        {
            config_.default_distance_m = max_distance;
        }
    }

    /*!
     * @brief:      TODO
     */
    bool getFullEuclidean() const
    {
        return config_.full_euclidean_distance;
    }

    /*!
     * @brief:      TODO
     */
    void setFullEuclidean(bool full_euclidean)
    {
        config_.full_euclidean_distance = full_euclidean;
    }

  protected:
    /*!
     * @brief:      TODO
     */
    Config config_;

    /*!
     * @brief:      TODO
     */
    Layer<TsdfVoxel> *tsdf_layer_;

    /*!
     * @brief:      TODO
     */
    Layer<EsdfVoxel> *esdf_layer_;

    /*!
     * @brief:      Open Queue for incremental updates. Contains global voxel
     *              indices for the ESDF layer.
     */
    BucketQueue<GlobalIndex> open_;

    /*!
     * @brief:      Raise set for updates; these are values that used to be in
     *              the fixed frontier and now have a higher value, or their
     *              children which need to have their values invalidated.
     */
    AlignedQueue<GlobalIndex> raise_;

    /*!
     * @brief:      TODO
     */
    size_t voxels_per_side_;

    /*!
     * @brief:      TODO
     */
    float voxel_size_;

    /*!
     * @brief:      Index of memory blocks which keep track of what blocks of
     *              tsdf need to be updated.
     */
    IndexSet updated_blocks_;
};

} // namespace voxblox

#endif // VOXBLOX_INTEGRATOR_ESDF_INTEGRATOR_H_
