/*!
 * @File:         conversions_inl.h
 *
 * @Brief:        Header file which contains the definition for the conversions
 *                used in the voxblox ros library.
 *
 * @Date:         09/07/2026
 *
 */

#ifndef VOXBLOX_ROS_CONVERSIONS_INL_H_
#define VOXBLOX_ROS_CONVERSIONS_INL_H_

#include <glog/logging.h>
#include <vector>

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <voxblox_msgs/msg/layer.hpp>

/* Generic Libraries */
/* None */

namespace voxblox
{

/*!
 * @brief           TODO
 *
 * @tparam          VoxelType
 *
 * @param           layer
 *
 * @param           serialize_only_updated_blocks
 *
 * @param           msg_out
 *
 * @param           action
 */
template <typename VoxelType>
void serializeLayerAsMsg(const Layer<VoxelType> &layer,
                         const bool              serialize_only_updated_blocks,
                         voxblox_msgs::msg::Layer     *msg_out,
                         const MapDerializationAction &action)
{
    /* Check that the message is not null */
    CHECK_NOTNULL(msg_out);

    /* Extract the number of voxels from the layer per side */
    msg_out->voxels_per_side = layer.voxels_per_side();

    /* Extract the size of each voxel within the layer */
    msg_out->voxel_size = layer.voxel_size();

    /* Extract the type of voxel in the layer */
    msg_out->layer_type = getVoxelType<VoxelType>();

    /* Extract the block list to serialize */
    BlockIndexList block_list;
    if (serialize_only_updated_blocks)
    {
        layer.getAllUpdatedBlocks(Update::kMap, &block_list);
    }
    else
    {
        layer.getAllAllocatedBlocks(&block_list);
    }

    /* Set action of list */
    msg_out->action = static_cast<uint8_t>(action);

    /* Reserve space for blocks in output msg */
    msg_out->blocks.reserve(block_list.size());

    /* For each block serialize the data and store in output msg */
    voxblox_msgs::msg::Block block_msg;
    for (const BlockIndex &index : block_list)
    {
        /* Load the index into the 'block_msg' */
        block_msg.x_index = index.x();
        block_msg.y_index = index.y();
        block_msg.z_index = index.z();

        /* Serialize the data from the index in the layer to 'data' */
        std::vector<uint32_t> data;
        layer.getBlockByIndex(index).serializeToIntegers(&data);

        /* Store data in the 'block_msg' variable */
        block_msg.data = data;

        /* Append the block msg to the output msg blocks */
        msg_out->blocks.push_back(block_msg);
    }
}

/*!
 * @brief       TODO
 *
 * @tparam      VoxelType
 *
 * @param       msg
 *
 * @param       layer
 *
 * @return      TODO
 */
template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::msg::Layer &msg,
                           Layer<VoxelType>               *layer)
{
    /* */
    CHECK_NOTNULL(layer);
    return deserializeMsgToLayer<VoxelType>(
        msg,
        static_cast<MapDerializationAction>(msg.action),
        layer);
}

/*!
 * @brief           TODO
 *
 * @tparam          VoxelType
 *
 * @param           msg
 *                  TODO
 *
 * @param           action
 *                  TODO
 *
 * @param           layer
 *                  TODO
 *
 * @return          TODO
 */
template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::msg::Layer &msg,
                           const MapDerializationAction   &action,
                           Layer<VoxelType>               *layer)
{
    /* Check that the message is not null */
    CHECK_NOTNULL(layer);

    /* Confirm that the 'msg' layer type matches VoxelType */
    if (getVoxelType<VoxelType>().compare(msg.layer_type) != 0)
    {
        return false;
    }

    /*!
     * If the number of voxels per side or the size of the voxels do not match
     * between the msg and layer then raise an error.
     *
     * The const below is the tolerance to account for floating point error.
     */
    constexpr double kVoxelSizeEpsilon = 1e-5;
    if (msg.voxels_per_side != layer->voxels_per_side() ||
        std::abs(msg.voxel_size - layer->voxel_size()) > kVoxelSizeEpsilon)
    {
        LOG(ERROR) << "Sizes don't match!";
        return false;
    }

    /* If action set, reset the layer */
    if (action == MapDerializationAction::kReset)
    {
        LOG(INFO) << "Resetting current layer.";
        layer->removeAllBlocks();
    }

    for (const voxblox_msgs::msg::Block &block_msg : msg.blocks)
    {
        BlockIndex index(block_msg.x_index,
                         block_msg.y_index,
                         block_msg.z_index);

        // Either we want to update an existing block or there was no block
        // there before.
        if (action == MapDerializationAction::kUpdate ||
            !layer->hasBlock(index))
        {
            // Create a new block if it doesn't exist yet, or get the existing
            // one at the correct block index.
            typename Block<VoxelType>::Ptr block_ptr =
                layer->allocateBlockPtrByIndex(index);

            std::vector<uint32_t> data = block_msg.data;
            block_ptr->deserializeFromIntegers(data);
        }
        else if (action == MapDerializationAction::kMerge)
        {
            typename Block<VoxelType>::Ptr old_block_ptr =
                layer->getBlockPtrByIndex(index);
            CHECK(old_block_ptr);

            typename Block<VoxelType>::Ptr new_block_ptr(
                new Block<VoxelType>(old_block_ptr->voxels_per_side(),
                                     old_block_ptr->voxel_size(),
                                     old_block_ptr->origin()));

            std::vector<uint32_t> data = block_msg.data;
            new_block_ptr->deserializeFromIntegers(data);

            old_block_ptr->mergeBlock(*new_block_ptr);
        }
    }

    switch (action)
    {
    case MapDerializationAction::kReset:
        CHECK_EQ(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
        break;
    case MapDerializationAction::kUpdate:
    // Fall through intended.
    case MapDerializationAction::kMerge:
        CHECK_GE(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
        break;
    }

    return true;
}

} // namespace voxblox

#endif /* VOXBLOX_ROS_CONVERSIONS_INL_H_ */