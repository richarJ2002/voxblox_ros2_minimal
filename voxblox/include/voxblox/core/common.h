#ifndef VOXBLOX_CORE_COMMON_H_
#define VOXBLOX_CORE_COMMON_H_

#include <deque>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>

namespace voxblox
{

// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedQueue = std::queue<Type, AlignedDeque<Type>>;
template <typename Type>
using AlignedStack = std::stack<Type, AlignedDeque<Type>>;
template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

template <typename Type, typename... Arguments>
inline std::shared_ptr<Type> aligned_shared(Arguments &&...arguments)
{
    typedef typename std::remove_const<Type>::type TypeNonConst;
    return std::allocate_shared<Type>(Eigen::aligned_allocator<TypeNonConst>(),
                                      std::forward<Arguments>(arguments)...);
}

// Types.
typedef int     IndexElement;
typedef int64_t LongIndexElement;

typedef Eigen::Matrix<float, 3, 1> Point;
typedef Eigen::Matrix<float, 3, 1> Ray;

typedef Eigen::Matrix<IndexElement, 3, 1> AnyIndex;
typedef AnyIndex                          VoxelIndex;
typedef AnyIndex                          BlockIndex;
typedef AnyIndex                          SignedIndex;

typedef Eigen::Matrix<LongIndexElement, 3, 1> LongIndex;
typedef LongIndex                             GlobalIndex;

typedef std::pair<BlockIndex, VoxelIndex> VoxelKey;

typedef AlignedVector<AnyIndex>  IndexVector;
typedef IndexVector              BlockIndexList;
typedef IndexVector              VoxelIndexList;
typedef AlignedVector<LongIndex> LongIndexVector;
typedef LongIndexVector          GlobalIndexVector;

struct Color;

// Pointcloud types for external interface.
typedef AlignedVector<Point> Pointcloud;
typedef AlignedVector<Color> Colors;

// For triangle meshing/vertex access.
typedef size_t                     VertexIndex;
typedef AlignedVector<VertexIndex> VertexIndexList;
typedef Eigen::Matrix<float, 3, 3> Triangle;
typedef AlignedVector<Triangle>    TriangleVector;

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<float> Rotation;
typedef kindr::minimal::RotationQuaternionTemplate<float>::Implementation
    Quaternion;

// For alignment of layers / point clouds
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> PointsMatrix;
template <size_t size> using SquareMatrix = Eigen::Matrix<float, size, size>;

// Interpolation structure
typedef Eigen::Matrix<float, 8, 8>       InterpTable;
typedef Eigen::Matrix<float, 1, 8>       InterpVector;
// Type must allow negatives:
typedef Eigen::Array<IndexElement, 3, 8> InterpIndexes;

/* -------------------------------------------------------------------------- *
 * CONSTANTS
 * -------------------------------------------------------------------------- */

/*!
 * @brief      TODO
 *
 * @note        Used for coordinates.
 */
constexpr float kEpsilon = 1e-6; /**<  */

/*!
 * @brief      TODO
 *
 * @note        Used for weights.
 */
constexpr float kFloatEpsilon = 1e-6;

/* -------------------------------------------------------------------------- *
 * STRUCT
 * -------------------------------------------------------------------------- */

struct Color
{

    /*!
     * @brief        Construcor which set the default values of the colour to 0.
     */
    Color() :
        r(0),
        g(0),
        b(0),
        a(0)
    {}

    /*!
     * @brief        Constructoer which sets the colour to input values with
     *               alpha set to max.
     */
    Color(uint8_t _r, uint8_t _g, uint8_t _b) :
        Color(_r, _g, _b, 255)
    {}

    /*!
     * @brief        Constructoer which sets the colour.
     */
    Color(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a) :
        r(_r),
        g(_g),
        b(_b),
        a(_a)
    {}

    /* ---------------------------------------------------------------------- *
     * STRUCT MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief:      8bit value which represents the colour red.
     */
    uint8_t r;

    /*!
     * @brief:      8bit value which represents the colour green.
     */
    uint8_t g;

    /*!
     * @brief:      8bit value which represents the colour blue.
     */
    uint8_t b;

    /*!
     * @brief:      8bit value which represents the alpha / opacity of the
     *              colour.
     */
    uint8_t a;

    /*!
     * @brief       This method blends two colours together returning a new
     *              colour which is the blend of both colours. The weighting's
     *              determine the mixing strength of the colours.
     *
     *              Note: Order of colours does not matter.
     *
     * @param       first_color
     *              Color struct which contains the first mixing colour.
     *
     * @param       first_weight
     *              Floating point number which represents the mixing weight
     *              of the first colour.
     *
     * @param       second_color
     *              Color struct which contains the second mixing colour.
     *
     * @param       second_weight
     *              Floating point number which represents the mixing weight
     *              of the second colour.
     *
     * @return      new_color_out
     *              Color struct with the new mixed colour.
     */
    static Color blendTwoColors(const Color &first_color,
                                float        first_weight,
                                const Color &second_color,
                                float        second_weight)
    {
        /* Find the total weight inputted */
        float total_weight = first_weight + second_weight;

        /* Normalize the weights to be percentages */
        first_weight /= total_weight;
        second_weight /= total_weight;

        /* Initiate a new colour */
        Color new_color_out;

        /* Find the new rgba values for the new colour */
        new_color_out.r = static_cast<uint8_t>(round(
            first_color.r * first_weight + second_color.r * second_weight));
        new_color_out.g = static_cast<uint8_t>(round(
            first_color.g * first_weight + second_color.g * second_weight));
        new_color_out.b = static_cast<uint8_t>(round(
            first_color.b * first_weight + second_color.b * second_weight));
        new_color_out.a = static_cast<uint8_t>(round(
            first_color.a * first_weight + second_color.a * second_weight));

        /* Return the new colour */
        return new_color_out;
    }

    /* ---------------------------------------------------------------------- *
     * DEFAULT COLOURS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Method which returns the RGB values for white colour.
     *
     * @return      Returns the Color struct with rgb values set for white.
     */
    static const Color White()
    {
        return Color(255, 255, 255);
    }

    /*!
     * @brief       Method which returns the RGB values for black colour.
     *
     * @return      Returns the Color struct with rgb values set for black.
     */
    static const Color Black()
    {
        return Color(0, 0, 0);
    }

    /*!
     * @brief       Method which returns the RGB values for grey colour.
     *
     * @return      Returns the Color struct with rgb values set for grey.
     */
    static const Color Gray()
    {
        return Color(127, 127, 127);
    }

    /*!
     * @brief       Method which returns the RGB values for red colour.
     *
     * @return      Returns the Color struct with rgb values set for red.
     */
    static const Color Red()
    {
        return Color(255, 0, 0);
    }

    /*!
     * @brief       Method which returns the RGB values for green colour.
     *
     * @return      Returns the Color struct with rgb values set for green.
     */
    static const Color Green()
    {
        return Color(0, 255, 0);
    }

    /*!
     * @brief       Method which returns the RGB values for blue colour.
     *
     * @return      Returns the Color struct with rgb values set for blue.
     */
    static const Color Blue()
    {
        return Color(0, 0, 255);
    }

    /*!
     * @brief       Method which returns the RGB values for yellow colour.
     *
     * @return      Returns the Color struct with rgb values set for yellow.
     */
    static const Color Yellow()
    {
        return Color(255, 255, 0);
    }

    /*!
     * @brief       Method which returns the RGB values for orange colour.
     *
     * @return      Returns the Color struct with rgb values set for orange.
     */
    static const Color Orange()
    {
        return Color(255, 127, 0);
    }

    /*!
     * @brief       Method which returns the RGB values for purple colour.
     *
     * @return      Returns the Color struct with rgb values set for purple.
     */
    static const Color Purple()
    {
        return Color(127, 0, 255);
    }

    /*!
     * @brief       Method which returns the RGB values for teal colour.
     *
     * @return      Returns the Color struct with rgb values set for teal.
     */
    static const Color Teal()
    {
        return Color(0, 255, 255);
    }

    /*!
     * @brief       Method which returns the RGB values for pink colour.
     *
     * @return      Returns the Color struct with rgb values set for pink.
     */
    static const Color Pink()
    {
        return Color(255, 0, 127);
    }
};

/* -------------------------------------------------------------------------- *
 * GRID <-> POINT CONVERSIN FUNCTIONS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       TODO
 *
 * @note        Due the limited accuracy of the float type, this function
 *              doesn't always compute the correct grid index for coordinates
 *              near the grid cell boundaries. Use the safer
 *              `getGridIndexFromOriginPoint` if the origin point is available.
 */
template <typename IndexType>
inline IndexType getGridIndexFromPoint(const Point &point,
                                       const float  grid_size_inv)
{
    return IndexType(std::floor(point.x() * grid_size_inv + kEpsilon),
                     std::floor(point.y() * grid_size_inv + kEpsilon),
                     std::floor(point.z() * grid_size_inv + kEpsilon));
}

/*!
 * @brief       TODO
 *
 * @note        Due the limited accuracy of the float type, this function
 *              doesn't always compute the correct grid index for coordinates
 *              near the grid cell boundaries.
 */
template <typename IndexType>
inline IndexType getGridIndexFromPoint(const Point &scaled_point)
{
    return IndexType(std::floor(scaled_point.x() + kEpsilon),
                     std::floor(scaled_point.y() + kEpsilon),
                     std::floor(scaled_point.z() + kEpsilon));
}

/**
 * NOTE:
 */
/*!
 * @brief       TODO
 *
 * @note        This function is safer than `getGridIndexFromPoint`, because it
 *              assumes we pass in not an arbitrary point in the grid cell, but
 *              the ORIGIN. This way we can avoid the floating point precision
 *              issue that arrises for calls to `getGridIndexFromPoint`for
 *              arbitrary points near the border of the grid cell.
 */
template <typename IndexType>
inline IndexType getGridIndexFromOriginPoint(const Point &point,
                                             const float  grid_size_inv)
{
    return IndexType(std::round(point.x() * grid_size_inv),
                     std::round(point.y() * grid_size_inv),
                     std::round(point.z() * grid_size_inv));
}

/*!
 * @brief       TODO
 */
template <typename IndexType>
inline Point getCenterPointFromGridIndex(const IndexType &idx, float grid_size)
{
    return Point((static_cast<float>(idx.x()) + 0.5) * grid_size,
                 (static_cast<float>(idx.y()) + 0.5) * grid_size,
                 (static_cast<float>(idx.z()) + 0.5) * grid_size);
}

/*!
 * @brief       TODO
 *
 * @param       idx
 *              TODO
 *
 * @param       grid_size
 *              TODO
 */
template <typename IndexType>
inline Point getOriginPointFromGridIndex(const IndexType &idx, float grid_size)
{
    return Point(static_cast<float>(idx.x()) * grid_size,
                 static_cast<float>(idx.y()) * grid_size,
                 static_cast<float>(idx.z()) * grid_size);
}

/*!
 * @brief       Converts between Block + Voxel index and GlobalVoxelIndex. Note
 *              that this takes int VOXELS_PER_SIDE, and
 *              getBlockIndexFromGlobalVoxelIndex takes voxels per side inverse.
 *
 * @param       block_index
 *              TODO
 *
 * @param       voxel_index
 *              TODO
 *
 * @param       voxels_per_side
 *              TODO
 *
 */
inline GlobalIndex
    getGlobalVoxelIndexFromBlockAndVoxelIndex(const BlockIndex &block_index,
                                              const VoxelIndex &voxel_index,
                                              int               voxels_per_side)
{
    return GlobalIndex(block_index.cast<LongIndexElement>() * voxels_per_side +
                       voxel_index.cast<LongIndexElement>());
}

/*!
 * @brief       TODO
 *
 * @param       global_voxel_idx
 *              TODO
 *
 * @param       voxels_per_side_inv
 *              TODO
 */
inline BlockIndex
    getBlockIndexFromGlobalVoxelIndex(const GlobalIndex &global_voxel_idx,
                                      float              voxels_per_side_inv)
{
    return BlockIndex(std::floor(static_cast<float>(global_voxel_idx.x()) *
                                 voxels_per_side_inv),
                      std::floor(static_cast<float>(global_voxel_idx.y()) *
                                 voxels_per_side_inv),
                      std::floor(static_cast<float>(global_voxel_idx.z()) *
                                 voxels_per_side_inv));
}

/*!
 * @brief       TODO
 *
 * @param       x
 *              TODO
 */
inline bool isPowerOfTwo(int x)
{
    return (x & (x - 1)) == 0;
}

/**
 *
 * NOTE:
 */

/*!
 * @brief       Converts from a global voxel index to the index inside a block.
 *
 * @note        Assumes that voxels_per_side is a power of 2 and uses a bitwise
 *              and as a computationally cheap substitute for the modulus
 *              operator.
 *
 * @param       global_voxel_idx
 *              TODO
 *
 * @param       voxels_per_side
 *              TODO
 */
inline VoxelIndex
    getLocalFromGlobalVoxelIndex(const GlobalIndex &global_voxel_idx,
                                 const int          voxels_per_side)
{
    // add a big number to the index to make it positive
    constexpr int offset = 1 << (8 * sizeof(IndexElement) - 1);

    CHECK(isPowerOfTwo(voxels_per_side));

    return VoxelIndex((global_voxel_idx.x() + offset) & (voxels_per_side - 1),
                      (global_voxel_idx.y() + offset) & (voxels_per_side - 1),
                      (global_voxel_idx.z() + offset) & (voxels_per_side - 1));
}

/*!
 * @brief       TODO
 *
 * @param       global_voxel_idx
 *              TODO
 *
 * @param       voxels_per_side
 *              TODO
 *
 * @param       block_index
 *              TODO
 *
 * @param       voxel_index
 *              TODO
 */
inline void getBlockAndVoxelIndexFromGlobalVoxelIndex(
    const GlobalIndex &global_voxel_idx,
    const int          voxels_per_side,
    BlockIndex        *block_index,
    VoxelIndex        *voxel_index)
{
    CHECK_NOTNULL(block_index);
    CHECK_NOTNULL(voxel_index);
    const float voxels_per_side_inv = 1.0 / voxels_per_side;
    *block_index = getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                     voxels_per_side_inv);
    *voxel_index =
        getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side);
}

/* -------------------------------------------------------------------------- *
 * MATH FUNCTIONS
 * -------------------------------------------------------------------------- */

/*!
 * @brief       TODO
 *
 * @param       x
 *              TODO
 *
 * @return      TODO
 */
inline int signum(float x)
{
    return (x == 0) ? 0 : x < 0 ? -1 : 1;
}

// For occupancy/octomap-style mapping.

/*!
 * @brief       TODO
 *
 * @param       probability
 *              TODO
 *
 * @return      TODO
 */
inline float logOddsFromProbability(float probability)
{
    CHECK(probability >= 0.0f && probability <= 1.0f);
    return log(probability / (1.0 - probability));
}

/*!
 * @brief       TODO
 *
 * @param       log_odds
 *              TODO
 *
 * @return      TODO
 */
inline float probabilityFromLogOdds(float log_odds)
{
    return 1.0 - (1.0 / (1.0 + exp(log_odds)));
}

/*!
 * @brief       TODO
 *
 * @param       T_N_O
 *              TODO
 *
 * @param       ptcloud
 *              TODO
 *
 * @param       ptcloud_out
 *              TODO
 */
inline void transformPointcloud(const Transformation &T_N_O,
                                const Pointcloud     &ptcloud,
                                Pointcloud           *ptcloud_out)
{
    ptcloud_out->clear();
    ptcloud_out->resize(ptcloud.size());

    for (size_t i = 0; i < ptcloud.size(); ++i)
    {
        (*ptcloud_out)[i] = T_N_O * ptcloud[i];
    }
}

} // namespace voxblox

#endif // VOXBLOX_CORE_COMMON_H_
