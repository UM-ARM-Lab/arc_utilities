#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <Eigen/Geometry>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

#ifndef DYNAMIC_SPATIAL_HASHED_VOXEL_GRID_HPP
#define DYNAMIC_SPATIAL_HASHED_VOXEL_GRID_HPP

namespace VoxelGrid
{
    struct CHUNK_REGION
    {
        Eigen::Vector3d base;

        CHUNK_REGION() : base(Eigen::Vector3d(0.0, 0.0, 0.0)) {}

        CHUNK_REGION(const double base_x, const double base_y, const double base_z) : base(Eigen::Vector3d(base_x, base_y, base_z)) {}

        CHUNK_REGION(const Eigen::Vector3d& in_base) : base(in_base) {}

        bool operator==(const CHUNK_REGION& other) const
        {
            if (EigenHelpers::Equal(base, other.base))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    };

    template <typename T>
    class DynamicSpatialHashedVoxelGridChunk
    {
    protected:

        CHUNK_REGION region_;
        double cell_x_size_;
        double cell_y_size_;
        double cell_z_size_;
        int64_t num_x_cells_;
        int64_t num_y_cells_;
        int64_t num_z_cells_;
        double chunk_x_size_;
        double chunk_y_size_;
        double chunk_z_size_;
        int64_t stride1_;
        int64_t stride2_;
        bool chunk_initialized_;
        bool cell_initialized_;
        std::vector<T> data_;
        T initial_value_;

        void SafetyCheckSizes(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells,const int64_t num_y_cells, const int64_t num_z_cells) const
        {
            if (cell_x_size <= 0.0)
            {
                throw std::invalid_argument("cell_x_size must be positive and non-zero");
            }
            if (isnan(cell_x_size))
            {
                throw std::invalid_argument("cell_x_size must not be NaN");
            }
            if (isinf(cell_x_size) != 0)
            {
                throw std::invalid_argument("cell_x_size must not be INF");
            }
            if (cell_y_size <= 0.0)
            {
                throw std::invalid_argument("cell_y_size must be positive and non-zero");
            }
            if (isnan(cell_y_size))
            {
                throw std::invalid_argument("cell_y_size must not be NaN");
            }
            if (isinf(cell_y_size) != 0)
            {
                throw std::invalid_argument("cell_y_size must not be INF");
            }
            if (cell_z_size <= 0.0)
            {
                throw std::invalid_argument("cell_z_size must be positive and non-zero");
            }
            if (isnan(cell_z_size))
            {
                throw std::invalid_argument("cell_z_size must not be NaN");
            }
            if (isinf(cell_z_size) != 0)
            {
                throw std::invalid_argument("cell_z_size must not be INF");
            }
            if (num_x_cells <= 0)
            {
                throw std::invalid_argument("num_x_cells must be positive and non-zero");
            }
            if (num_y_cells <= 0)
            {
                throw std::invalid_argument("num_y_cells must be positive and non-zero");
            }
            if (num_z_cells <= 0)
            {
                throw std::invalid_argument("num_z_cells must be positive and non-zero");
            }
        }

        int64_t GetLocationDataIndex(const Eigen::Vector3d& location) const
        {
            Eigen::Vector3d location_in_chunk = location - region_.base;
            // First, make sure the location is in the range this chunk covers
            if (location_in_chunk.x() < 0.0 || location_in_chunk.y() < 0.0 || location_in_chunk.z() < 0.0)
            {
                return -1;
            }
            else if (location_in_chunk.x() > chunk_x_size_ || location_in_chunk.y() > chunk_y_size_ || location_in_chunk.z() > chunk_z_size_)
            {
                return -1;
            }
            // Ok, we're inside the chunk
            else
            {
                int64_t x_cell = (int64_t)(location_in_chunk.x() / cell_x_size_);
                int64_t y_cell = (int64_t)(location_in_chunk.y() / cell_y_size_);
                int64_t z_cell = (int64_t)(location_in_chunk.z() / cell_z_size_);
                if (x_cell < 0 || y_cell < 0 || z_cell < 0 || x_cell >= num_x_cells_ || y_cell >= num_y_cells_ || z_cell >= num_z_cells_)
                {
                    return -1;
                }
                else
                {
                    return (x_cell * stride1_) + (y_cell * stride2_) + z_cell;
                }
            }
        }

    public:

        DynamicSpatialHashedVoxelGridChunk(const CHUNK_REGION& region, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T initial_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            chunk_x_size_ = cell_x_size_ * (double)num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)num_z_cells_;
            region_ = region;
            cell_initialized_ = true;
            chunk_initialized_ = false;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            initial_value_ = initial_value;
            data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, initial_value);
        }

        DynamicSpatialHashedVoxelGridChunk(const CHUNK_REGION& region, const T initial_value)
        {
            cell_x_size_ = 0.0;
            cell_y_size_ = 0.0;
            cell_z_size_ = 0.0;
            num_x_cells_ = 0;
            num_y_cells_ = 0;
            num_z_cells_ = 0;
            chunk_x_size_ = 0.0;
            chunk_y_size_ = 0.0;
            chunk_z_size_ = 0.0;
            stride1_ = 0;
            stride2_ = 0;
            region_ = region;
            initial_value_ = initial_value;
            data_.resize(1, initial_value);
            cell_initialized_ = false;
            chunk_initialized_ = true;
        }

        DynamicSpatialHashedVoxelGridChunk()
        {
            cell_x_size_ = 0.0;
            cell_y_size_ = 0.0;
            cell_z_size_ = 0.0;
            num_x_cells_ = 0;
            num_y_cells_ = 0;
            num_z_cells_ = 0;
            chunk_x_size_ = 0.0;
            chunk_y_size_ = 0.0;
            chunk_z_size_ = 0.0;
            stride1_ = 0;
            stride2_ = 0;
            cell_initialized_ = false;
            chunk_initialized_ = false;
        }

        bool IsCellInitialized() const
        {
            return cell_initialized_;
        }

        bool IsChunkInitialized() const
        {
            return chunk_initialized_;
        }

        std::pair<T&, bool> GetCellMutable(const Eigen::Vector3d& location)
        {
            assert(cell_initialized_);
            int64_t data_index = GetLocationDataIndex(location);
            if (data_index >= 0)
            {
                assert(data_index < data_.size());
                return std::pair<T&, bool>(data_[data_index], true);
            }
            else
            {
                return std::pair<T&, bool>(initial_value_, false);
            }
        }

        std::pair<const T&, bool> GetCellImmutable(const Eigen::Vector3d& location) const
        {
            assert(cell_initialized_);
            int64_t data_index = GetLocationDataIndex(location);
            if (data_index >= 0)
            {
                assert(data_index < data_.size());
                return std::pair<const T&, bool>(data_[data_index], true);
            }
            else
            {
                return std::pair<const T&, bool>(initial_value_, false);
            }
        }

        std::pair<T, bool> GetCellCopy(const Eigen::Vector3d& location) const
        {
            assert(cell_initialized_);
            int64_t data_index = GetLocationDataIndex(location);
            if (data_index >= 0)
            {
                assert(data_index < data_.size());
                return std::pair<T, bool>(data_[data_index], true);
            }
            else
            {
                return std::pair<T, bool>(initial_value_, false);
            }
        }

        T& GetChunkMutable()
        {
            assert(chunk_initialized_);
            assert(data_.size() == 1);
            return data_[0];
        }

        const T& GetChunkImmutable() const
        {
            assert(chunk_initialized_);
            assert(data_.size() == 1);
            return data_[0];
        }

        T GetChunkCopy() const
        {
            assert(chunk_initialized_);
            assert(data_.size() == 1);
            return data_[0];
        }

        bool SetCellValue(const Eigen::Vector3d& location, T value)
        {
            assert(cell_initialized_);
            int64_t data_index = GetLocationDataIndex(location);
            if (data_index >= 0)
            {
                assert(data_index < data_.size());
                data_[data_index] = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        bool SetCellReference(const Eigen::Vector3d& location, T& value)
        {
            assert(cell_initialized_);
            int64_t data_index = GetLocationDataIndex(location);
            if (data_index >= 0)
            {
                assert(data_index < data_.size());
                data_[data_index] = value;
                return true;
            }
            else
            {
                return false;
            }
        }

        bool SetChunkValue(T value)
        {
            assert(chunk_initialized_);
            assert(data_.size() == 1);
            data_[0] = value;
            return true;
        }

        bool SetChunkReference(T& value)
        {
            assert(chunk_initialized_);
            assert(data_.size() == 1);
            data_[0] = value;
            return true;
        }
    };


    template <typename T>
    class DynamicSpatialHashedVoxelGrid
    {
    protected:

        bool initialized_;
        Eigen::Affine3d origin_transform_;
        Eigen::Affine3d inverse_origin_transform_;
        std::unordered_map<CHUNK_REGION, DynamicSpatialHashedVoxelGridChunk<T>> chunks_;
        double chunk_x_size_;
        double chunk_y_size_;
        double chunk_z_size_;
        int64_t chunk_num_x_cells_;
        int64_t chunk_num_y_cells_;
        int64_t chunk_num_z_cells_;
        int64_t chunk_stride1_;
        int64_t chunk_stride2_;
        double cell_x_size_;
        double cell_y_size_;
        double cell_z_size_;
        T default_value_;

        void SafetyCheckSizes(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t chunk_num_x_cells,const int64_t chunk_num_y_cells, const int64_t chunk_num_z_cells) const
        {
            if (cell_x_size <= 0.0)
            {
                throw std::invalid_argument("cell_x_size must be positive and non-zero");
            }
            if (isnan(cell_x_size))
            {
                throw std::invalid_argument("cell_x_size must not be NaN");
            }
            if (isinf(cell_x_size) != 0)
            {
                throw std::invalid_argument("cell_x_size must not be INF");
            }
            if (cell_y_size <= 0.0)
            {
                throw std::invalid_argument("cell_y_size must be positive and non-zero");
            }
            if (isnan(cell_y_size))
            {
                throw std::invalid_argument("cell_y_size must not be NaN");
            }
            if (isinf(cell_y_size) != 0)
            {
                throw std::invalid_argument("cell_y_size must not be INF");
            }
            if (cell_z_size <= 0.0)
            {
                throw std::invalid_argument("cell_z_size must be positive and non-zero");
            }
            if (isnan(cell_z_size))
            {
                throw std::invalid_argument("cell_z_size must not be NaN");
            }
            if (isinf(cell_z_size) != 0)
            {
                throw std::invalid_argument("cell_z_size must not be INF");
            }
            if (chunk_num_x_cells <= 0)
            {
                throw std::invalid_argument("chunk_num_x_cells must be positive and non-zero");
            }
            if (chunk_num_y_cells <= 0)
            {
                throw std::invalid_argument("chunk_num_y_cells must be positive and non-zero");
            }
            if (chunk_num_z_cells <= 0)
            {
                throw std::invalid_argument("chunk_num_z_cells must be positive and non-zero");
            }
        }

    public:

        enum FOUND_STATUS {NOT_FOUND, FOUND_IN_CHUNK, FOUND_IN_CELL};

        enum SET_STATUS {NOT_SET, SET_CHUNK, SET_CELL};

        DynamicSpatialHashedVoxelGrid(Eigen::Affine3d origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t chunk_num_x_cells, const int64_t chunk_num_y_cells, const int64_t chunk_num_z_cells, T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, chunk_num_x_cells, chunk_num_y_cells, chunk_num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            chunk_num_x_cells_ = chunk_num_x_cells;
            chunk_num_y_cells_ = chunk_num_y_cells;
            chunk_num_z_cells_ = chunk_num_z_cells;
            chunk_x_size_ = cell_x_size_ * (double)chunk_num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)chunk_num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)chunk_num_z_cells_;
            default_value_ = default_value;
            chunk_stride1_ = chunk_num_y_cells_ * chunk_num_z_cells_;
            chunk_stride2_ = chunk_num_z_cells_;
            initialized_ = true;
        }

        DynamicSpatialHashedVoxelGrid(Eigen::Affine3d origin_transform, const double cell_size, const int64_t chunk_num_x_cells, const int64_t chunk_num_y_cells, const int64_t chunk_num_z_cells, T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, chunk_num_x_cells, chunk_num_y_cells, chunk_num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            chunk_num_x_cells_ = chunk_num_x_cells;
            chunk_num_y_cells_ = chunk_num_y_cells;
            chunk_num_z_cells_ = chunk_num_z_cells;
            chunk_x_size_ = cell_x_size_ * (double)chunk_num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)chunk_num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)chunk_num_z_cells_;
            default_value_ = default_value;
            chunk_stride1_ = chunk_num_y_cells_ * chunk_num_z_cells_;
            chunk_stride2_ = chunk_num_z_cells_;
            initialized_ = true;
        }

        DynamicSpatialHashedVoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t chunk_num_x_cells, const int64_t chunk_num_y_cells, const int64_t chunk_num_z_cells, T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, chunk_num_x_cells, chunk_num_y_cells, chunk_num_z_cells);
            origin_transform_.setIdentity();
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            chunk_num_x_cells_ = chunk_num_x_cells;
            chunk_num_y_cells_ = chunk_num_y_cells;
            chunk_num_z_cells_ = chunk_num_z_cells;
            chunk_x_size_ = cell_x_size_ * (double)chunk_num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)chunk_num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)chunk_num_z_cells_;
            default_value_ = default_value;
            chunk_stride1_ = chunk_num_y_cells_ * chunk_num_z_cells_;
            chunk_stride2_ = chunk_num_z_cells_;
            initialized_ = true;
        }

        DynamicSpatialHashedVoxelGrid(const double cell_size, const int64_t chunk_num_x_cells, const int64_t chunk_num_y_cells, const int64_t chunk_num_z_cells, T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, chunk_num_x_cells, chunk_num_y_cells, chunk_num_z_cells);
            origin_transform_.setIdentity();
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            chunk_num_x_cells_ = chunk_num_x_cells;
            chunk_num_y_cells_ = chunk_num_y_cells;
            chunk_num_z_cells_ = chunk_num_z_cells;
            chunk_x_size_ = cell_x_size_ * (double)chunk_num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)chunk_num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)chunk_num_z_cells_;
            default_value_ = default_value;
            chunk_stride1_ = chunk_num_y_cells_ * chunk_num_z_cells_;
            chunk_stride2_ = chunk_num_z_cells_;
            initialized_ = true;
        }

        DynamicSpatialHashedVoxelGrid()
        {
            origin_transform_ = Eigen::Affine3d::Identity();
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = 0.0;
            cell_y_size_ = 0.0;
            cell_z_size_ = 0.0;
            chunk_num_x_cells_ = 0;
            chunk_num_y_cells_ = 0;
            chunk_num_z_cells_ = 0;
            chunk_x_size_ = cell_x_size_ * (double)chunk_num_x_cells_;
            chunk_y_size_ = cell_y_size_ * (double)chunk_num_y_cells_;
            chunk_z_size_ = cell_z_size_ * (double)chunk_num_z_cells_;
            chunk_stride1_ = chunk_num_y_cells_ * chunk_num_z_cells_;
            chunk_stride2_ = chunk_num_z_cells_;
            initialized_ = true;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline std::pair<const T&, FOUND_STATUS> GetImmutable(const double x, const double y, const double z) const
        {
            Eigen::Vector3d location(x, y, z);
            return GetImmutable(location);
        }

        inline std::pair<T, FOUND_STATUS> GetCopy(const double x, const double y, const double z) const
        {
            Eigen::Vector3d location(x, y, z);
            return GetCopy(location);
        }

        inline std::pair<T&, FOUND_STATUS> GetMutable(const double x, const double y, const double z)
        {
            Eigen::Vector3d location(x, y, z);
            return GetMutable(location);
        }

        inline SET_STATUS SetCellWithReference(const double x, const double y, const double z, T& value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetCellWithReference(location, value);
        }

        inline SET_STATUS SetCellWithValue(const double x, const double y, const double z, T value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetCellWithValue(location, value);
        }

        inline SET_STATUS SetChunkWithReference(const double x, const double y, const double z, T& value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetChunkWithReference(location, value);
        }

        inline SET_STATUS SetChunkWithValue(const double x, const double y, const double z, T value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetChunkWithValue(location, value);
        }

        inline CHUNK_REGION GetContainingChunkRegion(const Eigen::Vector3d& grid_location) const
        {
            assert(initialized_);
            std::cout << "Grid location - " << PrettyPrint::PrettyPrint(grid_location) << std::endl;
            // Given a location in the grid frame, figure out which chunk region it falls into
            double raw_x_chunk_num = grid_location.x() / chunk_x_size_;
            double raw_y_chunk_num = grid_location.y() / chunk_y_size_;
            double raw_z_chunk_num = grid_location.z() / chunk_z_size_;
            int64_t x_chunk_num = (int64_t)floor(raw_x_chunk_num);
            int64_t y_chunk_num = (int64_t)floor(raw_y_chunk_num);
            int64_t z_chunk_num = (int64_t)floor(raw_z_chunk_num);
            double region_base_x = (double)x_chunk_num * chunk_x_size_;
            double region_base_y = (double)y_chunk_num * chunk_y_size_;
            double region_base_z = (double)z_chunk_num * chunk_z_size_;
            CHUNK_REGION region(region_base_x, region_base_y, region_base_z);
            std::cout << "Region base - " << PrettyPrint::PrettyPrint(region.base) << std::endl;
            return region;
        }

        inline std::pair<const T&, FOUND_STATUS> GetImmutable(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                const DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    // Get the data
                    std::pair<const T&, bool> found_in_cell = chunk.GetCellImmutable(grid_location);
                    if (found_in_cell.second)
                    {
                        return std::pair<const T&, FOUND_STATUS>(found_in_cell.first, FOUND_IN_CELL);
                    }
                    else
                    {
                        return std::pair<const T&, FOUND_STATUS>(default_value_, NOT_FOUND);
                    }
                }
                else if (chunk.IsChunkInitialized())
                {
                    return std::pair<const T&, FOUND_STATUS>(chunk.GetChunkImmutable(), FOUND_IN_CHUNK);
                }
                else
                {
                    return std::pair<const T&, FOUND_STATUS>(default_value_, NOT_FOUND);
                }
            }
            else
            {
                return std::pair<const T&, FOUND_STATUS>(default_value_, NOT_FOUND);
            }
        }

        inline std::pair<T, FOUND_STATUS> GetCopy(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                const DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    // Get the data
                    std::pair<T, bool> found_in_cell = chunk.GetCellCopy(grid_location);
                    if (found_in_cell.second)
                    {
                        return std::pair<T, FOUND_STATUS>(found_in_cell.first, FOUND_IN_CELL);
                    }
                    else
                    {
                        return std::pair<T, FOUND_STATUS>(default_value_, NOT_FOUND);
                    }
                }
                else if (chunk.IsChunkInitialized())
                {
                    return std::pair<T, FOUND_STATUS>(chunk.GetChunkCopy(), FOUND_IN_CHUNK);
                }
                else
                {
                    return std::pair<T, FOUND_STATUS>(default_value_, NOT_FOUND);
                }
            }
            else
            {
                return std::pair<T, FOUND_STATUS>(default_value_, NOT_FOUND);
            }
        }

        inline std::pair<T&, FOUND_STATUS> GetMutable(const Eigen::Vector3d& location)
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    // Get the data
                    std::pair<T&, bool> found_in_cell = chunk.GetCellMutable(grid_location);
                    if (found_in_cell.second)
                    {
                        return std::pair<T&, FOUND_STATUS>(found_in_cell.first, FOUND_IN_CELL);
                    }
                    else
                    {
                        return std::pair<T&, FOUND_STATUS>(default_value_, NOT_FOUND);
                    }
                }
                else if (chunk.IsChunkInitialized())
                {
                    return std::pair<T&, FOUND_STATUS>(chunk.GetChunkMutable(), FOUND_IN_CHUNK);
                }
                else
                {
                    return std::pair<T&, FOUND_STATUS>(default_value_, NOT_FOUND);
                }
            }
            else
            {
                return std::pair<T&, FOUND_STATUS>(default_value_, NOT_FOUND);
            }
        }

        inline SET_STATUS SetCellWithReference(const Eigen::Vector3d& location, T& value)
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    if (chunk.SetCellReference(grid_location, value))
                    {
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else if (chunk.IsChunkInitialized())
                {
                    T current_chunk_value = chunk.GetMutableChunkValue();
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, current_chunk_value);
                    if (new_chunk.SetCellReference(grid_location, value))
                    {
                        chunks_[region] = new_chunk;
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, default_value_);
                    if (new_chunk.SetCellReference(grid_location, value))
                    {
                        chunks_[region] = new_chunk;
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
            }
            else
            {
                // Make a new chunk
                DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, default_value_);
                if (new_chunk.SetCellReference(grid_location, value))
                {
                    chunks_[region] = new_chunk;
                    return SET_CELL;
                }
                else
                {
                    return NOT_SET;
                }
            }
        }

        inline SET_STATUS SetCellWithValue(const Eigen::Vector3d& location, T value)
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    if (chunk.SetCellValue(grid_location, value))
                    {
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else if (chunk.IsChunkInitialized())
                {
                    T current_chunk_value = chunk.GetChunkMutable();
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, current_chunk_value);
                    if (new_chunk.SetCellValue(grid_location, value))
                    {
                        chunks_[region] = new_chunk;
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, default_value_);
                    if (new_chunk.SetCellValue(grid_location, value))
                    {
                        chunks_[region] = new_chunk;
                        return SET_CELL;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
            }
            else
            {
                // Make a new chunk
                DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, cell_x_size_, cell_y_size_, cell_z_size_, chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_, default_value_);
                if (new_chunk.SetCellValue(grid_location, value))
                {
                    chunks_[region] = new_chunk;
                    return SET_CELL;
                }
                else
                {
                    return NOT_SET;
                }
            }
        }

        inline SET_STATUS SetChunkWithReference(const Eigen::Vector3d& location, T& value)
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                    chunks_[region] = new_chunk;
                    return SET_CHUNK;
                }
                else if (chunk.IsChunkInitialized())
                {
                    if (chunk.SetChunkReference(value))
                    {
                        return SET_CHUNK;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                    chunks_[region] = new_chunk;
                    return SET_CHUNK;
                }
            }
            else
            {
                // Make a new chunk
                DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                chunks_[region] = new_chunk;
                return SET_CHUNK;
            }
        }

        inline SET_STATUS SetChunkWithValue(const Eigen::Vector3d& location, T value)
        {
            assert(initialized_);
            Eigen::Vector3d grid_location = inverse_origin_transform_ * location;
            CHUNK_REGION region = GetContainingChunkRegion(grid_location);
            auto found_chunk_itr = chunks_.find(region);
            if (found_chunk_itr != chunks_.end())
            {
                DynamicSpatialHashedVoxelGridChunk<T>& chunk = found_chunk_itr->second;
                if (chunk.IsCellInitialized())
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                    chunks_[region] = new_chunk;
                    return SET_CHUNK;
                }
                else if (chunk.IsChunkInitialized())
                {
                    if (chunk.SetChunkValue(value))
                    {
                        return SET_CHUNK;
                    }
                    else
                    {
                        return NOT_SET;
                    }
                }
                else
                {
                    // Make a new chunk
                    DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                    chunks_[region] = new_chunk;
                    return SET_CHUNK;
                }
            }
            else
            {
                // Make a new chunk
                DynamicSpatialHashedVoxelGridChunk<T> new_chunk(region, value);
                chunks_[region] = new_chunk;
                return SET_CHUNK;
            }
        }

        inline std::vector<double> GetCellSizes() const
        {
            return std::vector<double>{cell_x_size_, cell_y_size_, cell_z_size_};
        }

        inline std::vector<double> GetChunkSizes() const
        {
            return std::vector<double>{cell_x_size_ * (double)chunk_num_x_cells_, cell_y_size_ * (double)chunk_num_y_cells_, cell_z_size_ * (double)chunk_num_z_cells_};
        }

        inline std::vector<int64_t> GetChunkNumCells() const
        {
            return std::vector<int64_t>{chunk_num_x_cells_, chunk_num_y_cells_, chunk_num_z_cells_};
        }

        inline T GetDefaultValue() const
        {
            return default_value_;
        }

        inline void SetDefaultValue(T default_value)
        {
            default_value_ = default_value;
        }

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return origin_transform_;
        }

    };
}

namespace std
{
    template <>
    struct hash<VoxelGrid::CHUNK_REGION>
    {
        std::size_t operator()(const VoxelGrid::CHUNK_REGION& region) const
        {
            using std::size_t;
            using std::hash;
            return (std::hash<Eigen::Vector3d>()(region.base));
        }
    };
}

#endif // DYNAMIC_SPATIAL_HASHED_VOXEL_GRID_HPP
