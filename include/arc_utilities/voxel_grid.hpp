#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>
#include <stdexcept>

#ifndef VOXEL_GRID_HPP
#define VOXEL_GRID_HPP

namespace VoxelGrid
{
    struct GRID_INDEX
    {
        int64_t x;
        int64_t y;
        int64_t z;

        GRID_INDEX() : x(-1), y(-1), z(-1) {}

        GRID_INDEX(const int64_t in_x, const int64_t in_y, const int64_t in_z) : x(in_x), y(in_y), z(in_z) {}

        bool operator==(const GRID_INDEX& other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
    };

    template <typename T>
    class VoxelGrid
    {
    protected:

        bool initialized_;
        Eigen::Affine3d origin_transform_;
        Eigen::Affine3d inverse_origin_transform_;
        std::vector<T> data_;
        double cell_x_size_;
        double cell_y_size_;
        double cell_z_size_;
        double x_size_;
        double y_size_;
        double z_size_;
        int64_t stride1_;
        int64_t stride2_;
        int64_t num_x_cells_;
        int64_t num_y_cells_;
        int64_t num_z_cells_;
        T default_value_;
        T oob_value_;

        inline int64_t GetDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }

        void SetContents(T value)
        {
            data_.clear();
            data_.resize(num_x_cells_ * num_y_cells_ * num_z_cells_, value);
        }

        void SafetyCheckSizes(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size,const double y_size, const double z_size) const
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
            if (x_size <= 0.0)
            {
                throw std::invalid_argument("x_size must be positive and non-zero");
            }
            if (y_size <= 0.0)
            {
                throw std::invalid_argument("y_size must be positive and non-zero");
            }
            if (z_size <= 0.0)
            {
                throw std::invalid_argument("z_size must be positive and non-zero");
            }
            if (isnan(x_size))
            {
                throw std::invalid_argument("x_size must not be NaN");
            }
            if (isnan(y_size))
            {
                throw std::invalid_argument("y_size must not be NaN");
            }
            if (isnan(z_size))
            {
                throw std::invalid_argument("z_size must not be NaN");
            }
            if (isinf(x_size) != 0)
            {
                throw std::invalid_argument("x_size must not be INF");
            }
            if (isinf(y_size) != 0)
            {
                throw std::invalid_argument("y_size must not be INF");
            }
            if (isinf(z_size) != 0)
            {
                throw std::invalid_argument("z_size must not be INF");
            }
        }

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

    public:

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_size, const double x_size, const double y_size, double const z_size, const T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, x_size, y_size, z_size);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_size, const double x_size, const double y_size, const double z_size, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, x_size, y_size, z_size);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, double const z_size, const T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            default_value_ = default_value;
            oob_value_ = default_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            default_value_ = default_value;
            oob_value_ = oob_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            default_value_ = default_value;
            oob_value_ = default_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const Eigen::Affine3d origin_transform, const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            origin_transform_ = origin_transform;
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            default_value_ = default_value;
            oob_value_ = oob_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_size, const double x_size, const double y_size, const double z_size, const T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, x_size, y_size, z_size);
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_size, const double x_size, const double y_size, const double z_size, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, x_size, y_size, z_size);
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const double x_size, const double y_size, const double z_size, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, x_size, y_size, z_size);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            x_size_ = fabs(x_size);
            y_size_ = fabs(y_size);
            z_size_ = fabs(z_size);
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            num_x_cells_ = (int64_t)(ceil(x_size_ / cell_x_size_));
            num_y_cells_ = (int64_t)(ceil(y_size_ / cell_y_size_));
            num_z_cells_ = (int64_t)(ceil(z_size_ / cell_z_size_));
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_size, cell_size, cell_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_size);
            cell_y_size_ = fabs(cell_size);
            cell_z_size_ = fabs(cell_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = default_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid(const double cell_x_size, const double cell_y_size, const double cell_z_size, const int64_t num_x_cells, const int64_t num_y_cells, const int64_t num_z_cells, const T default_value, const T oob_value)
        {
            SafetyCheckSizes(cell_x_size, cell_y_size, cell_z_size, num_x_cells, num_y_cells, num_z_cells);
            cell_x_size_ = fabs(cell_x_size);
            cell_y_size_ = fabs(cell_y_size);
            cell_z_size_ = fabs(cell_z_size);
            num_x_cells_ = num_x_cells;
            num_y_cells_ = num_y_cells;
            num_z_cells_ = num_z_cells;
            x_size_ = (double)num_x_cells_ * cell_x_size_;
            y_size_ = (double)num_y_cells_ * cell_y_size_;
            z_size_ = (double)num_z_cells_ * cell_z_size_;
            Eigen::Translation3d origin_translation(-x_size_ * 0.5, -y_size_ * 0.5, -z_size_ * 0.5);
            Eigen::Quaterniond origin_rotation;
            origin_rotation.setIdentity();
            origin_transform_ = origin_translation * origin_rotation;
            inverse_origin_transform_ = origin_transform_.inverse();
            default_value_ = default_value;
            oob_value_ = oob_value;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            SetContents(default_value_);
            initialized_ = true;
        }

        VoxelGrid()
        {
            origin_transform_ = Eigen::Affine3d::Identity();
            inverse_origin_transform_ = origin_transform_.inverse();
            cell_x_size_ = 0.0;
            cell_y_size_ = 0.0;
            cell_z_size_ = 0.0;
            x_size_ = 0.0;
            y_size_ = 0.0;
            z_size_ = 0.0;
            num_x_cells_ = 0;
            num_y_cells_ = 0;
            num_z_cells_ = 0;
            stride1_ = num_y_cells_ * num_z_cells_;
            stride2_ = num_z_cells_;
            initialized_ = false;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        void ResetWithDefault()
        {
            SetContents(default_value_);
        }

        void ResetWithNewValue(T new_value)
        {
            SetContents(new_value);
        }

        void ResetWithNewDefault(T new_default)
        {
            default_value_ = new_default;
            SetContents(default_value_);
        }

        inline std::pair<const T&, bool> GetImmutable(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            std::vector<int64_t> indices = LocationToGridIndex(location);
            if (indices.size() != 3)
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
            else
            {
                return GetImmutable(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<const T&, bool> GetImmutable(const double x, const double y, const double z) const
        {
            Eigen::Vector3d location(x, y, z);
            return GetImmutable(location);
        }

        inline std::pair<const T&, bool> GetImmutable(const GRID_INDEX& index) const
        {
            return GetImmutable(index.x, index.y, index.z);
        }

        inline std::pair<const T&, bool> GetImmutable(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<const T&, bool>(oob_value_, false);
            }
            else
            {
                int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < data_.size());
                return std::pair<const T&, bool>(data_[data_index], true);
            }
        }

        inline std::pair<T, bool> GetCopy(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            std::vector<int64_t> indices = LocationToGridIndex(location);
            if (indices.size() != 3)
            {
                return std::pair<T, bool>(oob_value_, false);
            }
            else
            {
                return GetCopy(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<T, bool> GetCopy(const double x, const double y, const double z) const
        {
            Eigen::Vector3d location(x, y, z);
            return GetCopy(location);
        }

        inline std::pair<T, bool> GetCopy(const GRID_INDEX& index) const
        {
            return GetCopy(index.x, index.y, index.z);
        }

        inline std::pair<T, bool> GetCopy(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<T, bool>(oob_value_, false);
            }
            else
            {
                int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < data_.size());
                return std::pair<T, bool>(data_[data_index], true);
            }
        }

        inline std::pair<T&, bool> GetMutable(const Eigen::Vector3d& location)
        {
            assert(initialized_);
            std::vector<int64_t> indices = LocationToGridIndex(location);
            if (indices.size() != 3)
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
            else
            {
                return GetMutable(indices[0], indices[1], indices[2]);
            }
        }

        inline std::pair<T&, bool> GetMutable(const double x, const double y, const double z)
        {
            Eigen::Vector3d location(x, y, z);
            return GetMutable(location);
        }

        inline std::pair<T&, bool> GetMutable(const GRID_INDEX& index)
        {
            return GetMutable(index.x, index.y, index.z);
        }

        inline std::pair<T&, bool> GetMutable(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::pair<T&, bool>(oob_value_, false);
            }
            else
            {
                int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < data_.size());
                return std::pair<T&, bool>(data_[data_index], true);
            }
        }

        inline bool SetWithReference(const Eigen::Vector3d& location, T& value)
        {
            assert(initialized_);
            std::vector<int64_t> indices = LocationToGridIndex(location);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                return SetWithReference(indices[0], indices[1], indices[2], value);
            }
        }

        inline bool SetWithReference(const double x, const double y, const double z, T& value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetWithReference(location, value);
        }

        inline bool  SetWithReference(const GRID_INDEX& index, T& value)
        {
            return SetWithReference(index.x, index.y, index.z, value);
        }

        inline bool SetWithReference(const int64_t x_index, const int64_t y_index, const int64_t z_index, T& value)
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return false;
            }
            else
            {
                int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < data_.size());
                data_[data_index] = value;
                return true;
            }
        }

        inline bool SetWithValue(const Eigen::Vector3d& location, T value)
        {
            assert(initialized_);
            std::vector<int64_t> indices = LocationToGridIndex(location);
            if (indices.size() != 3)
            {
                return false;
            }
            else
            {
                return SetWithValue(indices[0], indices[1], indices[2], value);
            }
        }

        inline bool SetWithValue(const double x, const double y, const double z, T value)
        {
            Eigen::Vector3d location(x, y, z);
            return SetWithValue(location, value);
        }

        inline bool  SetWithValue(const GRID_INDEX& index, T value)
        {
            return SetWithValue(index.x, index.y, index.z, value);
        }

        inline bool SetWithValue(const int64_t x_index, const int64_t y_index, const int64_t z_index, T value)
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return false;
            }
            else
            {
                int64_t data_index = GetDataIndex(x_index, y_index, z_index);
                assert(data_index >= 0 && data_index < data_.size());
                data_[data_index] = value;
                return true;
            }
        }

        inline double GetXSize() const
        {
            return x_size_;
        }

        inline double GetYSize() const
        {
            return y_size_;
        }

        inline double GetZSize() const
        {
            return z_size_;
        }

        inline std::vector<double> GetCellSizes() const
        {
            return std::vector<double>{cell_x_size_, cell_y_size_, cell_z_size_};
        }

        inline T GetDefaultValue() const
        {
            return default_value_;
        }

        inline T GetOOBValue() const
        {
            return oob_value_;
        }

        inline void SetDefaultValue(T default_value)
        {
            default_value_ = default_value;
        }

        inline void SetOOBValue(T oob_value)
        {
            oob_value_ = oob_value;
        }

        inline int64_t GetNumXCells() const
        {
            return num_x_cells_;
        }

        inline int64_t GetNumYCells() const
        {
            return num_y_cells_;
        }

        inline int64_t GetNumZCells() const
        {
            return num_z_cells_;
        }

        inline Eigen::Affine3d GetOriginTransform() const
        {
            return origin_transform_;
        }

        inline std::vector<int64_t> LocationToGridIndex(const double x, const double y, const double z) const
        {
            Eigen::Vector3d point(x, y, z);
            return LocationToGridIndex(point);
        }

        inline std::vector<int64_t> LocationToGridIndex(const Eigen::Vector3d& location) const
        {
            assert(initialized_);
            Eigen::Vector3d point_in_grid_frame = inverse_origin_transform_ * location;
            int64_t x_cell = (int64_t)(point_in_grid_frame.x() / cell_x_size_);
            int64_t y_cell = (int64_t)(point_in_grid_frame.y() / cell_y_size_);
            int64_t z_cell = (int64_t)(point_in_grid_frame.z() / cell_z_size_);
            if (x_cell < 0 || y_cell < 0 || z_cell < 0 || x_cell >= num_x_cells_ || y_cell >= num_y_cells_ || z_cell >= num_z_cells_)
            {
                return std::vector<int64_t>();
            }
            else
            {
                return std::vector<int64_t>{x_cell, y_cell, z_cell};
            }
        }

        inline std::vector<double> GridIndexToLocation(const GRID_INDEX& index) const
        {
            return GridIndexToLocation(index.x, index.y, index.z);
        }

        inline std::vector<double> GridIndexToLocation(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            assert(initialized_);
            if (x_index < 0 || y_index < 0 || z_index < 0 || x_index >= num_x_cells_ || y_index >= num_y_cells_ || z_index >= num_z_cells_)
            {
                return std::vector<double>();
            }
            else
            {
                Eigen::Vector3d point_in_grid_frame(cell_x_size_ * ((double)x_index + 0.5), cell_y_size_ * ((double)y_index + 0.5), cell_z_size_ * ((double)z_index + 0.5));
                Eigen::Vector3d point = origin_transform_ * point_in_grid_frame;
                return std::vector<double>{point.x(), point.y(), point.z()};
            }
        }

        const std::vector<T>& GetRawData() const
        {
            return data_;
        }

        std::vector<T> CopyRawData() const
        {
            return data_;
        }

        bool SetRawData(std::vector<T>& data)
        {
            assert(initialized_);
            int64_t expected_length = num_x_cells_ * num_y_cells_ * num_z_cells_;
            if ((int64_t)data.size() != expected_length)
            {
                std::cerr << "Failed to load internal data - expected " << expected_length << " got " << data.size() << std::endl;
                return false;
            }
            else
            {
                data_ = data;
                return true;
            }
        }

        inline u_int64_t HashDataIndex(const int64_t x_index, const int64_t y_index, const int64_t z_index) const
        {
            return (x_index * stride1_) + (y_index * stride2_) + z_index;
        }
    };
}

namespace std
{
    template <>
    struct hash<VoxelGrid::GRID_INDEX>
    {
        std::size_t operator()(const VoxelGrid::GRID_INDEX& index) const
        {
            using std::size_t;
            using std::hash;
            return ((std::hash<int64_t>()(index.x) ^ (std::hash<int64_t>()(index.y) << 1) >> 1) ^ (std::hash<int64_t>()(index.z) << 1));
        }
    };
}

#endif // VOXEL_GRID_HPP
