// Copyright (c) 2019 Martyn Afford
// Licensed under the MIT licence
// Modified by Yanze on March 14, 2025

#ifndef BRIDSON_POISSON_DISC_DISTRIBUTION_HPP
#define BRIDSON_POISSON_DISC_DISTRIBUTION_HPP

#include <cmath>
#include <limits>
#include <stack>
#include <vector>

constexpr static auto infinity = std::numeric_limits<float>::infinity();
struct Point
{
    float x = infinity;
    float y = infinity;
    float z = infinity;
};

namespace bridson
{
    struct config
    {
        float xx = 1.0f; // xx is the length along x axis
        float yy = 1.0f;
        float zz = 1.0f;
        float min_distance = 0.05f;
        int max_attempts = 300;
        Point start;
    };

    template <typename T, typename T1>
    std::vector<Point> poisson_disc_distribution(config conf, T &&random, T1 &&in_area)
    {
        // cell size is bounded by r/sqrt(n), where r is minimum distance and n is space dimension
        float cell_size = conf.min_distance / std::sqrt(3);
        // X is the size along x axis
        int X = std::ceil(conf.xx / cell_size);
        int Y = std::ceil(conf.yy / cell_size);
        int Z = std::ceil(conf.zz / cell_size);

        std::vector<Point> grid(X * Y * Z);
        std::stack<Point> process; // this corresponds to the "active list" in Bridson's paper.
        std::vector<Point> points;

        auto squared_distance = [](const Point &a, const Point &b)
        {
            auto delta_x = a.x - b.x;
            auto delta_y = a.y - b.y;
            auto delta_z = a.z - b.z;
            return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
        };

        auto point_around = [&conf, &random](Point p)
        {
            // If we directly uniformly sample r in [r,2r],
            // the points will not be evenly distributed in the sphere,
            // but will be more concentrated at the center of the sphere!

            //  2D： auto radius = conf.min_distance * std::sqrt(random(3) + 1);
            //  3D：
            auto radius = conf.min_distance * std::cbrt(random(7) + 1);
            auto altitude = random(M_PI) - M_PI / 2.0; // altitude 仰角
            auto azimuth = random(2 * M_PI);           // azimuth方位角
            p.x += std::cos(altitude) * std::cos(azimuth) * radius;
            p.y += std::sin(altitude) * radius;
            p.z += std::cos(altitude) * std::sin(azimuth) * radius;
            return p;
        };

        auto index = [X, Y, Z](int x_index, int y_index, int z_index)
        {
            return x_index * Y * Z + y_index * Z + z_index;
        };

        auto set = [cell_size, X, Y, Z, &grid, &index](const Point &p)
        {
            int x_index = p.x / cell_size;
            int y_index = p.y / cell_size;
            int z_index = p.z / cell_size;
            grid[index(x_index, y_index, z_index)] = p;
        };

        auto add = [&process, &set, &points](const Point &p)
        {
            process.push(p);
            points.push_back(p);
            set(p);
        };

        auto point_too_close = [&](const Point &p)
        {
            int x_index = std::floor(p.x / cell_size);
            int y_index = std::floor(p.y / cell_size);
            int z_index = std::floor(p.z / cell_size);

            if (grid[index(x_index, y_index, z_index)].x != infinity)
            {
                return true;
            }

            auto min_dist_squared = conf.min_distance * conf.min_distance;
            auto min_x = std::max(x_index - 2, 0);
            auto min_y = std::max(y_index - 2, 0);
            auto min_z = std::max(z_index - 2, 0);
            auto max_x = std::min(x_index + 2, X - 1);
            auto max_y = std::min(y_index + 2, Y - 1);
            auto max_z = std::min(z_index + 2, Z - 1);

            for (auto z = min_z; z <= max_z; ++z)
            {
                for (auto y = min_y; y <= max_y; ++y)
                {
                    for (auto x = min_x; x <= max_x; ++x)
                    {
                        auto point = grid[index(x, y, z)];
                        auto exists = point.x != infinity;
                        if (exists && squared_distance(p, point) < min_dist_squared)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        };

        if (conf.start.x == infinity)
        {
            do
            {
                conf.start.x = random(conf.xx);
                conf.start.y = random(conf.yy);
                conf.start.z = random(conf.zz);
            } while (!in_area(conf.start));
        }
        add(conf.start);
        while (!process.empty())
        {
            auto point = process.top();
            process.pop();
            for (int i = 0; i != conf.max_attempts; ++i)
            {
                auto p = point_around(point);
                if (in_area(p) && !point_too_close(p))
                {
                    add(p);
                }
            }
        }
        return points;
    }
} // namespace bridson

namespace uniform_comparison
{

    struct config
    {
        float xx = 1.0f;
        float yy = 1.0f;
        float zz = 1.0f;
        int sample_number = 100;
    };

    template <typename T, typename T1>
    std::vector<Point> uniform_distribution(config conf, T &&random, T1 &&in_area)
    {
        std::vector<Point> points;
        for (int i = 0; i < conf.sample_number; i++)
        {
            Point p;
            p.x = random(conf.xx);
            p.y = random(conf.yy);
            p.z = random(conf.zz);
            if (in_area(p))
            {
                points.push_back(p);
            }
        }
        return points;
    }
} // namespace uniform_comparison

#endif /* BRIDSON_POISSON_DISC_DISTRIBUTION_HPP */