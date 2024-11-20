#include <cmath>
#include <cstdlib>
#include <vector>
#include "singleChannelMap.h"
#include "vect2.h"

PerlinNoiseMap::PerlinNoiseMap(Vect2 offset, double strength, size_t w, size_t h, double cell_size, Interp f): SingleChannelMap{}, grid{std::vector<std::vector<Vect2>>()}, offset{offset}, strength{strength}, cell_size{cell_size}, f{f} {
    grid.reserve(h);
    for (size_t i = 0; i < h; ++i) {
        grid.emplace_back();
        grid[i].reserve(w);
        for (size_t j = 0; j < w; ++j) {
            // get random angle
            double theta = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) * 2.0 * M_PI;
            // store associated unit vector
            grid[i].emplace_back(std::cos(theta), std::sin(theta));
        }
    }
}

double PerlinNoiseMap::sample(Vect2 p) const {
    p = p + offset;
    if (p.getX() < 0 || p.getY() < 0 || p.getY() >= grid.size() * cell_size || p.getX() >= grid[0].size() * cell_size) {
        throw OutOfBounds {};
    }

    // normalize coordinates
    p = p.scale(1 / cell_size);

    // obtain dot products for each corner
    size_t cell_x = int(floor(p.getX()));
    size_t cell_y = int(floor(p.getY()));
    Vect2 offset(p.getX() - floor(p.getX()), p.getY() - floor(p.getY()));
    double dot_00 = offset.dot(grid[cell_y][cell_x]);
    double dot_01 = (offset - Vect2(0, 1)).dot(grid[cell_y + 1][cell_x]);
    double dot_10 = (offset - Vect2(1, 0)).dot(grid[cell_y][cell_x + 1]);
    double dot_11 = (offset - Vect2(1, 1)).dot(grid[cell_y + 1][cell_x + 1]);

    // interpolate
    return (f(f(dot_00, dot_10, offset.getX()), f(dot_01, dot_11, offset.getX()), offset.getY()) * 0.5 + 0.5) * strength;
}

ClampMap::ClampMap(std::unique_ptr<SingleChannelMap> map, double min, double max): SingleChannelMap{}, map{std::move(map)}, min{min}, max{max} {}

double ClampMap::sample(Vect2 p) const {
    double val = map->sample(p);
    if (val < min) val = min;
    if (val > max) val = max;
    return val;
}

WaterMap::WaterMap(std::unique_ptr<SingleChannelMap> map, double threshold): SingleChannelMap{}, map{std::move(map)}, threshold{threshold} {}

double WaterMap::sample(Vect2 p) const {
    return map->sample(p) >= threshold ? 1.0 : 0.0;
}

