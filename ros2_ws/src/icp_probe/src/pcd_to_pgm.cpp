#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

struct Options {
  std::filesystem::path input;
  std::filesystem::path output_base;
  double resolution = 0.05;
  double min_z = -0.20;
  double max_z = 1.50;
  double min_x = -std::numeric_limits<double>::infinity();
  double max_x = std::numeric_limits<double>::infinity();
  double min_y = -std::numeric_limits<double>::infinity();
  double max_y = std::numeric_limits<double>::infinity();
  double padding = 1.0;
  int dilation_cells = 1;
};

void PrintUsage(const char *program) {
  std::cerr
      << "Usage: " << program
      << " --input MAP.pcd --output MAP_BASE [--resolution 0.05]"
         " [--min-z -0.20] [--max-z 1.50] [--padding 1.0]"
         " [--min-x X] [--max-x X] [--min-y Y] [--max-y Y]"
         " [--dilation-cells 1]\n";
}

Options ParseArguments(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("Missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--input") {
      options.input = value;
    } else if (argument == "--output") {
      options.output_base = value;
    } else if (argument == "--resolution") {
      options.resolution = std::stod(value);
    } else if (argument == "--min-z") {
      options.min_z = std::stod(value);
    } else if (argument == "--max-z") {
      options.max_z = std::stod(value);
    } else if (argument == "--min-x") {
      options.min_x = std::stod(value);
    } else if (argument == "--max-x") {
      options.max_x = std::stod(value);
    } else if (argument == "--min-y") {
      options.min_y = std::stod(value);
    } else if (argument == "--max-y") {
      options.max_y = std::stod(value);
    } else if (argument == "--padding") {
      options.padding = std::stod(value);
    } else if (argument == "--dilation-cells") {
      options.dilation_cells = std::stoi(value);
    } else {
      throw std::invalid_argument("Unknown argument: " + argument);
    }
  }

  if (options.input.empty() || options.output_base.empty()) {
    throw std::invalid_argument("--input and --output are required");
  }
  if (options.resolution <= 0.0 || options.min_z > options.max_z ||
      options.min_x > options.max_x || options.min_y > options.max_y ||
      options.padding < 0.0 || options.dilation_cells < 0) {
    throw std::invalid_argument("Invalid map conversion parameters");
  }
  return options;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = ParseArguments(argc, argv);
    using Point = pcl::PointXYZI;
    pcl::PointCloud<Point> cloud;
    if (pcl::io::loadPCDFile<Point>(options.input.string(), cloud) != 0) {
      throw std::runtime_error("Failed to load " + options.input.string());
    }

    std::vector<const Point *> projected_points;
    projected_points.reserve(cloud.size());
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto &point : cloud) {
      if (!pcl::isFinite(point) || point.z < options.min_z ||
          point.z > options.max_z || point.x < options.min_x ||
          point.x > options.max_x || point.y < options.min_y ||
          point.y > options.max_y) {
        continue;
      }
      projected_points.push_back(&point);
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    if (projected_points.empty()) {
      throw std::runtime_error("No points remain in the requested z band");
    }

    const double origin_x =
        std::floor((min_x - options.padding) / options.resolution) *
        options.resolution;
    const double origin_y =
        std::floor((min_y - options.padding) / options.resolution) *
        options.resolution;
    const auto width = static_cast<std::size_t>(
        std::ceil((max_x + options.padding - origin_x) /
                  options.resolution) +
        1.0);
    const auto height = static_cast<std::size_t>(
        std::ceil((max_y + options.padding - origin_y) /
                  options.resolution) +
        1.0);
    if (width == 0 || height == 0 || width > 32768 || height > 32768 ||
        width > std::numeric_limits<std::size_t>::max() / height) {
      throw std::runtime_error("Generated map dimensions are invalid");
    }

    std::vector<std::uint8_t> occupied(width * height, 0);
    for (const Point *point : projected_points) {
      const auto column = static_cast<long>(
          std::floor((point->x - origin_x) / options.resolution));
      const auto row = static_cast<long>(
          std::floor((point->y - origin_y) / options.resolution));
      for (int dy = -options.dilation_cells;
           dy <= options.dilation_cells; ++dy) {
        for (int dx = -options.dilation_cells;
             dx <= options.dilation_cells; ++dx) {
          const long candidate_x = column + dx;
          const long candidate_y = row + dy;
          if (candidate_x >= 0 && candidate_y >= 0 &&
              candidate_x < static_cast<long>(width) &&
              candidate_y < static_cast<long>(height)) {
            occupied[static_cast<std::size_t>(candidate_y) * width +
                     static_cast<std::size_t>(candidate_x)] = 1;
          }
        }
      }
    }

    std::filesystem::path pgm_path = options.output_base;
    pgm_path.replace_extension(".pgm");
    std::filesystem::path yaml_path = options.output_base;
    yaml_path.replace_extension(".yaml");
    if (!pgm_path.parent_path().empty()) {
      std::filesystem::create_directories(pgm_path.parent_path());
    }

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      throw std::runtime_error("Failed to create " + pgm_path.string());
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (std::size_t image_row = 0; image_row < height; ++image_row) {
      const std::size_t grid_row = height - 1 - image_row;
      for (std::size_t column = 0; column < width; ++column) {
        const std::uint8_t pixel =
            occupied[grid_row * width + column] ? 0 : 254;
        pgm.write(reinterpret_cast<const char *>(&pixel), 1);
      }
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) {
      throw std::runtime_error("Failed to create " + yaml_path.string());
    }
    yaml << "image: " << pgm_path.filename().string() << "\n"
         << "mode: trinary\n"
         << "resolution: " << std::fixed << std::setprecision(6)
         << options.resolution << "\n"
         << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n";

    std::cout << "PCD points: " << cloud.size() << "\n"
              << "Projected points: " << projected_points.size() << "\n"
              << "Z band: " << options.min_z << " .. " << options.max_z
              << " m\n"
              << "Map: " << width << " x " << height << " @ "
              << options.resolution << " m/cell\n"
              << "Origin: [" << origin_x << ", " << origin_y << "]\n"
              << "Wrote: " << pgm_path << "\n"
              << "Wrote: " << yaml_path << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "pcd_to_pgm: " << error.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
