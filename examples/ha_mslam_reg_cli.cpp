#include "back_end/reglib.h"
#include "global_definition/global_definition.h"
#include "utils/config.h"

#include <pcl/io/pcd_io.h>

#include <Eigen/Core>

#include <iomanip>
#include <iostream>
#include <string>

namespace {

void printMatrix(const Eigen::Matrix4d & matrix)
{
  std::cout << "[";
  for (int row = 0; row < 4; ++row) {
    if (row != 0) {
      std::cout << ",";
    }
    std::cout << "[";
    for (int column = 0; column < 4; ++column) {
      if (column != 0) {
        std::cout << ",";
      }
      std::cout << matrix(row, column);
    }
    std::cout << "]";
  }
  std::cout << "]";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: ha_mslam_reg_cli CONFIG SOURCE_PCD TARGET_PCD\n";
    return 2;
  }

  const std::string config_path =
    g3reg::WORK_SPACE_PATH + "/" + std::string(argv[1]);
  g3reg::InitGLOG(config_path, argv);
  g3reg::config.load_config(config_path, argv);

  pcl::PointCloud<pcl::PointXYZ>::Ptr source(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(
    new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *source) == -1) {
    std::cerr << "Failed to read source PCD\n";
    return 3;
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *target) == -1) {
    std::cerr << "Failed to read target PCD\n";
    return 4;
  }

  const FRGresult result =
    g3reg::GlobalRegistration(source, target);

  std::cout << std::setprecision(17);
  std::cout << "{";
  std::cout << "\"valid\":" << (result.valid ? "true" : "false") << ",";
  std::cout << "\"source_points\":" << source->size() << ",";
  std::cout << "\"target_points\":" << target->size() << ",";
  std::cout << "\"inlier_count\":" << result.inliers.rows() << ",";
  std::cout << "\"plane_inliers\":" << result.plane_inliers << ",";
  std::cout << "\"line_inliers\":" << result.line_inliers << ",";
  std::cout << "\"cluster_inliers\":" << result.cluster_inliers << ",";
  std::cout << "\"feature_time_ms\":" << result.feature_time << ",";
  std::cout << "\"graph_time_ms\":" << result.graph_time << ",";
  std::cout << "\"clique_time_ms\":" << result.clique_time << ",";
  std::cout << "\"tf_solver_time_ms\":" << result.tf_solver_time << ",";
  std::cout << "\"verify_time_ms\":" << result.verify_time << ",";
  std::cout << "\"total_time_ms\":" << result.total_time << ",";
  std::cout << "\"T_target_source\":";
  printMatrix(result.tf);
  std::cout << ",\"candidates\":[";
  for (std::size_t index = 0; index < result.candidates.size(); ++index) {
    if (index != 0U) {
      std::cout << ",";
    }
    printMatrix(result.candidates[index]);
  }
  std::cout << "]}\n";

  return result.valid ? 0 : 5;
}
