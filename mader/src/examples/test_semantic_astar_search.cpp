/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <Eigen/Dense>
#include "semantic_astar.hpp"
#include "mader_types.hpp"
#include "utils.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros

Eigen::Vector3d getPosDynObstacle(double t)
{
  double t_scaled = 4.3 * t;

  Eigen::Vector3d pos;
  pos << sin(t_scaled) + 2 * sin(2 * t_scaled),  //////////////////////
      cos(t_scaled) - 2 * cos(2 * t_scaled),     //////////////////////
      -sin(3 * t_scaled);                        //////////////////////

  pos.x() = pos.x() / 2.5;  // scales down the obstacle's motion.
  pos.y() = pos.y() / 2.5;
  pos.z() = pos.z() / 1.5;

  pos.z() = pos.z() + 1.0;  //shifts the obstacle's z-position upward by 1.0 unit.

  return pos;
}

// Function to get the voxel index of a node
Eigen::Vector3i getNodeVoxelIndex(const Eigen::Vector3d& node_qi, double voxel_size, const Eigen::Vector3d& voxel_origin) {
  Eigen::Vector3i voxel_index;
  voxel_index.x() = std::round((node_qi.x() - voxel_origin.x()) / voxel_size);
  voxel_index.y() = std::round((node_qi.y() - voxel_origin.y()) / voxel_size);
  voxel_index.z() = std::round((node_qi.z() - voxel_origin.z()) / voxel_size);
  return voxel_index;
}

std::pair<std::vector<Eigen::Vector3i>, visualization_msgs::MarkerArray> createVoxelRegion(
    const Eigen::Vector3d& region_origin, double region_bbox_x, double region_bbox_y, double region_bbox_z, Eigen::Vector3d color,
    double voxel_size, int id_start) {
    std::vector<Eigen::Vector3i> voxel_region;
    visualization_msgs::MarkerArray voxel_region_markers;

    // Calculate the min and max corners of the region
    //region_min is now the origin of the region
    Eigen::Vector3d region_min = region_origin;
    Eigen::Vector3d region_max = region_origin + Eigen::Vector3d(region_bbox_x, region_bbox_y, region_bbox_z);

    // Calculate the voxel indices for the min and max corners
    Eigen::Vector3i voxel_min_index = getNodeVoxelIndex(region_min, voxel_size, region_origin);
    Eigen::Vector3i voxel_max_index = getNodeVoxelIndex(region_max, voxel_size, region_origin);

    // Iterate through the voxel indices within the region
    int id = id_start;
    for (int x = voxel_min_index.x(); x < voxel_max_index.x(); ++x) {
        for (int y = voxel_min_index.y(); y < voxel_max_index.y(); ++y) {
            for (int z = voxel_min_index.z(); z < voxel_max_index.z(); ++z) {
                voxel_region.push_back(Eigen::Vector3i(x, y, z));

                // Create a marker for each voxel
                visualization_msgs::Marker voxel_marker;
                voxel_marker.header.frame_id = "world";
                voxel_marker.header.stamp = ros::Time::now();
                voxel_marker.ns = "voxel_region";
                voxel_marker.id = id++;
                voxel_marker.type = visualization_msgs::Marker::CUBE;
                voxel_marker.action = visualization_msgs::Marker::ADD;
                voxel_marker.pose.position.x = region_origin.x() + (x + 0.5) * voxel_size;
                voxel_marker.pose.position.y = region_origin.y() + (y + 0.5) * voxel_size;
                voxel_marker.pose.position.z = region_origin.z() + (z + 0.5) * voxel_size;
                voxel_marker.pose.orientation.w = 1.0;
                voxel_marker.scale.x = voxel_size;
                voxel_marker.scale.y = voxel_size;
                voxel_marker.scale.z = voxel_size;
                voxel_marker.color.a = 0.3; // Make it slightly transparent
                voxel_marker.color.r = color.x();
                voxel_marker.color.g = color.y();
                voxel_marker.color.b = color.z();
                voxel_region_markers.markers.push_back(voxel_marker);
            }
        }
    }

    return std::make_pair(voxel_region, voxel_region_markers);
}
//
// Function to calculate the mean distance from a node to a voxel region
double calculateMeanDistanceToVoxelRegion(const Eigen::Vector3d& node_qi,
                                          const std::vector<Eigen::Vector3i>& voxel_region,
                                          double voxel_size,
                                          const Eigen::Vector3d& voxel_origin) {
  if (voxel_region.empty()) {
    return 0.0; // Handle the case where the voxel region is empty
  }

  std::vector<double> distances;
  for (const auto& voxel_index : voxel_region) {
    // Calculate the center of the voxel
    Eigen::Vector3d voxel_center;
    voxel_center.x() = voxel_origin.x() + (voxel_index.x() + 0.5) * voxel_size;
    voxel_center.y() = voxel_origin.y() + (voxel_index.y() + 0.5) * voxel_size;
    voxel_center.z() = voxel_origin.z() + (voxel_index.z() + 0.5) * voxel_size;

    // Calculate the distance between the node and the voxel center
    double distance = (node_qi, voxel_center).norm();
    distances.push_back(distance);
  }

  // Calculate the mean distance
  double sum_of_distances = std::accumulate(distances.begin(), distances.end(), 0.0);
  double mean_distance = sum_of_distances / distances.size();

  return mean_distance;
}
//

ConvexHullsOfCurve createStaticObstacle(visualization_msgs::Marker& stat_obs_ma, double x, double y, double z, int num_pol, double bbox_x, double bbox_y, double bbox_z, int id)
{
  ConvexHullsOfCurve hulls_curve;
  std::vector<Point_3> points;

  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));

  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));

  CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points);

  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull_interval);  // static obstacle
  }

  //
  stat_obs_ma.type = visualization_msgs::Marker::CUBE;
  stat_obs_ma.header.frame_id = "world";
  stat_obs_ma.header.stamp = ros::Time::now();
  stat_obs_ma.ns = "marker_stat_obs";
  stat_obs_ma.action = visualization_msgs::Marker::ADD;
  stat_obs_ma.scale.x = bbox_x;
  stat_obs_ma.scale.y = bbox_y;  // rviz complains if not
  stat_obs_ma.scale.z = bbox_z;  // rviz complains if not
  stat_obs_ma.pose.position.x = x;
  stat_obs_ma.pose.position.y = y;
  stat_obs_ma.pose.position.z = z;
  stat_obs_ma.pose.orientation.w = 1.0;
  stat_obs_ma.id = id;
  stat_obs_ma.color.g = 0.3;
  stat_obs_ma.color.b = 0.0;
  stat_obs_ma.color.r = 0.6;
  stat_obs_ma.color.a = 1.0;

  return hulls_curve;
}

visualization_msgs::Marker getMarker(Eigen::Vector3d& center, double bbox_x, double bbox_y, double bbox_z, double t_min,
                                     double t_final, double t, int id)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::CUBE;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "marker_dyn_obs";
  m.action = visualization_msgs::Marker::ADD;
  m.id = id;
  m.color = mu::getColorJet(t, t_min, t_final);
  m.scale.x = bbox_x;
  m.scale.y = bbox_y;  // rviz complains if not
  m.scale.z = bbox_z;  // rviz complains if not
  m.pose.position.x = center.x();
  m.pose.position.y = center.y();
  m.pose.position.z = center.z();
  m.pose.orientation.w = 1.0;
  return m;

  // visualization_msgs::MarkerArray marker_array;

  // if (data.size() == 0)
  // {
  //   return marker_array;
  // }

  // int j = 9000;
  // for (int i = 0; i < data.size(); i = i + 1)
  // {
  //   visualization_msgs::Marker m;
  //   m.type = visualization_msgs::Marker::CUBE;
  //   m.header.frame_id = "world";
  //   m.header.stamp = ros::Time::now();
  //   m.ns = ns;
  //   m.action = visualization_msgs::Marker::ADD;
  //   m.id = j;
  //   m.color = mu::getColorJet(t, t_min, t_final);
  //   m.scale.x = bbox_x;
  //   m.scale.y = bbox_y;  // rviz complains if not
  //   m.scale.z = bbox_z;  // rviz complains if not
  //   m.pose.position.x = data[i].x();
  //   m.pose.position.y = data[i].y();
  //   m.pose.position.z = data[i].z();
  //   m.pose.orientation.w = 1.0;

  //   marker_array.markers.push_back(m);
  //   j = j + 1;
  // }
  // return marker_array;
}

ConvexHullsOfCurve createDynamicObstacle(std::vector<visualization_msgs::MarkerArray>& ma_vector, double x, double y,
                                         double z, int num_pol, double bbox_x, double bbox_y, double bbox_z,
                                         double t_min, double t_max)
{
  double time_per_interval = (t_max - t_min) / num_pol;
  double time_discretization = 0.01;

  ConvexHullsOfCurve hulls_curve;

  ma_vector.clear();
  // ma.markers.clear();

  int j = 0;
  for (int interval_index = 0; interval_index < num_pol; interval_index++)
  {
    std::vector<Point_3> points_interval;

    visualization_msgs::MarkerArray ma;
    std::cout << "--------------Interval " << interval_index << std::endl;
    for (double t = interval_index * time_per_interval; t < (interval_index + 1) * time_per_interval;
         t = t + time_discretization)
    {
      Eigen::Vector3d current_pos = getPosDynObstacle(t);
      x = current_pos.x();
      y = current_pos.y();
      z = current_pos.z();

      double bbox_x_infl = bbox_x + 0.03;
      double bbox_y_infl = bbox_y + 0.03;
      double bbox_z_infl = bbox_z + 0.03;

      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));

      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));

      ma.markers.push_back(getMarker(current_pos, bbox_x, bbox_y, bbox_z, t_min, t_max, t, j));
      j = j + 1;
    }

    ma_vector.push_back(ma);

    CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points_interval);
    hulls_curve.push_back(hull_interval);
  }
  return hulls_curve;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testSemanticAwareAStar");
  ros::NodeHandle nh("~");
  ros::Publisher trajectories_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/sea_trajectories_found", 1000, true);

  ros::Publisher best_trajectory_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/sea_best_trajectory_found", 1000, true);
  ros::Publisher convex_hulls_pub = nh.advertise<visualization_msgs::Marker>("/convex_hulls", 1, true);

  ros::Publisher static_obs_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/sea_static_obs", 1000, true);

  std::vector<ros::Publisher> jps_poly_pubs;  // = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_jps", 1, true);
  std::vector<ros::Publisher> traj_obstacle_colored_pubs;
  std::vector<ros::Publisher> best_trajectory_found_intervals_pubs;

  int num_pol = 7;  //origin=7
  int deg_pol = 3;
  visualization_msgs::Marker stat_obs_ma;
  visualization_msgs::MarkerArray stat_obs_ma_array;

  for (int i = 0; i < num_pol; i++)
  {
    ros::Publisher tmp =
        nh.advertise<visualization_msgs::MarkerArray>("/sea_traj_obstacle_colored_int_" + std::to_string(i), 1, true);
    traj_obstacle_colored_pubs.push_back(tmp);

    ros::Publisher tmp2 = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/sea_poly_jps_int_" + std::to_string(i), 1, true);
    jps_poly_pubs.push_back(tmp2);

    ros::Publisher tmp3 =
        nh.advertise<visualization_msgs::MarkerArray>("/sea_best_trajectory_found_int_" + std::to_string(i), 1, true);
    best_trajectory_found_intervals_pubs.push_back(tmp3);
  }

  // ros::Publisher traj_obstacle_colored_int0_pub =
  //     nh.advertise<visualization_msgs::MarkerArray>("traj_obstacle_colored_int0", 1, true);

  std::string basis;

  nh.getParam("basis", basis);

  std::cout << "Basis= " << basis << std::endl;

  int samples_x = 5;  // odd number
  int samples_y = 5;  // odd number
  int samples_z = 5;  // odd number

  double alpha_shrink = 0.9;

  double fraction_voxel_size = 0.0;  // grid used to prune nodes that are on the same cell

  double runtime = 0.1;    //[seconds]
  double goal_size = 0.1;  //[meters]

  Eigen::Vector3d v_max(7.0, 7.0, 7.0);
  Eigen::Vector3d a_max(400000.0, 4000000.0, 4000000.0);

  Eigen::Vector3d q0(-3, 0.5, 1);
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;
  Eigen::Vector3d goal(5.0, 0, 1);

  double t_min = 0.0;
  double t_max = t_min + (goal - q0).norm() / (0.8 * v_max(0));

  std::cout << "t_min= " << t_min << std::endl;
  std::cout << "t_max= " << t_max << std::endl;
  std::cout << "=========================" << std::endl;

  ConvexHullsOfCurves hulls_curves;

  //dimensions of dynamic obstacles
  double bbox_x = 0.4;
  double bbox_y = 0.4;
  double bbox_z = 0.4;

  //dimensions of static obstacles
  double stat_obs_bbox_x = 0.4;
  double stat_obs_bbox_y = 0.4;
  double stat_obs_bbox_z = 1.0;

  double dc = 0.002;  // Simply used for visualization

  int num_of_obs = 1;  // odd number, origin=1
  double separation = 0.4;

  int num_of_obs_up = (num_of_obs - 1) / 2.0;

  std::vector<visualization_msgs::MarkerArray> ma_vector;

  // ConvexHullsOfCurve hulls_curve = createStaticObstacle(stat_obs_ma, 0.0,stat_obs_bbox_y + separation, 0.0, num_pol, stat_obs_bbox_x, stat_obs_bbox_y, stat_obs_bbox_z, 0);
  ConvexHullsOfCurve hulls_curve =
      createDynamicObstacle(ma_vector, 0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z, t_min, t_max);
  hulls_curves.push_back(hulls_curve);  // only one obstacle

  //create pairs of static obstacles mirrored across the y-axis
  //they separate themselves by 'separation' variable
  int stat_obs_id = 0;
  for (int i = 1; i <= num_of_obs_up; i++)
  {
    ConvexHullsOfCurve hulls_curve =
        createStaticObstacle(stat_obs_ma, 0.0, i * (stat_obs_bbox_y + separation), 0.0, num_pol, stat_obs_bbox_x, stat_obs_bbox_y, stat_obs_bbox_z, stat_obs_id++);
    hulls_curves.push_back(hulls_curve);  // only one obstacle
    stat_obs_ma_array.markers.push_back(stat_obs_ma);

    hulls_curve = createStaticObstacle(stat_obs_ma, 0.0, -i * (stat_obs_bbox_y + separation), 0.0, num_pol, stat_obs_bbox_x, stat_obs_bbox_y, stat_obs_bbox_z,stat_obs_id++);
    hulls_curves.push_back(hulls_curve);  // only one obstacle
    stat_obs_ma_array.markers.push_back(stat_obs_ma);
  }

  for (int i = 0; i < ma_vector.size(); i++)
  {
    traj_obstacle_colored_pubs[i].publish(ma_vector[i]);
  }

  std::cout << "hulls_curves.size()= " << hulls_curves.size() << std::endl;

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);
  // vec_E<Polyhedron<3>> jps_poly = cu::vectorGCALPol2vectorJPSPol(hulls_curves);

  for (int i = 0; i < num_pol; i++)
  {
    ConvexHullsOfCurve tmp2;
    ConvexHullsOfCurves tmp;

    tmp2.push_back(hulls_curve[i]);
    tmp.push_back(tmp2);

    // convert the obstacles polyhedron arrays
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(cu::vectorGCALPol2vectorJPSPol(tmp));
    poly_msg.header.frame_id = "world";
    jps_poly_pubs[i].publish(poly_msg);
  }

  // hull.push_back(Eigen::Vector3d(-1.0, -1.0, -700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, -1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, 1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, 1.0, -700.0));

  // hull.push_back(Eigen::Vector3d(1.0, 1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(1.0, 1.0, -700.0));
  // hull.push_back(Eigen::Vector3d(1.0, -1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(1.0, -1.0, -700.0));

  // Assummes static obstacle
  /*  for (int i = 0; i < num_pol; i++)
    {
      hulls_curve.push_back(hull);
    }

    hulls_curves.push_back(hulls_curve);*/
  //// voxels region
  Eigen::Vector3d region_origin(0.0, 3.0, 0.0);
  double voxel_size = 0.1;
  double region_bbox_x = 1.0;
  double region_bbox_y = 1.0;
  double region_bbox_z = voxel_size;
  Eigen::Vector3d region_color(0.0, 1.0, 0.0);
  int id_start = 0;
  
  std::pair<std::vector<Eigen::Vector3i>, visualization_msgs::MarkerArray> result =
      createVoxelRegion(region_origin, region_bbox_x, region_bbox_y, region_bbox_z, region_color, voxel_size, id_start);

  std::vector<Eigen::Vector3i> voxel_region = result.first;
  std::pair<Eigen::Vector3d, std::vector<Eigen::Vector3i>> att_region
      = std::make_pair(region_origin, voxel_region);

  visualization_msgs::MarkerArray voxel_region_markers = result.second;
  ros::Publisher voxel_region_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxel_region", 1000, true);
  voxel_region_pub.publish(voxel_region_markers);
  // Testing: Calculate the mean distance
  // Eigen::Vector3d node_qi(0.0, 0.0, 0.0);//(1.5, 0.2, 0.8); // Example node position
  // double mean_distance = calculateMeanDistanceToVoxelRegion(node_qi, voxel_region, voxel_size, region_origin);

  // std::cout << "Mean distance from node to voxel region: " << mean_distance << std::endl;

  ////
  SemanticAstar mySemAstarSolver(basis, num_pol, deg_pol, alpha_shrink);
  mySemAstarSolver.setUp(t_min, t_max, hulls_std);

  mySemAstarSolver.setq0q1q2(q0, q1, q2);
  mySemAstarSolver.setGoal(goal);

  mySemAstarSolver.setXYZMinMaxAndRa(-1e6, 1e6, -1e6, 1e6, -1.0, 10.0, 1e6);  // limits for the search, in world frame
  mySemAstarSolver.setBBoxSearch(30.0, 30.0, 30.0);                           // limits for the search, centered on q2
  mySemAstarSolver.setMaxValuesAndSamples(v_max, a_max, samples_x, samples_y, samples_z, fraction_voxel_size);
  // std::cout << "The size of voxel = " << voxel_size_ <<std::endl;

  mySemAstarSolver.setRunTime(runtime);
  mySemAstarSolver.setGoalSize(goal_size);

  //set weights
  mySemAstarSolver.setBias(1.0);
  mySemAstarSolver.setAttWeight(3.0);//3.0
  mySemAstarSolver.setRepWeight(0.0);
  mySemAstarSolver.setAttRegion(att_region);

  mySemAstarSolver.setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool solved = mySemAstarSolver.run(q, n, d);

  // Recover all the trajectories found and the best trajectory
  std::vector<mt::trajectory> all_trajs_found;
  mySemAstarSolver.getAllTrajsFound(all_trajs_found);

  mt::trajectory best_traj_found;
  mt::PieceWisePol pwp_best_traj_found;
  mySemAstarSolver.getBestTrajFound(best_traj_found, pwp_best_traj_found, dc);

  // for (double t = t_min; t <= t_max; t = t + dc)
  // {
  //   Eigen::Vector3d pos = pwp_best_traj_found.eval(t);
  //   std::cout << "At t= " << t << " pos=" << pos.transpose() << " pos_obstacle=" << getPosDynObstacle(t).transpose()
  //             << std::endl;
  // }

  // Convert to marker arrays
  //---> all the trajectories found
  int increm = 2;
  int increm_best = 1;
  double scale = 0.02;
  int j = 0;

  visualization_msgs::MarkerArray marker_array_all_trajs;
  for (auto traj : all_trajs_found)
  {
    visualization_msgs::MarkerArray marker_array_traj = mu::trajectory2ColoredMarkerArray(
        traj, v_max.maxCoeff(), increm, "traj" + std::to_string(j), scale, "time", 0, 1);
    // std::cout << "size of marker_array_traj= " << marker_array_traj.markers.size() << std::endl;
    for (auto marker : marker_array_traj.markers)
    {
      marker_array_all_trajs.markers.push_back(marker);
    }
    j++;
  }

  //---> the best trajectory found
  scale = 0.15;
  visualization_msgs::MarkerArray marker_array_best_traj;
  marker_array_best_traj = mu::trajectory2ColoredMarkerArray(best_traj_found, v_max.maxCoeff(), increm_best,
                                                             "traj" + std::to_string(j), scale, "time", 0, 1);

  double entries_per_interval = marker_array_best_traj.markers.size() / num_pol;
  for (int i = 0; i < num_pol; i++)
  {
    std::vector<visualization_msgs::Marker> tmp(                                 /////////
        marker_array_best_traj.markers.begin() + i * entries_per_interval,       /////////
        marker_array_best_traj.markers.begin() + (i + 1) * entries_per_interval  /////////
    );

    visualization_msgs::MarkerArray ma;
    ma.markers = tmp;
    best_trajectory_found_intervals_pubs[i].publish(ma);
  }

  best_trajectory_found_pub.publish(marker_array_best_traj);

  // Get the edges of the convex hulls and publish them
  mt::Edges edges_convex_hulls;
  mySemAstarSolver.getEdgesConvexHulls(edges_convex_hulls);
  convex_hulls_pub.publish(mu::edges2Marker(edges_convex_hulls, mu::color(mu::red_normal)));

  // publish the trajectories
  trajectories_found_pub.publish(marker_array_all_trajs);

  // publish the static obstacles
  static_obs_pub.publish(stat_obs_ma_array);

  /*----- octopus search -----*/

  //
  ros::spinOnce();

  /*
    vectorOfNodes2vectorOfStates()

        traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max, increm, name_drone_);
    pub_traj_committed_colored_.publish(traj_committed_colored_);*/

  // if (solved == true)
  // {
  //   std::cout << "This is the result" << std::endl;
  //   for (auto qi : q)
  //   {
  //     std::cout << qi.transpose() << std::endl;
  //   }
  // }
  // else
  // {
  //   std::cout << "A* didn't find a solution" << std::endl;
  // }

  // std::cout << "Normal Vectors: " << std::endl;
  // for (auto ni : n)
  // {
  //   std::cout << ni.transpose() << std::endl;
  // }

  // std::cout << "D coefficients: " << std::endl;
  // for (auto di : d)
  // {
  //   std::cout << di << std::endl;
  // }

  ros::spin();

  return 0;
}
