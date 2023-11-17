#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "Eigen/Dense"
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <queue>

using namespace std;

#define PI 3.14159265359

int dx_[8] = {1, 0,-1, 0, 1,-1,-1, 1};
int dy_[8] = {0, 1, 0,-1, 1, 1,-1,-1};
// int dx_[] = {1, 0,-1, 0};
// int dy_[] = {0, 1, 0,-1};

/// Struct for grid_cell ////
struct Cell
{
    bool in_openSet = false; // check is in openSet or not 
    bool obstacle = false;
    double near_obs = 0;
    double f_score = 1000;
    double g_score = 1000;
    double h_score = 1000; // score function 
    int x_grid;
    int y_grid;
    Cell* parent = nullptr;

    void set_grid(int i, int j)
    {
        x_grid = i;
        y_grid = j;
    }

    void calculate_h(int x_final_grid, int y_final_grid)
    {
        h_score = sqrt((x_final_grid - x_grid)*(x_final_grid - x_grid) + (y_final_grid - y_grid)*(y_final_grid - y_grid));
        // h_score = abs(x_final_grid - x_grid) + abs(y_final_grid - y_grid);
    }

    void calculate_f()
    {
        f_score = h_score + g_score + near_obs;
    }

    void reset()
    {
        in_openSet = false; // check is in openSet or not 
        obstacle = false;
        near_obs = 0;
        g_score = 1000;
        f_score = 1000;
        parent = nullptr;
    }
};


/// Comparison function ////
class Compare{
    public:
        bool operator()(Cell* a, Cell* b)
        {
            return (a->f_score >= b->f_score);
        }
};

/// Check boundary of grid map
bool check_boundary(int &x, int &y)
{
    return (x<=17 && x>=0 && y<=39 && y>=0);
}

/// Find nearest waypoint to hovercraft
int find_looking_ind(pcl::PointCloud<pcl::PointXYZ>::Ptr path, double x_hover_lidar, double y_hover_lidar)
{
    // First_find_closest index
    int index = 0;
    double min_distance = 999999;
    
    int path_size = path->size();
    for (int i=0; i<path_size; i++)
    {
        double distance = sqrt((x_hover_lidar - path->at(i).x)*(x_hover_lidar - path->at(i).x)
                              +(y_hover_lidar - path->at(i).y)*(y_hover_lidar - path->at(i).y));

        if (distance < min_distance)
        {
            min_distance = distance;
            index = i;
        }
    }

    int look_index = index + 2;
    look_index = min(look_index, path_size-1);

    return look_index;
}

/// A_star path planning
Cell* current_cell;
Cell* neighbor;
pcl::PointCloud<pcl::PointXYZ>::Ptr A_star(vector<vector<Cell*>> grid_map, Cell* start_cell, Cell* final_cell, double x_start_grid_coor, double y_start_grid_coor, double size_cell)
{
    // Create priority queue
    priority_queue<Cell*, vector<Cell*>, Compare> openSet;

    // Create path
    pcl::PointCloud<pcl::PointXYZ>::Ptr path(new pcl::PointCloud<pcl::PointXYZ>());

    // Add start node to open nodes queue 
    start_cell->in_openSet = true;
    start_cell->g_score = 0;
    start_cell->calculate_f();
    openSet.push(start_cell);

    // A* algorithm loop
    while (!openSet.empty())
    {
        // Step1: select current_cell
        current_cell = openSet.top();

        // Step2: Remove current_cell from openSet and mark visited
        current_cell->in_openSet = false;
        openSet.pop();
        

        // Step3: Check whether the current is final_cell -> return path
        if (current_cell->x_grid == final_cell->x_grid && current_cell->y_grid == final_cell->y_grid)
        {
            // Trace back from final_cell to find a path
            while(current_cell != nullptr)
            {
                // convert cell from grid frame to lidar frame 
                pcl::PointXYZ current_point_lidar;
                // current_point_global.x = (current_cell->x_grid - start_cell->x_grid)*size_cell
                //                          + x_start_global + size_cell/2;
                // current_point_global.y = (current_cell->y_grid - start_cell->y_grid)*size_cell
                //                          + y_start_global + size_cell/2;   
                // current_point_global.z = 0;

                current_point_lidar.x = -(current_cell->x_grid - x_start_grid_coor)*size_cell;
                                         
                current_point_lidar.y = -(current_cell->y_grid - y_start_grid_coor + 0.2)*size_cell;
                                            
                current_point_lidar.z = 0;

                path->push_back(current_point_lidar);                                      

                // Change to parent to trace back 
                current_cell = current_cell->parent;
            }

            // Reverse order of path and return            
            std::reverse(path->begin(),path->end());
            return path;
        }

        // Step4: check and add neighbor to wait list
        // for (int dx:dx_)
        // {
        //     for (int dy:dy_)
        //     {
        for (int k=0; k<8; k++)
        {
            int dx = dx_[k];
            int dy = dy_[k];
            int x_neighbor_grid = current_cell->x_grid + dx;
            int y_neighbor_grid = current_cell->y_grid + dy;

            // Check whether it is in boundary 
            if (check_boundary(x_neighbor_grid, y_neighbor_grid))
            {
                neighbor = grid_map[x_neighbor_grid][y_neighbor_grid];
                
                // Skip if neighbor is wall 
                if (neighbor->obstacle)
                {
                    break;
                }

                // if (dx==0 && dy==0)
                // {
                //     break;
                // }

                double step = sqrt(dx*dx + dy*dy); // distance from current to neighbor
                double g_score_temp = current_cell->g_score + step; 
                double f_score_temp = g_score_temp + neighbor->h_score + neighbor->near_obs;

                // Check g_score
                if (g_score_temp <= neighbor->g_score)
                {
                    // set g_Score for neighbor 
                    neighbor->g_score = g_score_temp;
                    // Update f_score 
                    neighbor->calculate_f();
                    // Update parent of neighbor
                    neighbor->parent = current_cell;
                    
                    // Add to openSet
                    if (!neighbor->in_openSet)
                    {
                        neighbor->in_openSet = true;
                        openSet.push(neighbor);   
                    }
                }
            }
        }        
    }

    // Return the empty list 
    return path;
}








class Navigate20200726 : public rclcpp::Node
{
  public:
    Navigate20200726(const char* nodeName)
    : Node(nodeName){
        /// Editing from here is not recommended, but you can do it.
        this->declare_parameter("evalPath", "/home");
        publisher2_ = this->create_publisher<std_msgs::msg::Bool>("/Final/Finished", 10);
        publisher1_ = this->create_publisher<geometry_msgs::msg::Wrench>("/Final/ForceInput/F1", 10);
        publisher0_ = this->create_publisher<geometry_msgs::msg::Wrench>("/Final/ForceInput/F2", 10);
        laser_subs2_ = this->create_subscription<sensor_msgs::msg::LaserScan> ("/ray/laserscan", 10, std::bind(&Navigate20200726::laser_callback, this, std::placeholders::_1));
        point_subs2_ = this->create_subscription<sensor_msgs::msg::PointCloud> ("/ray/pointcloud", 10, std::bind(&Navigate20200726::point_callback, this, std::placeholders::_1));
        this->randomMaze_once();
        /// Editing to here is not recommended, but you can do it.

        /// subscribing imu is optional, if you can do, please utilize it.
        /// imu sensor's topic : "imu/imu_data"
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu> ("/imu/data", 10, std::bind(&Navigate20200726::imu_callback, this, std::placeholders::_1));

        pcl_viewer_on = false;
        if( pcl_viewer_on){
            pcl_viewer2_ = new pcl::visualization::PCLVisualizer("Cloud Viewer");
            pcl_viewer2_->setBackgroundColor(0, 0, 0);
        }

        /// Pointcloud setting 
        cloud_current.reset(new pcl::PointCloud<pcl::PointXYZ>());
        // cloud_previous.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_align.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
        path.reset(new pcl::PointCloud<pcl::PointXYZ>());

        // Setup filter 
        voxel_grid.reset(new pcl::VoxelGrid<pcl::PointXYZ>);
        voxel_grid->setLeafSize(0.3,0.3,0.01);

        /// Matching algorithm setting
        gicp.reset(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
        gicp->setMaxCorrespondenceDistance(0.5);
        gicp->setTransformationEpsilon(0.00002);
        gicp->setMaximumIterations(100); 

        ///Setup Force-Torque matrix 
        FT << -1.0, 0.0, 0.0,
              -0.0, -1.0, -1.0,
              -0.2, 0.4, -0.4;
    }

  private:
    /// randomMaze_once is generating map, i don't recomment you edit this function.
    void randomMaze_once(){
        // Create a client to call the spawn_entity service
        auto client = node_spawn_once->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        // Wait for the service to be available
        if (!client->wait_for_service(std::chrono::seconds(7))) {
            RCLCPP_ERROR(node_spawn_once->get_logger(), "Failed to connect to /spawn_entity service");
            return ;
        }

        // Load and process the Xacro file
        std::string evalPath = this->get_parameter("evalPath").as_string();
        std::string xacroFilePath = evalPath+"/maze.xacro";
        std::string xmlFilePath = evalPath+"/maze.xml";
        std::string command = "ros2 run xacro xacro " + xacroFilePath + " -o " + xmlFilePath;
        int results = system(command.c_str());
        if (results != 0) {
            std::cerr << "Failed to convert xacro file to XML." << std::endl;
            RCLCPP_ERROR(node_spawn_once->get_logger(), "Failed to convert xacro file to XML.");
            return ;
        }
        std::ifstream xmlFile(xmlFilePath);
        if (!xmlFile.is_open()) {
            std::cerr << "Failed to open XML file." << std::endl;
            RCLCPP_ERROR(node_spawn_once->get_logger(), "Failed to open XML file.");
            return ;
        }
        std::string xmlContent;
        std::stringstream xmlStream;
        xmlStream << xmlFile.rdbuf();
        xmlContent = xmlStream.str();

        // remove 6lines - sdf formatting
        size_t pos = xmlContent.find_first_of('\n');
        for (int i = 0; i < 5 && pos != std::string::npos; ++i) {
            pos = xmlContent.find_first_of('\n', pos + 1);
        }
        if (pos != std::string::npos) {
            xmlContent = xmlContent.substr(pos + 1);
        }

        // remove last line
        pos = xmlContent.find_last_of('\n');
        if (pos != std::string::npos) {
            xmlContent = xmlContent.substr(0, pos);
        }
        xmlFile.close();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distribution(9, 12);
        std::uniform_real_distribution<double> distribution_pos(-4, 4);
        std::uniform_real_distribution<double> distribution_hole(-6, 6);

        double randomValue1 = distribution(gen);
        double randomValue2 = distribution_hole(gen);
        double randomValue3 = distribution(gen);
        double randomValue4 = distribution_hole(gen);
        double randomValue5 = distribution(gen);
        double randomPos1 = distribution_pos(gen);
        double randomPos3 = distribution_pos(gen);
        double randomPos5 = distribution_pos(gen);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random value 1 between 9 and 12: %f", randomValue1);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random value 2 between -6 and 6: %f", randomValue2);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random value 3 between 9 and 12: %f", randomValue3);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random value 4 between -6 and 6: %f", randomValue4);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random value 5 between 9 and 12: %f", randomValue5);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random position 1 between -4 and 4: %f", randomPos1);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random position 3 between -4 and 4: %f", randomPos3);
        RCLCPP_INFO(node_spawn_once->get_logger(),"Random position 5 between -4 and 4: %f", randomPos5);

        std::vector<std::pair<std::string, std::string>> replacements = {
                {"link0_w", std::to_string(randomValue1)},
                {"link0_pos_x", std::to_string(randomPos1)},
                {"link0_pos_y", std::to_string(-10.0)},
                {"link11_w", std::to_string(7.5+randomValue2)},
                {"link11_pos_x", std::to_string(-5.25 + 0.5 * randomValue2)},
                {"link11_pos_y", std::to_string(-5.0)},
                {"link12_w", std::to_string(7.5-randomValue2)},
                {"link12_pos_x", std::to_string(5.25 + 0.5 * randomValue2)},
                {"link12_pos_y", std::to_string(-5.0)},
                {"link2_w", std::to_string(randomValue3)},
                {"link2_pos_x", std::to_string(randomPos3)},
                {"link2_pos_y", std::to_string(0.0)},
                {"link31_w", std::to_string(7.5+randomValue4)},
                {"link31_pos_x", std::to_string(-5.25 + 0.5 * randomValue4)},
                {"link31_pos_y", std::to_string(5.0)},
                {"link32_w", std::to_string(7.5-randomValue4)},
                {"link32_pos_x", std::to_string(5.25 + 0.5 * randomValue4)},
                {"link32_pos_y", std::to_string(5.0)},
                {"link4_w", std::to_string(randomValue5)},
                {"link4_pos_x", std::to_string(randomPos5)},
                {"link4_pos_y", std::to_string(10.0)}
        };

        for (const auto& replacement : replacements) {
            const std::string& findStr = replacement.first;
            const std::string& replaceStr = replacement.second;

            size_t pos = xmlContent.find(findStr);
            while (pos != std::string::npos) {
                xmlContent.replace(pos, findStr.length(), replaceStr);
                pos = xmlContent.find(findStr, pos + replaceStr.length());
            }
        }

        auto requests = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        requests->name = "maze";
        requests->xml = xmlContent.c_str();
        auto result = client->async_send_request(requests);

        // Wait for the response
        if (rclcpp::spin_until_future_complete(node_spawn_once, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_spawn_once->get_logger(), "Failed to spawn model");
            return ;
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Update angular velocity 
        gyro_yaw = msg->angular_velocity.z;
    }


    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        /// to do : this is not necessary, you can utilize laser_callback or point_callback or both.
        /// you can utilize LaserScan Data in here.
        /// 1. you might publish "/Final/ForceInput/F1" by publisher1_ along its situation.
        /// type = geometry_msgs::msg::Wrench
        /// 2. you might publish "/Final/ForceInput/F2" by publisher0_ along its situation.
        /// type = geometry_msgs::msg::Wrench
        /// 3. you might publish "/Final/Finished", when you think all mission was cleared.
        /// type = std_msgs::msg::Bool
        return;

    }

    void point_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        /// to do : this is not necessary, you can utilize laser_callback or point_callback or both.
        /// you can utilize PointCloud Data in here.
        /// 1. you might publish "/Final/ForceInput/F1" by publisher1_ along its situation.
        /// type = geometry_msgs::msg::Wrench
        /// 2. you might publish "/Final/ForceInput/F2" by publisher0_ along its situation.
        /// type = geometry_msgs::msg::Wrench
        /// 3. you might publish "/Final/Finished", when you think all mission was cleared.
        /// type = std_msgs::msg::Bool


        ////// Find the coordinate position //////
        rclcpp::Clock clock;
        current_time = clock.now().seconds();

        //////////////////////////////////
        /////////// Step1: UPDATE MAP AND POINT CLOUD ////////////////
        //////////////////////////////////

        /// Update current point_cloud
        cloud_current->clear();
        for (auto input_point: msg->points)
        {
            // double dis = sqrt(input_point.x * input_point.x + input_point.y * input_point.y);
                pcl::PointXYZ point;
                point.x = input_point.x;
                point.y = input_point.y;
                point.z = 0.0;
                cloud_current->push_back(point); // add point to pointcloud      
        }

        // Set size of map and start point, target point for first scan 
        if (start)
        {
            // grid map size
            double width = abs(cloud_current->points[0].x) + abs(cloud_current->points[90].x);
            width_grid = width/grid_size + 2;
            height_grid = 40/grid_size;

            // start point 
            x_start = abs(cloud_current->points[90].x);
            x_start_grid = round(x_start/grid_size);
            x_start_grid = min(x_start_grid, width_grid-1);

            y_start = abs(cloud_current->points[135].y);
            y_start_grid = round(y_start/grid_size);
            y_start_grid = min(y_start_grid, height_grid-1);

            x_start_global = 10.0 - 1 - abs(cloud_current->points[0].x);
            y_start_global = -20.0 + 1 + abs(cloud_current->points[135].y);

            // End point 
            x_final_grid = width_grid/2-1;
            y_final_grid = 33/grid_size;

            RCLCPP_INFO(this->get_logger(), "x_grid : %d", x_start_grid);
            RCLCPP_INFO(this->get_logger(), "y_grid : %d", y_start_grid);
            RCLCPP_INFO(this->get_logger(), "x_start : %f", x_start_global);
            RCLCPP_INFO(this->get_logger(), "y_start : %f", y_start_global);

            // If first scan, set cloud_previous = cloud_current  // 
            // *cloud_previous = *cloud_current;
            *cloud_map = *cloud_current;
            *cloud_align = *cloud_current;

            // Reset grid map
            grid_map.clear();
            for (int i=0; i < width_grid; i++){
                vector<Cell*> col_vec;
                for (int j=0; j < height_grid; j++){
                    Cell* cell_grid(new Cell());
                    cell_grid->set_grid(i,j); // set location of cell
                    cell_grid->calculate_h(x_final_grid,y_final_grid); // find h_score 
                    cell_grid->reset(); // find initial f_score
                    col_vec.push_back(cell_grid);
                }               
                grid_map.push_back(col_vec);
            }
            
            // Change start
            start = false;
        }

        // Update point cloud of map every 10 iterations
         
        int mod = count_ % 20;
        if (mod == 0)
        {
            // Add align point cloud to map 
            for (auto point:*cloud_align)
            {
                cloud_map->push_back(point);
            }
            
            // Filtering point cloud map after update
            voxel_grid->setInputCloud(cloud_map);
            voxel_grid->filter(*cloud_map);

            // Reset grid map 
            for (int i=0; i < width_grid; i++){
                for (int j=0; j < height_grid; j++){
                    grid_map[i][j]->reset();
                }
            }

            // RCLCPP_INFO(this->get_logger(), "x_size : %d", grid_map.size());
            // RCLCPP_INFO(this->get_logger(), "y_size : %d", grid_map[0].size());
            RCLCPP_INFO(this->get_logger(), "RESET_MAP");
            // RCLCPP_INFO(this->get_logger(), "Obstacle: %d", cloud_map->size());

            // Set obstacles for grid map
            for (auto point: *cloud_map)
            {
                int x_grid_obstacle = round(-point.x/grid_size) + x_start_grid; // covert lidar coor to grid coor 
                x_grid_obstacle = min(x_grid_obstacle, width_grid-1); // contraint x_grid
                int y_grid_obstacle = round(-point.y/grid_size) + y_start_grid; // covert lidar coor to grid coor
                y_grid_obstacle = min(y_grid_obstacle, height_grid-1); // contraint y_grid
                
                // add grid in neighbors of obstacles
                for (int dx = -1; dx < 2; dx++)
                {
                    for (int dy = -1; dy < 2; dy++)
                    {
                        int x_neighbor_grid = x_grid_obstacle+dx;
                        int y_neighbor_grid = y_grid_obstacle+dy;

                        if (check_boundary(x_neighbor_grid, y_neighbor_grid))
                        {
                            // grid_map[x_neighbor_grid][y_neighbor_grid]->obstacle = true;
                            // grid_map[x_neighbor_grid][y_neighbor_grid]->near_obs = 1000000;
                            double temp_near_obs;
                            if (dx == 0)
                                if (dy == 0)
                                {
                                    temp_near_obs = 1000000;
                                }
                                else if (dy == -1)
                                {
                                    temp_near_obs = 1000;
                                }                            
                                else
                                {
                                    temp_near_obs = 15;
                                }
                            else
                            {
                                temp_near_obs = 15;
                            }


                            if (temp_near_obs > grid_map[x_neighbor_grid][y_neighbor_grid]->near_obs)
                            {
                                grid_map[x_neighbor_grid][y_neighbor_grid]->near_obs = temp_near_obs;
                            }
                        }
                    }
                }
            }

            /// Find new path-planning 
            /// Set start point as hovercraft location 
            int x_hover_grid = round(-matrix(0,3)/grid_size) + x_start_grid; // convert lidar coor to grid coor
            int y_hover_grid = round(-matrix(1,3)/grid_size) + y_start_grid;

            int x_target_grid = x_hover_grid; // convert lidar coor to grid coor
            int y_target_grid = y_hover_grid + 20;

            y_target_grid = min(y_target_grid,35);

            RCLCPP_INFO(this->get_logger(), "x_hover_ : %d", x_hover_grid);
            RCLCPP_INFO(this->get_logger(), "y_hover_ : %d", y_hover_grid);

            path = A_star(grid_map, // map
                        grid_map[x_hover_grid][y_hover_grid], // start cell
                        grid_map[x_final_grid][y_target_grid], // final cell
                        x_start_grid, 
                        y_start_grid,
                        grid_size);

            RCLCPP_INFO(this->get_logger(), "path_size : %d", path->size());
        }

        ////////////////////////////////////////////////
        /////////// Step2: LOCALIZATION ////////////////
        ////////////////////////////////////////////////

        // Matching point cloud to find localization of hovercraft in laser coordinate 
        voxel_grid->setInputCloud(cloud_current);
        voxel_grid->filter(*cloud_current);

        gicp->setInputSource(cloud_current);
        gicp->setInputTarget(cloud_map);
        gicp->align(*cloud_align,matrix);
        matrix = gicp->getFinalTransformation();

        // Visualize point cloud 
        if(pcl_viewer_on){
            // pcl_viewer2_->addPointCloud<pcl::PointXYZ>(cloud_align,"cloud_in");
            // pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_in");
            // pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR , 1.00, 0.00, 0.00, "cloud_in");
            pcl_viewer2_->addPointCloud<pcl::PointXYZ>(cloud_map,"cloud_target");
            pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_target");
            pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR , 0.00, 1.00, 0.00, "cloud_target");
            pcl_viewer2_->addPointCloud<pcl::PointXYZ>(path,"path");
            pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "path");
            pcl_viewer2_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR , 0.00, 0.00, 1.00, "path");

            pcl_viewer2_->spinOnce();
            pcl_viewer2_->removeAllPointClouds();
        }

        // Find localization of car
        // Convert lidar coor to global coor
        x_global = -matrix(0,3) + x_start_global; // because the coordinate of lidar is rotate 180
        y_global = -matrix(1,3) + y_start_global; // add the translation of start position 
        double cos_theta = matrix(0,0);
        double sin_theta = matrix(1,0);
        yaw = atan2(sin_theta,cos_theta); 
        
        // Estimate velocity of hovercraft
        if (count_ == 0)
        {
            vel_x = 0.0;
            vel_y = 0.0;
        }
        else
        {
            vel_x = 0.5*vel_x + 0.5*(x_global - x_global_previous)/(current_time - previous_time);
            vel_y = 0.5*vel_y + 0.5*(y_global - y_global_previous)/(current_time - previous_time);
        }

        // Setup point clouds for next iteration
        // *cloud_previous = *cloud_current; 
        previous_time = current_time;
        x_global_previous = x_global;
        y_global_previous = y_global;


        ///////////////////////////////////////////////////////////
        /////////// Step3: FIND LOOKING AHEAD WAYPOINT ////////////////
        /////////////////////////////////////////////////////////////
        double look_wp_x_global;
        double look_wp_y_global;

        if (!path->empty())
        {
            int look_wp_ind = find_looking_ind(path,matrix(0,3),matrix(1,3));

            // transform location of looking waypoint from lidar coordinate to global coordinate 
            look_wp_x_global = -path->at(look_wp_ind).x + x_start_global;
            look_wp_y_global = -path->at(look_wp_ind).y + y_start_global;
        }
        else
        {
            look_wp_x_global = x_global;
            look_wp_y_global = y_global;
        }

        /////////////////////////////////////////////////////////////////
        /////////// Step4: ERROR IN GLOBAL AND BODY FRAME////////////////
        /////////////////////////////////////////////////////////////////
        
        // double error_x_global = x_start_global - 3 - x_global;
        // double error_y_global = y_start_global + 0 - y_global;

        double error_x_global = look_wp_x_global - x_global;
        double error_y_global = look_wp_y_global - y_global;
        double error_yaw = 0.0 - yaw;

        double error_x_local = cos(yaw)*error_x_global + sin(yaw)*error_y_global;
        double error_y_local = -sin(yaw)*error_x_global + cos(yaw)*error_y_global;
                
        //////////////////////////////////
        /////////// Step5: BODY FORCE ////////////////
        //////////////////////////////////
        double Fx = 5.0*error_x_local - 20.0*vel_x;   // Kp 0.5 0.8, Kd 3.2
        double Fy = 5.0*error_y_local - 20.0*vel_y;
        double M = 0.2*error_yaw - 0.628*gyro_yaw;

        /// Add force from potential field 
        // double dis_sq_min = 9999999;
        // int index = 0;
        // for (int i = 0; i<cloud_current->size(); i++)
        // {
        //     double dis_sq = cloud_current->at(i).x*cloud_current->at(i).x + cloud_current->at(i).y*cloud_current->at(i).y;
        //     if (dis_sq < dis_sq_min)
        //     {
        //         dis_sq_min = dis_sq;
        //         index = i;
        //     }
        // }
        for (auto point: *cloud_current)
        {
            double dis_sq_min = point.x*point.x + point.y*point.y;
            if (sqrt(dis_sq_min) < 1.6)
            {
                // double force = -0.2*1/((sqrt(dis_sq_min)-0.2)*(sqrt(dis_sq_min)-0.2)); // distance: 1.5  K = 0.2 
                double force = -0.8*1/(sqrt(dis_sq_min))*(sqrt(dis_sq_min));
                double angle = atan2(point.y, point.x);
                Fx -= force*cos(angle);
                Fy -= force*sin(angle); 
                RCLCPP_INFO(this->get_logger(), "POTENTIAL FIELD");
            }

        }
        Eigen::Vector3f A(Fx,Fy,M);
        
        /////////////////////////////////////////////////////////////////
        /////////// Step6: PUBLISH FORCE FOR EACH MOTORS ////////////////
        /////////////////////////////////////////////////////////////////
        Eigen::Vector3f B = FT.inverse()*A;

        auto Wrench_msg_1 = geometry_msgs::msg::Wrench(); //LEFT
        Wrench_msg_1.force.x = min(double(B(0)),10.0);
        Wrench_msg_1.force.y = min(double(B(1)),10.0);
        publisher1_->publish(Wrench_msg_1);

        auto Wrench_msg_0 = geometry_msgs::msg::Wrench(); // RIGHT
        Wrench_msg_0.force.x = 0;
        Wrench_msg_0.force.y = min(double(B(2)),10.0);
        publisher0_->publish(Wrench_msg_0);
        // RCLCPP_INFO(this->get_logger(), "Mode : %f", Wrench_msg.force.y);

        /////////////////////////////////////////////////////////////////
        /////////// Step7: PUBLISH FINISH STATE /////////////////////////
        /////////////////////////////////////////////////////////////////
        std_msgs::msg::Bool finish; 
        finish.data = false;
        if (y_global > 10.0)
        {
            finish.data = true;
            RCLCPP_INFO(this->get_logger(), "DONEEEEEEE");
        }
        publisher2_->publish(finish);
        

        ///////// UPDATE COUNT //////////////
        count_++;

    }

    /// variables - editing not recommended
    bool pcl_viewer_on;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher2_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher0_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_subs2_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subs2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Node::SharedPtr node_spawn_once = std::make_shared<rclcpp::Node>("spawn_model_student_node");
    pcl::visualization::PCLVisualizer *pcl_viewer2_;


    /// Point cloud setup 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current; // current scan 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous; // previous scan 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align; // align point cloud  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map; // map point cloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr path;

    /// Filter and Matching setup 
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp;
    pcl::VoxelGrid<pcl::PointXYZ>::Ptr voxel_grid;
    bool start = true;
    int count_ = 0;

    // Localization of hover in lidar coordinate setup 
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();  

    // Map parameter
    double grid_size = 1; // m/grid
    int width_grid;
    int height_grid;
    double x_start;
    double y_start;
    int x_start_grid;
    int y_start_grid;
    double x_final;
    double y_final;
    int x_final_grid;
    int y_final_grid;

    double x_start_global;
    double y_start_global;
    double x_final_global;
    double y_final_global;

    vector<vector<Cell*>> grid_map;

    // State of hovercraft 
    double x_global, x_global_previous;
    double y_global, y_global_previous;
    double yaw;
    double gyro_yaw;
    double vel_x, vel_y;
    double current_time = 0;
    double previous_time = 0;


    // Force-Torque convert matrix 
    Eigen::Matrix3f FT;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigate20200726>("Navigate20200726"));
  rclcpp::shutdown();
  current_cell = nullptr;
  neighbor = nullptr;
  delete current_cell;
  delete neighbor;
  return 0;
}




