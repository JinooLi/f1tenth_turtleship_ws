#include "local_costmap_generator/local_costmap_generator.hpp"

LocalCostmapGenerator::LocalCostmapGenerator() : Node("local_costmap_generator_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // subscriber for laser scan
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LocalCostmapGenerator::scanCallback, this, std::placeholders::_1)
    );

    // publisher for costmap
    costmapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "cost_map", 10
    );
    occupancyGridPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "occupancy_grid_map", 10
    );
    
    // flag for if received scan
    isScanReceived_ = false;

    // timer_callback함수를 timerPeriod(ms)마다 호출한다.
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timerPeriod), std::bind(&LocalCostmapGenerator::timerCallback, this));

    // LaserProjection Object used for method `projectLaser`
    laserProjection_ = std::make_shared<laser_geometry::LaserProjection>();

    // pointcloud2 object
    pointCloud2_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // pcl object
    pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // grid map object
    costmap_ = new grid_map::GridMap({"collision_layer"});
    costmap_->setGeometry(grid_map::Length(gridLength, gridLength), resolution, grid_map::Position(0.0, 0.0));
    costmap_->get("collision_layer").setConstant(0.0);
}

void LocalCostmapGenerator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // scan 데이터를 pointcloud2로 변환한다.
    laserProjection_->projectLaser(*scan, *pointCloud2_);

    // 데이터 전처리를 위해 pointcloud2를 pcl로 변환한다.
    pcl::fromROSMsg(*pointCloud2_, *pcl_);

    // 스캔 데이터를 받았다는 플래그를 true로 설정한다.
    isScanReceived_ = true;
}

void LocalCostmapGenerator::timerCallback()
{
    if (!isScanReceived_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
        return;
    }

    // convert pcl's frame(laser) to robot frame(base_link)
    sensorFrameToRobotFrame(pcl_);

    // convert pcl to costmap
    const std::vector<grid_map::Index> occupiedIndices = pclToCostmap(pcl_, costmap_);

    // publish costmap as grid_map_msgs::msg::GridMap for planning
    std::shared_ptr<grid_map_msgs::msg::GridMap> costmapMsg;
    costmapMsg = grid_map::GridMapRosConverter::toMessage(*costmap_);
    costmapPublisher_->publish(*costmapMsg);

    // publish costmap as nav_msgs::msg::OccupancyGrid for visualization
    nav_msgs::msg::OccupancyGrid occupancyGridMsg;
    grid_map::GridMapRosConverter::toOccupancyGrid(*costmap_, "collision_layer", 0.0, 1.0, occupancyGridMsg);
    occupancyGridMsg.header.stamp = this->get_clock()->now();
    occupancyGridMsg.header.frame_id = robotFrameId_;
    occupancyGridPublisher_->publish(occupancyGridMsg);
}

void LocalCostmapGenerator::sensorFrameToRobotFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl)
{   

    // base_link frame와 laser frame 사이의 변환을 tf를 통해 받는다.
    try {
        transformStamped_ = tf_buffer_.lookupTransform(robotFrameId_, sensorFrameId_, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[LocalCostmapGenerator] %s", ex.what());
        return;
    }

    // tf를 Eigen의 Isometry3d로 변환한다. Isometry3d는 Transformation Matrix를 나타내는 Eigen의 클래스이다.
    // 3차원의 변환행렬을 나타내므로 tf를 Eigen으로 변환하면 3차원 변환행렬(이는 4x3 행렬이다.)을 얻을 수 있다.
    // 아니 왜 4x3 행렬이지? - 3x3행렬로 나타낼 수 있는 변환을 생각해보자. 원점을 평행이동하는 게 가능한가?
    // 절대 아닐 것이다. 3x3 행렬에 0벡터를 곱하면 0벡터가 나오기 때문이다.
    // 여기에서 트릭을 사용한다. 벡터에 차원을 하나 더하는 것이다. 그리고 그 더해진 차원을 선형변환하면 평행이동이 가능하다.
    // eg. (1;2)벡터가 있다. 이것을 (1 0 ; 0 1) 행렬로 선형변환하면 (1;2)가 나온다. Identity 행렬이기 때문이다.
    // 이것을 (1;2;1)로 차원을 하나 더해 바꾸고, (1 0 1; 0 1 1)로 선형변환하면 (2;3) 벡터가 나온다.
    // 같은 연산을 (0;0)벡터에 대해 해보자.(0;0;1)에 (1 0 1; 0 1 1)을 곱하면 (1;1)이 나온다.
    // 원래 2차원에서 (0;0)인 벡터가 (1;1)로 변환되었다. 이렇게 차원을 하나 더해놓고 선형변환을 하면 평행이동이 가능해진다.
    // 이것이 3차원에서 평행이동을 나타낼 때 4x3 행렬이 나오는 이유이며, 
    // 여기에서 좌표변환을 할 때 eigen을 사용해 변환 행렬을 얻고 행렬곱을 하여 좌표를 변환하는 것이다.
    const Eigen::Isometry3d transformMatrix = tf2::transformToEigen(transformStamped_.transform);

    // laser frame 기준 pcl(좌표)을 변환행렬을 통해 base_link frame의 pcl(좌표)로 변환한다.
    pcl::transformPointCloud(*pcl, *pcl, transformMatrix.matrix().cast<float>());
}

std::vector<grid_map::Index> LocalCostmapGenerator::pclToCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl, 
    grid_map::GridMap* costmap
) const
{
    // transform pcl to costmap
    std::vector<grid_map::Index> occupiedIndices(pcl->points.size(), grid_map::Index(0, 0));

    // initialize cost map
    costmap->get("collision_layer").setConstant(0.0);
    for (unsigned int index = 0; index < pcl->points.size(); index++) {
        const auto& point = pcl->points[index];

        // const double len = gridLength / 2; // grid map의 길이(m)의 절반
        const double map_x_helf_length = gridLength / 2;
        const double map_y_half_length = gridLength / 2;

        if (point.x >= -map_x_helf_length
            && point.x <= map_x_helf_length
            && point.y >= -map_y_half_length
            && point.y <= map_y_half_length
        ) {
            // Convert point to grid map coordinates
            int grid_x = static_cast<int>((-point.x + map_x_helf_length) / resolution);
            int grid_y = static_cast<int>((-point.y + map_y_half_length) / resolution);

            // 그리드 인덱스가 범위를 벗어나지 않도록 체크
            if (grid_x >= 0 
                && grid_x < gridSize 
                && grid_y >= 0 
                && grid_y < gridSize
            ) {
                // occupiedIndices[index] = grid_map::Index(grid_x, grid_y);
                // update costmap
                costmap->at("collision_layer", grid_map::Index(grid_x, grid_y)) = 1.0;
            } else {
                occupiedIndices[index] = grid_map::Index(-1, -1); // 범위를 벗어난 인덱스
            }
        } else {
            occupiedIndices[index] = grid_map::Index(-1, -1);
        }
    }

    // // 디버깅용 출력: 행렬 시각화.
    // Eigen::MatrixXf& costmapData = costmap->get("collision_layer"); // costmap의 데이터를 가져온다.
    // std::cout << "----------------------------------" << std::endl;
    // std::cout << costmapData << std::endl; // 디버깅용 출력: costmap 시각화
    // std::cout << "----------------------------------" << std::endl;
    
    return occupiedIndices;
}