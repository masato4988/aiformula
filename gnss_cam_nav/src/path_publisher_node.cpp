#include "gnssnav/path_publisher_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace gnssnav{

Publisher::Publisher(const rclcpp::NodeOptions& options) : Publisher("", options) {}

Publisher::Publisher(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("path_publisher_node", name_space, options),
init_flag_(true),
freq(get_parameter("interval_ms").as_int()),
first_path_file_name(get_parameter("first_path_file_name").as_string()),
second_path_file_name(get_parameter("second_path_file_name").as_string())
{
    first_publisher_ = this->create_publisher<nav_msgs::msg::Path>("first_gnss_path", 10);
    second_publisher_ = this->create_publisher<nav_msgs::msg::Path>("second_gnss_path", 10);
    visual_publisher_ = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);

    std::string file_path = ament_index_cpp::get_package_share_directory("gnssnav")+"/config/"+"course_data/"+first_path_file_name+".csv";
    loadCSV(file_path, first_xs_, first_ys_);
    file_path = ament_index_cpp::get_package_share_directory("gnssnav")+"/config/"+"course_data/"+second_path_file_name+".csv";
    loadCSV(file_path, second_xs_, second_ys_);

    first_path_msg_ = setMsg(first_xs_, first_ys_);
    second_path_msg_ = setMsg(second_xs_, second_ys_);
    visual_path_msg_ = setMsg(xs_, ys_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq),
        std::bind(&Publisher::loop, this));
}

// load CSV file
void Publisher::loadCSV(const std::string file_path, std::vector<double>& xs, std::vector<double>& ys){
    std::ifstream file(file_path);
    printf("loadCSV: file_path_ = %s\n", file_path.c_str());

    if (!file.is_open()) {
        std::cerr << "loadCSV is failed to open file" << file_path << std::endl;
        return;
    }else {
        std::cerr << "loadCSV is sucsess to open file" << std::endl;
    }

    while(std::getline(file, line_)) {
        std::stringstream ss(line_);
        tokens_.clear();
        while(std::getline(ss, cell_, ',')) {
            tokens_.push_back(cell_);
        }
        double lat = std::stod(tokens_[0]);
        double lon = std::stod(tokens_[1]);
        auto [x, y] = convertGPStoUTM(lat, lon);

        if(init_flag_) setInitPose(x, y);

        xs_.push_back(x - base_x_);
        ys_.push_back(y - base_y_);

        xs.push_back(x);
        ys.push_back(y);
    }
}

// init pose
void Publisher::setInitPose(double x, double y){
    base_x_ = x;
    base_y_ = y;
    init_flag_ = false;
}

// path create
nav_msgs::msg::Path Publisher::setMsg(const std::vector<double>& xs, const std::vector<double>& ys){
    std::vector<Eigen::Vector2d> spline_points = interpolateSpline(xs, ys, 100);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    for (const auto& coord : spline_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = coord.x();
        pose.pose.position.y = coord.y();
        path_msg.poses.push_back(pose);
    }

    return path_msg;
}

// spline
std::vector<Eigen::Vector2d> Publisher::interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points){
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
    for (size_t i=0; i < xs.size(); ++i){
        points(i, 0) = xs[i];
        points(i, 1) = ys[i];
    }

    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points.transpose(), 2); //2次のキュービックスプライン

    std::vector<Eigen::Vector2d> result_;
    if(num_points > 1)
        step = 1.0 / (num_points -1);
    for (int i = 0; i < num_points; ++i) {
        double u = i * step;
        Eigen::Vector2d pt = spline(u);
        result_.push_back(pt);
    }
    return result_;
}

// WGS84系からUTM座標系へ変換
std::pair<double, double> Publisher::convertGPStoUTM(double lat, double lon) {
    if (!(-90 <= lat) || !(lat <= 90) || !(-180 <= lon) || !(lon <= 180)) {
        std::cerr << "Error: Latitude or longitude values are out of valid range." << std::endl;
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }
    PJ *P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32654", nullptr);
    PJ_COORD p = proj_coord(lat, lon, 0, 0);
    p = proj_trans(P, PJ_FWD, p);
    proj_destroy(P);
    return {p.xy.x, p.xy.y};
}

void Publisher::loop(void){
    first_publisher_->publish(first_path_msg_);
    second_publisher_->publish(second_path_msg_);
    visual_publisher_->publish(visual_path_msg_);
}

}  // namespace gnssnav
