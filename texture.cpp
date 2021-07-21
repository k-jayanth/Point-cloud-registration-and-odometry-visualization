#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"


int main(int argc, char const *argv[])
{
	using namespace open3d;
	camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
	utility::LogInfo("Open3D {}\n", OPEN3D_VERSION);
	geometry::Image color, depth, color1, depth1;

	// Reading Data
	// Texture source
	io::ReadImage("../images/Texture/source/1341840839.193463.png", color);
    io::ReadImage("../images/Texture/source/1341840839.193568.png", depth);
    
    // Texture target
	io::ReadImage("../images/Texture/Target/1341840839.225363.png", color1);
    io::ReadImage("../images/Texture/Target/1341840839.225381.png", depth1);

    // Creating the RGBD image
    double depth_scale = 1000.0, depth_trunc = 3.0;
    bool convert_rgb_to_intensity = true;

    std::shared_ptr<geometry::RGBDImage> source =
            geometry::RGBDImage::CreateFromTUMFormat(
                    color, depth);
    std::shared_ptr<geometry::RGBDImage> target =
            geometry::RGBDImage::CreateFromTUMFormat(
                    color, depth);


    // Visualization
    auto pcd = geometry::PointCloud::CreateFromRGBDImage(*source, intrinsic);
    visualization::DrawGeometries({pcd});

    Eigen::Matrix4d odo_init = Eigen::Matrix4d::Identity();
    std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> rgbd_odo =
            pipelines::odometry::ComputeRGBDOdometry(
                    *source, *target, intrinsic, odo_init,
                    pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(),
                    pipelines::odometry::OdometryOption());
    std::cout << "RGBD Odometry" << std::endl;
    std::cout << std::get<1>(rgbd_odo) << std::endl;

    io::WritePointCloud("../output.ply", *pcd);

	return 0;
}