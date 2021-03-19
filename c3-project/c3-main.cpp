#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <boost/optional/optional_io.hpp>

#include <iostream>
#include <chrono>

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;
using namespace Eigen;
Eigen::Matrix4f get_transformation(double x, double y, double z, double yaw, double pitch, double roll){

	Quaterniond q;
	q = AngleAxisd(roll, Vector3d::UnitX())
		* AngleAxisd(pitch, Vector3d::UnitY())
		* AngleAxisd(yaw, Vector3d::UnitZ());
	auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
	Matrix3d r = q.toRotationMatrix();
	Matrix4d res = Matrix4d::Identity();

	res.block<3,3>(0,0) = r;
	res(0,3) = x;
	res(1,3) = y;
	res(2,3) = z;

	Eigen::Matrix4f res_f = res.cast <float> ();
	return res_f;
}

Eigen::Matrix4f convert2Eigen(const Pose &pose){
	double x;
	double y;
	double z;
	double yaw,  pitch,  roll;

	x = pose.position.x;
	y = pose.position.y;
	z = pose.position.z;

	yaw = pose.rotation.yaw;
	pitch = pose.rotation.pitch;
	roll = pose.rotation.roll;

	return get_transformation( x,  y,  z,  yaw,  pitch,  roll);
}

std::string convert2String(const Pose &pose){
	double x;
	double y;
	double z;
	double yaw,  pitch,  roll;
	std::stringstream ss;
	x = pose.position.x;
	y = pose.position.y;
	z = pose.position.z;

	yaw = pose.rotation.yaw;
	pitch = pose.rotation.pitch;
	roll = pose.rotation.roll;

	ss<<"["<<x <<","<<y<<","<<z<<",,"<<yaw<<","<<pitch<<","<<roll<<"]";
	return ss.str();
}

bool refresh_view = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	int up = 1;
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	cout << "UP Pressed: " << up++ << endl;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

int main(){

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	double cur_sim_time = -1;
	double last_sim_time = -1;
	double dt = -1;
	lidar->Listen([&cur_sim_time, &last_sim_time, &new_scan, &lastScanTime, &scanCloud](auto data){
		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);

			for (auto detection : *scan){
				if((detection.point.x*detection.point.x + detection.point.y*detection.point.y + detection.point.z*detection.point.z) > 8.0){ // Don't include points touching ego
					pclCloud.points.push_back(PointT(detection.point.x, detection.point.y, detection.point.z));
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
				if(cur_sim_time == -1){
					cur_sim_time = scan->GetTimestamp ();
				}else{
					last_sim_time = cur_sim_time;
					cur_sim_time = scan->GetTimestamp ();
				}
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	Pose pose_lidarRef(Point(lidar->GetTransform().location.x, lidar->GetTransform().location.y, lidar->GetTransform().location.z), Rotate(lidar->GetTransform().rotation.yaw * pi/180, lidar->GetTransform().rotation.pitch * pi/180, lidar->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	auto settings = world.GetSettings();
	std::cout<<"settings.fixed_delta_seconds= "<< settings.fixed_delta_seconds <<std::endl;
	std::cout<<"settings.synchronous_mode= "<< settings.synchronous_mode <<std::endl;
	std::cout<<"settings.no_rendering_mode= "<< settings.no_rendering_mode <<std::endl;

	Pose last_pose;
	last_pose.position.x = -100;
	double vx = 0;
	double vy = 0;
	double last_icp_score = -1;
	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::cout << "world tick"<<std::endl;
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		if(!new_scan){
			if(last_sim_time !=-1){
				dt = cur_sim_time -last_sim_time;
			}

			// TODO: (Filter scan using voxel filter)
			Timer tmr;
			pcl::VoxelGrid<PointT> sor;
			sor.setInputCloud (scanCloud);
			sor.setLeafSize (0.2f, 0.2f, 0.2f);
			sor.filter (*cloudFiltered);

			// TODO: Find pose transform by using ICP or NDT matching
			//pose = ....
			tmr.reset();
			pcl::IterativeClosestPoint<PointT, PointT> icp;
			icp.setInputSource(cloudFiltered);
			icp.setInputTarget(mapCloud);
			icp.setMaximumIterations (15);
			if(last_icp_score > 0.04){
				icp.setMaximumIterations (25);
			}

			pcl::PointCloud<pcl::PointXYZ> Final;
			Eigen::Matrix4f guess;
			auto pred_pose = pose;
			if(last_sim_time !=-1){
				pred_pose.position.x = pose.position.x + dt * vx;
				pred_pose.position.y = pose.position.y + dt * vy;
			}
			guess = convert2Eigen(pred_pose);

			icp.align(Final, guess);
			last_icp_score = icp.getFitnessScore();
			Matrix4f transformation_matrix = icp.getFinalTransformation();
			pose = getPose(transformation_matrix.cast <double>());


			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			pcl::transformPointCloud (*scanCloud, *scanCloud, convert2Eigen(pose));

			viewer->removePointCloud("scan");
			// TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, scanCloud, "scan", Color(1,0,0) );


			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);

			pclCloud.points.clear();
		}

		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

		double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );


		if(poseError > maxError)
			maxError = poseError;
		double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
		viewer->removeShape("maxE");
		viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
		viewer->removeShape("derror");
		viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
		viewer->removeShape("dist");
		viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

		if(last_pose.position.x == -100){
			last_pose = pose;
		}else{
			double dist_x = pose.position.x - last_pose.position.x;
			double dist_y = pose.position.y - last_pose.position.y;
			double dist_gap = sqrt( dist_x * dist_x + dist_y * dist_y);
			double v = 0;
			if(dt !=-1){
				vx = dist_x/dt;
				vy = dist_y/dt;
				v = dist_gap /dt;
			}
			RowVectorXd vec_gt(7);
			vec_gt <<dt, dist_gap, dist_x,dist_y,v,vx,vy;

			last_pose = pose;
		}
		if(maxError > 1.2 || distDriven >= 170.0 ){
			viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
				std::cout << "Try Again"<<std::endl;
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
				std::cout << "Passed!"<<std::endl;
			}
		}

  		viewer->spinOnce ();
  		new_scan = true;

		

  	}
	return 0;
}