#include "common/common_types.h"
#include "common/common_utils.h"
#include "common/ConfigParser.h"
#include "common/sanity_check.h"
#include "common/CameraObservation.h"
#include "visualization/Visualizer.h"
#include "imgproc/frameio/FetchInterface.h"
#include "imgproc/frameio/GenericFileFetch.h"
// #include "imgproc/frameio/AzureKinectDKFetch.h"
#include "imgproc/frameio/VolumeDeformFileFetch.h"
#include "imgproc/frameio/RealsenseFectch.h"
#include "imgproc/ImageProcessor.h"

#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>


int holes_fill = 1;
int filter_magnitude = 3;
int smooth_alpha = 50; // 滑动条范围为0-100，实际使用时除以100
int smooth_delta = 15;
int holes_fill_filter = 0;

void on_trackbar(int, void*) {}

void testBoundary() {

	using namespace surfelwarp;
	//Parpare the test data
	std::string config_path;
	config_path = "/home/rvclab/dev/surfelwarp/test_data/realsense_config.json";
	auto& config = ConfigParser::Instance();
	config.ParseConfig(config_path);

	//First test fetching
	FetchInterface::Ptr fetcher = std::make_shared<RealsenseFetch>(config.data_path(), config.isSaveOnlineFrame());
	ImageProcessor::Ptr processor = std::make_shared<ImageProcessor>(fetcher);
	
	while(true) 
	{
		cv::Mat depth_img, rgb_img;
		fetcher->FetchDepthAndRGBImage(0, depth_img, rgb_img);
		depth_img.convertTo(depth_img, CV_8UC1, 255.0 / 4000); // 假设深度范围是0-10000mm
		cv::Mat depth_colormap;
		cv::applyColorMap(depth_img, depth_colormap, cv::COLORMAP_JET);
		cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
		cv::namedWindow("rgb", cv::WINDOW_AUTOSIZE);
		cv::imshow("depth", depth_colormap);
		cv::imshow("rgb", rgb_img);
		if(cv::waitKey(1) == 27) {
			break;
		}
	}

	// 图像处理结果的可视化
	// CameraObservation observation;
	// for(int i = 0; i<config.num_frames(); i++) {
	// 	LOG(INFO) << "The " << i << "th Frame";
	// 	processor->ProcessFrameStreamed(observation, i);
		// cv::Mat depth_img, rgb_img;
		// fetcher->FetchDepthAndRGBImage(0, depth_img, rgb_img);
		// cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
		// cv::namedWindow("rgb", cv::WINDOW_AUTOSIZE);
		// cv::imshow("depth", depth_img);
		// cv::imshow("rgb", rgb_img);
		// if(cv::waitKey(1) == 27) {
		// 	break;
		// }
		
		
		// auto draw_func = [&]() {
		//Visualizer::DrawPointCloud(observation.vertex_confid_map);
		// Visualizer::DrawSegmentMask(observation.foreground_mask, observation.normalized_rgba_map, 1);
		// //Visualizer::DrawGrayScaleImage(observation.filter_foreground_mask);
		// };

		// std::thread draw_thread(draw_func);
		// draw_thread.join();
	// }
}

// 测试滤波器
void test()
{
	rs2::pipeline pipe;
	rs2::config cfg;

	// 启用深度和颜色流
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

	// 启动管道
	rs2::pipeline_profile profile = pipe.start(cfg);

	// 初始化空间滤波器
	rs2::spatial_filter m_spatial_filter;
	rs2::hole_filling_filter m_hole_fill_filter;
	rs2::disparity_transform m_depth_to_disparity(true);
	rs2::disparity_transform m_disparity_to_depth(false);

	cv::namedWindow("Filtered Depth Frame", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Color Frame", cv::WINDOW_AUTOSIZE);
	cv::createTrackbar("Holes Fill", "Filtered Depth Frame", &holes_fill, 5, on_trackbar);
	cv::createTrackbar("Filter Magnitude", "Filtered Depth Frame", &filter_magnitude, 4, on_trackbar);
	cv::createTrackbar("Smooth Alpha", "Filtered Depth Frame", &smooth_alpha, 75, on_trackbar);
	cv::createTrackbar("Smooth Delta", "Filtered Depth Frame", &smooth_delta, 49, on_trackbar);
	cv::createTrackbar("Holes Fill Filter", "Filtered Depth Frame", &holes_fill_filter, 2, on_trackbar);

	while (true)
	{
		// 等待下一帧数据
		rs2::frameset frameset = pipe.wait_for_frames();
		rs2::align align2color(RS2_STREAM_COLOR);
		frameset = align2color.process(frameset);
		

		// 更新空间滤波器参数
		m_spatial_filter.set_option(RS2_OPTION_HOLES_FILL, holes_fill);
		m_spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, filter_magnitude+1);
		m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.25+smooth_alpha / 100.0f); 
		m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, smooth_delta+1);
		m_hole_fill_filter.set_option(RS2_OPTION_HOLES_FILL, holes_fill_filter);

		rs2::frame depth_frame = frameset.get_depth_frame()
								.apply_filter(m_depth_to_disparity)
								.apply_filter(m_spatial_filter)
								.apply_filter(m_hole_fill_filter)
								.apply_filter(m_disparity_to_depth);
		rs2::frame color_frame = frameset.get_color_frame();


		// 将RealSense帧转换为OpenCV矩阵
		cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

		// 将深度图像归一化以适合显示
		cv::Mat depth_image_normalized;
		cv::normalize(depth_image, depth_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

		// 显示深度和颜色图像
		cv::imshow("Filtered Depth Frame", depth_image_normalized);
		cv::imshow("Color Frame", color_image);

		// 检查用户输入以退出循环
		if (cv::waitKey(1) == 27) { // 27 是 ESC 键的 ASCII 码
			break;
		}
    }

}

int main() 
{
	testBoundary();
	// test();
	
}