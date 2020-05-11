#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "common/OpenCV_CrossPlatform.h"
#include "common/ConfigParser.h"
#include "VolumeDeformFileFetch.h"


surfelwarp::VolumeDeformFileFetch::VolumeDeformFileFetch(const path& data_path) : m_data_path(data_path) {
	const auto & config = ConfigParser::Instance();
	// before read any image, m_cur_frame_idx = start_idx - 1
	m_cur_frame_idx = config.start_frame_idx() - 1;
    m_thread = std::thread(&surfelwarp::VolumeDeformFileFetch::ReadImageFromFile, this);
}



void surfelwarp::VolumeDeformFileFetch::FetchDepthImage(size_t frame_idx, cv::Mat & depth_img)
{
	//Read the image
	if (frame_idx == m_cur_frame_idx){
		std::unique_lock<std::mutex> lock(m_mutex);
		depth_img = m_depth_images.front();
		m_depth_images.pop_front();
	}else{
		std::cout << "this should not happen, just for debug" << std::endl;
		std::cout << "fetch frame_idx: " << frame_idx << ";  cur_frame_idx: " << m_cur_frame_idx << std::endl;
	}
}

void surfelwarp::VolumeDeformFileFetch::FetchDepthImage(size_t frame_idx, void * depth_img)
{

}

void surfelwarp::VolumeDeformFileFetch::FetchRGBImage(size_t frame_idx, cv::Mat & rgb_img)
{
	if(frame_idx == m_cur_frame_idx){
		std::unique_lock<std::mutex> lock(m_mutex);
		rgb_img = m_color_images.front();
		m_color_images.pop_front();
	}else{
		std::cout << "this should not happen, just for debug" << std::endl;
		std::cout << "fetch frame_idx: " << frame_idx << ";  cur_frame_idx: " << m_cur_frame_idx << std::endl;
	}
}

void surfelwarp::VolumeDeformFileFetch::FetchDepthAndRGBImage(size_t frame_idx, cv::Mat & depth_img, cv::Mat & rgb_img){
	if(frame_idx == m_cur_frame_idx){
		std::unique_lock<std::mutex> lock(m_mutex);
		rgb_img = m_color_images.front();
		m_color_images.pop_front();

		depth_img = m_depth_images.front();
		m_depth_images.pop_front();
	}else{
		std::cout << "this should not happen, just for debug" << std::endl;
		std::cout << "fetch frame_idx: " << frame_idx << ";  cur_frame_idx: " << m_cur_frame_idx << std::endl;
	}
}

void surfelwarp::VolumeDeformFileFetch::FetchRGBImage(size_t frame_idx, void * rgb_img)
{

}

boost::filesystem::path surfelwarp::VolumeDeformFileFetch::FileNameVolumeDeform(size_t frame_idx, bool is_depth_img) const
{
	//Construct the file_name
	char frame_idx_str[20];
	sprintf(frame_idx_str, "%06d", static_cast<int>(frame_idx));
	std::string file_name = "frame-";
	file_name += std::string(frame_idx_str);
	if (is_depth_img) {
		file_name += ".depth";
	}
	else {
		file_name += ".color";
	}
	file_name += ".png";

	//Construct the path
	path file_path = m_data_path / path(file_name);
	return file_path;
}



boost::filesystem::path surfelwarp::VolumeDeformFileFetch::FileNameSurfelWarp(size_t frame_idx, bool is_depth_img) const {
	return FileNameVolumeDeform(frame_idx, is_depth_img);
}

//std::mutex surfelwarp::VolumeDeformFileFetch::m_mutex;

void surfelwarp::VolumeDeformFileFetch::ReadImageFromFile(){
	while(1) {
		bool is_empty;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            bool d_empty = m_depth_images.empty();
            bool c_empty = m_color_images.empty();
            if (d_empty != c_empty){
                std::cout << "Error: the number of depth_image and color_image is not queal!" << std::endl;
                break;
            }
		    is_empty = d_empty && c_empty;
        }
        if (is_empty){
            std::unique_lock<std::mutex> lock(m_mutex);

		    m_cur_frame_idx ++; 

            path file_path;
            file_path = FileNameSurfelWarp(m_cur_frame_idx, true);
            if (! boost::filesystem::exists(file_path)){
                break;
            }
        	cv::Mat depth_img = cv::imread(file_path.string(), CV_ANYCOLOR | CV_ANYDEPTH);
		    m_depth_images.push_back(depth_img);

    	    file_path = FileNameSurfelWarp(m_cur_frame_idx, false);
        	if (! boost::filesystem::exists(file_path)){
                break;
            }
            cv::Mat rgb_img = cv::imread(file_path.string(), CV_ANYCOLOR | CV_ANYDEPTH);
    		m_color_images.push_back(rgb_img);
        } else {
            usleep(3000);
        }
    }
}
