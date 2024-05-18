//
// Created by wei on 5/22/18.
//

#include "common/common_utils.h"
#include "common/ConfigParser.h"
#include "core/SurfelWarpSerial.h"
#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
	using namespace surfelwarp;
	
	//Get the config path
	std::string config_path;
	if (argc <= 1) {
		config_path = "/home/rvclab/dev/surfelwarp/test_data/breathe_config.json";
	} else {
		config_path = std::string(argv[1]);
	}

	//Parse it
	auto& config = ConfigParser::Instance();
	config.ParseConfig(config_path);

	//The context
	//auto context = initCudaContext();

	//The processing loop
	SurfelWarpSerial fusion;

	fusion.ProcessFirstFrame();
	for(auto i = config.start_frame_idx() + 1; i < config.num_frames(); i++){
		LOG(INFO) << "The " << i << "th Frame";
		fusion.ProcessNextFrameWithReinit(config);
	}
	
	//destroyCudaContext(context);
}
