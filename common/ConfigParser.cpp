#include "common/logging.h"
#include "common/ConfigParser.h"
#include "common/global_configs.h"
#include "common/Constants.h"
#include "math/eigen_device_tranfer.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include "ConfigParser.h"

surfelwarp::ConfigParser::ConfigParser()
{
    setDefaultParameters();
}

void surfelwarp::ConfigParser::setDefaultParameters()
{
    setDefaultPathConfig();
    setDefaultFrameIndex();
    setDefaultPeroidsValue();
    setDefaultImageSize();
    setDefaultClipValue();
    setDefaultCameraIntrinsic();
    setDefaultPenaltyConfigs();
    setDefaultIOMode();
    setDefaultOfflineRendering();
}

/* Public interface
 */
void surfelwarp::ConfigParser::ParseConfig(const std::string& config_path)
{
    using json = nlohmann::json;

    // Construct and load the json file
    json config_json;
    std::ifstream input_file(config_path);
    input_file >> config_json;
    input_file.close();

    // The path from json, these are requested
    loadPathConfigFromJson((const void*)&config_json);
    loadFrameIndexFromJson((const void*)&config_json);
    loadDownSample((const void*)&config_json);
    loadIOModeFromJson((const void*)&config_json);
    loadPeroidsValueFromJson((const void*)&config_json);
    loadImageSizeFromJson((const void*)&config_json);
    loadClipValueFromJson((const void*)&config_json);
    loadCameraIntrinsicFromJson((const void*)&config_json);
    loadPenaltyConfigFromJson((const void*)&config_json);
    loadSaveOnlineFrame((const void*)&config_json);
    loadOfflineRendering((const void*)&config_json);
}

void surfelwarp::ConfigParser::SaveConfig(const std::string& config_path) const
{
    using json = nlohmann::json;

    // Construct the json file
    json config_json;

    savePathConfigToJson((void*)&config_json);
    saveFrameIndexToJson((void*)&config_json);
    savePeroidsValueToJson((void*)&config_json);
    saveImageSizeToJson((void*)&config_json);
    saveClipValueToJson((void*)&config_json);
    saveCameraIntrinsicToJson((void*)&config_json);
    savePenaltyConfigToJson((void*)&config_json);
    saveIOModeToJson((void*)&config_json);
    saveOfflineRendering((void*)&config_json);

    // Save it to file
    std::ofstream file_output(config_path);
    file_output << config_json.dump();
    file_output.close();
}

// Access interface
surfelwarp::ConfigParser& surfelwarp::ConfigParser::Instance()
{
    static ConfigParser parser;
    return parser;
}

/* The query about the path
 */
void surfelwarp::ConfigParser::setDefaultPathConfig()
{
    // File Options
#if defined(_WIN32)
    m_data_prefix
        = "C:/Users/wei/Documents/Visual Studio "
          "2015/Projects/Fusion/data/boxing";
    m_gpc_model_path
        = "C:/Users/wei/Documents/Visual Studio "
          "2015/Projects/surfelwarp/data/gpc_models/sintel_small_speed";
#else
    m_data_prefix
        = "/home/roboticlab/Desktop/Maihn/teleop_virtual_fixture/thirdparty/"
          "surfelwarp/test_data";
    m_gpc_model_path
        = "/home/roboticlab/Desktop/Maihn/teleop_virtual_fixture/thirdparty/"
          "surfelwarp/test_data/sintel_small_speed";
#endif
}

void surfelwarp::ConfigParser::savePathConfigToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    // Save it
    config_json["data_prefix"] = m_data_prefix;
    config_json["gpc_model_path"] = m_gpc_model_path;
}

void surfelwarp::ConfigParser::loadPathConfigFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // Check it
    SURFELWARP_CHECK(config_json.find("data_prefix") != config_json.end());
    SURFELWARP_CHECK(config_json.find("gpc_model_path") != config_json.end());
    SURFELWARP_CHECK(config_json.find("save_path") != config_json.end());

    // Load it
    m_data_prefix = config_json["data_prefix"];
    m_gpc_model_path = config_json["gpc_model_path"];
    m_save_path = config_json["save_path"];
}

const boost::filesystem::path surfelwarp::ConfigParser::data_path() const
{
    boost::filesystem::path data_dir(m_data_prefix);
    // data_dir = data_dir / m_scene_name;
    return data_dir;
}

const boost::filesystem::path surfelwarp::ConfigParser::gpc_model_path() const
{
    return boost::filesystem::path(m_gpc_model_path);
}

const boost::filesystem::path surfelwarp::ConfigParser::save_path() const
{
    return boost::filesystem::path(m_save_path);
}

// The query about frame
void surfelwarp::ConfigParser::setDefaultFrameIndex()
{
    m_start_frame_idx = 0;
    m_num_frames = 100;
}

void surfelwarp::ConfigParser::saveFrameIndexToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    // Save it
    config_json["start_frame"] = m_start_frame_idx;
    config_json["num_frame"] = m_num_frames;
}

void surfelwarp::ConfigParser::loadFrameIndexFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // Load it, these parameter are not required
    // but implemented as required for debug
    SURFELWARP_CHECK(config_json.find("start_frame") != config_json.end());
    SURFELWARP_CHECK(config_json.find("num_frame") != config_json.end());

    // load it
    m_start_frame_idx = config_json["start_frame"];
    m_num_frames = config_json["num_frame"];
}

int surfelwarp::ConfigParser::start_frame_idx() const
{
    return m_start_frame_idx;
}

int surfelwarp::ConfigParser::num_frames() const
{
    return m_num_frames;
}

// The method about peroids
void surfelwarp::ConfigParser::setDefaultPeroidsValue()
{
    m_use_periodic_reinit = false;
    m_reinit_period = -1; // By default, no periodic
}

void surfelwarp::ConfigParser::savePeroidsValueToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    config_json["use_periodic_reinit"] = m_use_periodic_reinit;
    config_json["reinit_period"] = m_reinit_period;
}

void surfelwarp::ConfigParser::loadPeroidsValueFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // Not in the config
    if (config_json.find("use_periodic_reinit") == config_json.end()) {
        m_use_periodic_reinit = false;
        m_reinit_period = -1;
        return;
    }

    // In the config file
    SURFELWARP_CHECK(config_json.find("reinit_period") != config_json.end());
    m_use_periodic_reinit = config_json["use_periodic_reinit"];
    m_reinit_period = config_json["reinit_period"];
}

/* The image size methods
 */
void surfelwarp::ConfigParser::setDefaultImageSize()
{
    m_raw_image_rows = 480;
    m_raw_image_cols = 640;
    m_clip_image_rows = m_raw_image_rows - 2 * boundary_clip;
    m_clip_image_cols = m_raw_image_cols - 2 * boundary_clip;
}

void surfelwarp::ConfigParser::saveImageSizeToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    config_json["image_rows"] = m_raw_image_rows;
    config_json["image_cols"] = m_raw_image_cols;
}

void surfelwarp::ConfigParser::loadImageSizeFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // Load it, these parameter are not required
    // but implemented as required for debug
    SURFELWARP_CHECK(config_json.find("image_rows") != config_json.end());
    SURFELWARP_CHECK(config_json.find("image_cols") != config_json.end());

    m_raw_image_rows = config_json["image_rows"];
    m_raw_image_cols = config_json["image_cols"];

    if (m_downsample_scale != 1) {
        m_raw_image_rows = m_raw_image_rows / m_downsample_scale;
        m_raw_image_cols = m_raw_image_cols / m_downsample_scale;
    }

    m_clip_image_rows = m_raw_image_rows - 2 * boundary_clip;
    m_clip_image_cols = m_raw_image_cols - 2 * boundary_clip;
}

/* The method for clip size
 */
void surfelwarp::ConfigParser::setDefaultClipValue()
{
    m_clip_near = 10;
    m_clip_far = 1000;
}

void surfelwarp::ConfigParser::saveClipValueToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    config_json["clip_near"] = m_clip_near;
    config_json["clip_far"] = m_clip_far;
}

void surfelwarp::ConfigParser::loadClipValueFromJson(const void* json_ptr)
{
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // Load it, these parameter are required
    SURFELWARP_CHECK(config_json.find("clip_near") != config_json.end());
    SURFELWARP_CHECK(config_json.find("clip_far") != config_json.end());

    m_clip_near = config_json["clip_near"];
    m_clip_far = config_json["clip_far"];
}

/* The method and buffer for querying the intrinsic
 */

void surfelwarp::ConfigParser::setDefaultCameraIntrinsic()
{
    const float focal_x = 570.f;
    const float focal_y = 570.f;
    const float principal_x_raw = 320.0f;
    const float principal_y_raw = 240.0f;

    // This transform is little bit subtle
    const float principal_x_clip = principal_x_raw - boundary_clip;
    const float principal_y_clip = principal_y_raw - boundary_clip;

    // Init the intrinsic parameter
    raw_depth_intrinsic
        = Intrinsic(focal_x, focal_y, principal_x_raw, principal_y_raw);
    raw_rgb_intrinsic
        = Intrinsic(focal_x, focal_y, principal_x_raw, principal_y_raw);
    clip_rgb_intrinsic
        = Intrinsic(focal_x, focal_y, principal_x_clip, principal_y_clip);
}

void surfelwarp::ConfigParser::saveCameraIntrinsicToJson(void* json_ptr) const
{
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    // The depth camera instrinsic parameter
    config_json["depth_focal_x"] = raw_depth_intrinsic.focal_x;
    config_json["depth_focal_y"] = raw_depth_intrinsic.focal_y;
    config_json["depth_principal_x"] = raw_depth_intrinsic.principal_x;
    config_json["depth_principal_y"] = raw_depth_intrinsic.principal_y;

    // The rgb camera intrinsic parameter
    config_json["rgb_focal_x"] = raw_rgb_intrinsic.focal_x;
    config_json["rgb_focal_y"] = raw_rgb_intrinsic.focal_y;
    config_json["rgb_principal_x"] = raw_rgb_intrinsic.principal_x;
    config_json["rgb_principal_y"] = raw_rgb_intrinsic.principal_y;
}

void surfelwarp::ConfigParser::loadCameraIntrinsicFromJson(const void* json_ptr)
{
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // The function for loading
    auto check_and_load = [&](const std::string& key, float& value) -> void {
        SURFELWARP_CHECK(config_json.find(key) != config_json.end());
        value = config_json[key];
    };

    check_and_load("depth_focal_x", raw_depth_intrinsic.focal_x);
    check_and_load("depth_focal_y", raw_depth_intrinsic.focal_y);
    check_and_load("depth_principal_x", raw_depth_intrinsic.principal_x);
    check_and_load("depth_principal_y", raw_depth_intrinsic.principal_y);

    check_and_load("rgb_focal_x", raw_rgb_intrinsic.focal_x);
    check_and_load("rgb_focal_y", raw_rgb_intrinsic.focal_y);
    check_and_load("rgb_principal_x", raw_rgb_intrinsic.principal_x);
    check_and_load("rgb_principal_y", raw_rgb_intrinsic.principal_y);

    if (m_use_downsample) {
        std::cout << "Use downsample, downsample scale:2.0, modifying the "
                     "intrinsic parameters"
                  << std::endl;
        raw_depth_intrinsic.focal_x /= m_downsample_scale;
        raw_depth_intrinsic.focal_y /= m_downsample_scale;
        raw_depth_intrinsic.principal_x /= m_downsample_scale;
        raw_depth_intrinsic.principal_y /= m_downsample_scale;

        raw_rgb_intrinsic.focal_x /= m_downsample_scale;
        raw_rgb_intrinsic.focal_y /= m_downsample_scale;
        raw_rgb_intrinsic.principal_x /= m_downsample_scale;
        raw_rgb_intrinsic.principal_y /= m_downsample_scale;
    }

    // The clip intrinsic
    const float principal_x_clip
        = raw_rgb_intrinsic.principal_x - boundary_clip;
    const float principal_y_clip
        = raw_rgb_intrinsic.principal_y - boundary_clip;
    clip_rgb_intrinsic = Intrinsic(raw_rgb_intrinsic.focal_x,
        raw_rgb_intrinsic.focal_y, principal_x_clip, principal_y_clip);
}

surfelwarp::Intrinsic surfelwarp::ConfigParser::depth_intrinsic_raw() const
{
    return raw_depth_intrinsic;
}

surfelwarp::Intrinsic surfelwarp::ConfigParser::rgb_intrinsic_raw() const
{
    return raw_rgb_intrinsic;
}

surfelwarp::Intrinsic surfelwarp::ConfigParser::rgb_intrinsic_clip() const
{
    return clip_rgb_intrinsic;
}

surfelwarp::mat34 surfelwarp::ConfigParser::depth2rgb_dev() const
{
    mat34 se3;
    se3.set_identity();
    return se3;
}

// The method about penalty terms
void surfelwarp::ConfigParser::setDefaultPenaltyConfigs()
{
    m_use_foreground_term = false;
    m_use_density_term = false;
    m_use_offline_foreground = true;
}

void surfelwarp::ConfigParser::savePenaltyConfigToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    config_json["use_density"] = m_use_density_term;
    config_json["use_foreground"] = m_use_foreground_term;
    config_json["use_offline_foreground"] = m_use_offline_foreground;
}

void surfelwarp::ConfigParser::loadPenaltyConfigFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    const auto check_and_load = [&](bool& assign_value, const std::string& key,
                                    bool default_value) -> void {
        // The value is not in the config
        if (config_json.find(key) == config_json.end()) {
            assign_value = default_value;
            return;
        }

        // In the config
        assign_value = config_json[key];
    };

    // Do processing
    check_and_load(m_use_density_term, "use_density", false);
    check_and_load(m_use_foreground_term, "use_foreground", true);
    check_and_load(m_use_offline_foreground, "use_offline_foreground", false);
    check_and_load(m_use_segmentation, "use_segmentation", true);
}

bool surfelwarp::ConfigParser::use_foreground_term() const
{
    return m_use_foreground_term;
}

bool surfelwarp::ConfigParser::use_offline_foreground_segmneter() const
{
    return m_use_offline_foreground;
}

bool surfelwarp::ConfigParser::use_density_term() const
{
    return m_use_density_term;
}

bool surfelwarp::ConfigParser::use_segmentation() const
{
    return m_use_segmentation;
}
void surfelwarp::ConfigParser::setDefaultIOMode()
{
    m_io_mode = "GenericFileFetch";
}

std::string surfelwarp::ConfigParser::getIOMode() const
{
    return m_io_mode;
}

void surfelwarp::ConfigParser::loadIOModeFromJson(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    // SURFELWARP_CHECK(config_json.find("io_mode") != config_json.end());
    if (config_json.find("io_mode") != config_json.end()) {
        m_io_mode = config_json["io_mode"];
    }
}

void surfelwarp::ConfigParser::saveIOModeToJson(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);

    config_json["io_mode"] = m_io_mode;
}

void surfelwarp::ConfigParser::loadSaveOnlineFrame(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    if (config_json.find("save_online_frame") != config_json.end()) {
        m_save_online_frame = config_json["save_online_frame"];
    }
}

void surfelwarp::ConfigParser::loadDownSample(const void* json_ptr)
{
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    if (config_json.find("use_downsample") != config_json.end()) {
        m_use_downsample = config_json["use_downsample"];
    }
}

bool surfelwarp::ConfigParser::isSaveOnlineFrame() const
{
    return m_save_online_frame;
}

void surfelwarp::ConfigParser::setDefaultOfflineRendering()
{
    m_offline_rendering = false;
    // save camera observation
    save_segment_mask = false;
    save_filter_depth_image = false;
    save_raw_depth_image = false;
    // save solver maps
    save_rendered_albedo_map = false;
    save_validity_index_map = false;
    save_alignment_error_map = false;
    // save visualization maps
    save_live_normal_map = false;
    save_live_albedo_map = false;
    save_live_phong_map = false;
    save_reference_normal_map = false;
    save_reference_albedo_map = false;
    save_reference_phong_map = false;
    save_se3 = false;
}
void surfelwarp::ConfigParser::loadOfflineRendering(const void* json_ptr)
{
    // Recovery the json type
    using json = nlohmann::json;
    const auto& config_json = *((const json*)json_ptr);

    if (config_json.find("offline_rendering") != config_json.end()) {
        m_offline_rendering = config_json["offline_rendering"];
    }

    if (config_json.find("show_online") != config_json.end()) {
        m_show_online = config_json["show_online"];
    }

    if (config_json.find("save_all") != config_json.end()) {
        if (config_json["save_all"] == true) {
            save_segment_mask = save_filter_depth_image = save_raw_depth_image
                = save_rendered_albedo_map = save_validity_index_map
                = save_alignment_error_map = save_live_normal_map
                = save_live_albedo_map = save_live_phong_map
                = save_reference_albedo_map = save_reference_normal_map
                = save_reference_phong_map = true;
        }
    }
    if (config_json.find("save_segment_mask") != config_json.end()) {
        save_segment_mask = config_json["save_segment_mask"];
    }
    if (config_json.find("save_filter_depth_image") != config_json.end()) {
        save_filter_depth_image = config_json["save_filter_depth_image"];
    }
    if (config_json.find("save_raw_depth_image") != config_json.end()) {
        save_raw_depth_image = config_json["save_raw_depth_image"];
    }
    if (config_json.find("save_rendered_albedo_map") != config_json.end()) {
        save_rendered_albedo_map = config_json["save_rendered_albedo_map"];
    }
    if (config_json.find("save_validity_index_map") != config_json.end()) {
        save_validity_index_map = config_json["save_validity_index_map"];
    }
    if (config_json.find("save_alignment_error_map") != config_json.end()) {
        save_alignment_error_map = config_json["save_alignment_error_map"];
    }
    if (config_json.find("save_live_normal_map") != config_json.end()) {
        save_live_normal_map = config_json["save_live_normal_map"];
    }
    if (config_json.find("save_live_albedo_map") != config_json.end()) {
        save_live_albedo_map = config_json["save_live_albedo_map"];
    }
    if (config_json.find("save_live_phong_map") != config_json.end()) {
        save_live_phong_map = config_json["save_live_phong_map"];
    }
    if (config_json.find("save_reference_normal_map") != config_json.end()) {
        save_reference_normal_map = config_json["save_reference_normal_map"];
    }
    if (config_json.find("save_reference_albedo_map") != config_json.end()) {
        save_reference_albedo_map = config_json["save_reference_albedo_map"];
    }
    if (config_json.find("save_reference_phong_map") != config_json.end()) {
        save_reference_phong_map = config_json["save_reference_phong_map"];
    }
    if (config_json.find("save_se3") != config_json.end()) {
        save_se3 = config_json["save_se3"];
    }
}

void surfelwarp::ConfigParser::saveOfflineRendering(void* json_ptr) const
{
    // Recovery the json type
    using json = nlohmann::json;
    auto& config_json = *((json*)json_ptr);
    config_json["offline_rendering"] = m_offline_rendering;
}

bool surfelwarp::ConfigParser::isOfflineRendering() const
{
    return m_offline_rendering;
}

bool surfelwarp::ConfigParser::isShowOnline() const
{
    return m_show_online;
}

void surfelwarp::ConfigParser::setOfflineRendering(bool offline_rendering)
{
    m_offline_rendering = offline_rendering;
}
