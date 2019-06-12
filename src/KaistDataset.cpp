/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   KaistDataset.cpp
 * @brief  RawDataSource from the KAIST urban dataset
 * @author Jose Luis Blanco Claraco
 * @date   May 9, 2019
 */

/** \defgroup mola_sensor_kaist_dataset_grp mola_sensor_kaist_dataset_grp.
 * RawDataSource from the KAIST urban dataset
 */

#include <mola-kernel/variant_helper.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-input-kaist-dataset/KaistDataset.h>
#include <mrpt/core/initializer.h>
#include <mrpt/io/vector_loadsave.h>
//#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
// Eigen must be before csv.h
#include <mrpt/io/csv.h>

using namespace mola;
using namespace mola::kaist_dataset;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(KaistDataset)}

KaistDataset::KaistDataset() = default;

void KaistDataset::initialize(const std::string& cfg_block)
{
    using namespace std::string_literals;

    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    // Mandatory parameters:
    auto c = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(base_dir, std::string);
    YAML_LOAD_MEMBER_REQ(sequence, std::string);

    // Optional params with default values:
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);

    // Preload everything we may need later to quickly replay the dataset in
    // realtime:
    seq_dir_ = base_dir_ + "/"s + sequence_ + "/sensor_data"s;
    ASSERT_DIRECTORY_EXISTS_(seq_dir_);

    const auto calib_dir = base_dir_ + "/"s + sequence_ + "/calibration"s;
    ASSERT_DIRECTORY_EXISTS_(calib_dir);

    MRPT_LOG_INFO_STREAM("Loading KAIST dataset from: " << seq_dir_);

    // Sensor poses on the vehicle:
    const auto calib_vlp_l = calib_dir + "/Vehicle2LeftVLP.txt"s;
    pose_VLP_left_         = load_pose_from_kaist_txt(calib_vlp_l);

    const auto calib_vlp_r = calib_dir + "/Vehicle2RightVLP.txt"s;
    pose_VLP_right_        = load_pose_from_kaist_txt(calib_vlp_r);

    MRPT_LOG_DEBUG_STREAM("Left VLP pose: " << pose_VLP_left_.asString());
    MRPT_LOG_DEBUG_STREAM("Right VLP pose: " << pose_VLP_right_.asString());

    MRPT_TODO("continue");
#if 0
    // Velodynes: {0,1}
    for (uint8_t cam_id = 0; cam_id < 2; cam_id++)
    {
        Eigen::Matrix<uint64_t, Eigen::Dynamic, Eigen::Dynamic> dat;

        const auto cam_data_fil =
            seq_dir_ + "/cam"s + std::to_string(cam_id) + "/data.csv"s;
        ASSERT_FILE_EXISTS_(cam_data_fil);

        mrpt::io::load_csv(cam_data_fil, dat);
        ASSERT_(dat.cols() == 2);
        ASSERT_(dat.rows() > 10);

        SensorCamera se_cam;
        se_cam.sensor_name = "cam"s + std::to_string(cam_id);
        se_cam.cam_idx     = cam_id;

        for (int row = 0; row < dat.rows(); row++)
        {
            const auto t = static_cast<euroc_timestamp_t>(dat(row, 0));
            se_cam.img_file_name =
                "/cam"s + std::to_string(cam_id) + "/data/"s +
                std::to_string(static_cast<euroc_timestamp_t>(dat(row, 1))) +
                ".png"s;

            dataset_.emplace_hint(dataset_.end(), t, se_cam);
        }
    }
#endif

    // Start at the dataset begin:
    dataset_next_ = dataset_.begin();

    MRPT_END
}  // end initialize()

void KaistDataset::spinOnce()
{
    MRPT_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    // Starting time:
    if (!replay_started_)
    {
        replay_begin_time_ = mrpt::Clock::now();
        replay_started_    = true;
    }

    // get current replay time:
    const double t =
        mrpt::system::timeDifference(replay_begin_time_, mrpt::Clock::now()) *
        time_warp_scale_;

    if (dataset_next_ == dataset_.end())
    {
        MRPT_LOG_THROTTLE_INFO(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }

    // We have to publish all observations until "t":
    while (dataset_next_ != dataset_.end() && t >= dataset_next_->first)
    {
        // Convert from dataset format:
        const auto obs_tim =
            mrpt::Clock::fromDouble(dataset_next_->first * 1e-9);

        MRPT_TODO("continue");
#if 0
        std::visit(
            overloaded{[&](std::monostate&) {
                           THROW_EXCEPTION("Un-initialized entry!");
                       },
                       [&](SensorCamera& cam) {
                           build_dataset_entry_obs(cam);
                           cam.obs->timestamp = obs_tim;
                           this->sendObservationsToFrontEnds(cam.obs);
                           cam.obs.reset();  // free mem
                       },
                       [&](SensorIMU& imu) {
                           build_dataset_entry_obs(imu);
                           imu.obs->timestamp = obs_tim;
                           this->sendObservationsToFrontEnds(imu.obs);
                           imu.obs.reset();  // free mem
                       }},
            dataset_next_->second);
#endif
        // Advance:
        ++dataset_next_;
    }

    // Read ahead to save delays in the next iteration:
    {
        ProfilerEntry tle(profiler_, "spinOnce.read_ahead");
        //
    }

    MRPT_END
}

mrpt::math::TPose3D mola::kaist_dataset::load_pose_from_kaist_txt(
    const std::string& filename)
{
    using namespace std::string_literals;

    MRPT_START
    ASSERT_FILE_EXISTS_(filename);

    std::vector<std::string> lines;
    if (!mrpt::io::loadTextFile(lines, filename))
        THROW_EXCEPTION_FMT("Error reading filename: `%s`", filename.c_str());

    // clang-format off
    /** Example content:
     *
     * Left Velodyne (VLP-16) extrinsic calibration parameter from vehicle
     * RPY(roll/pitch/yaw, degree), R(rotation matrix), T(translation matrix)
     * RPY: 1.69162 44.9466 136.579
     * R: -0.514066 -0.702201 -0.492595 0.486485 -0.711672 0.506809 -0.706447 0.0208933 0.707457
     * T: -0.440699 0.397052 1.90953*
     */
    // clang-format on

    ASSERT_(lines.size() >= 5);
    ASSERT_(lines[3].substr(0, 2) == "R:"s);
    ASSERT_(lines[4].substr(0, 2) == "T:"s);

    const auto rot_str   = "["s + lines[3].substr(2) + "]"s;
    const auto trans_str = "["s + lines[4].substr(2) + "]"s;

    mrpt::math::CMatrixDouble rot_mat;
    if (!rot_mat.fromMatlabStringFormat(rot_str))
        THROW_EXCEPTION_FMT(
            "Error parsing rotation matrix line: `%s`", lines[3].c_str());
    ASSERT_EQUAL_(rot_mat.rows(), 1UL);
    ASSERT_EQUAL_(rot_mat.cols(), 9UL);

    mrpt::math::CMatrixDouble trans_mat;
    if (!trans_mat.fromMatlabStringFormat(trans_str))
        THROW_EXCEPTION_FMT(
            "Error parsing translation line: `%s`", lines[4].c_str());
    ASSERT_EQUAL_(trans_mat.rows(), 1UL);
    ASSERT_EQUAL_(trans_mat.cols(), 3UL);

    mrpt::math::CMatrixDouble33 R;
    for (int i = 0; i < 9; i++) R(i) = rot_mat(0, i);

    mrpt::poses::CPose3D p;
    p.setRotationMatrix(R);
    p.x(trans_mat(0, 0));
    p.y(trans_mat(0, 1));
    p.z(trans_mat(0, 2));

    return p.asTPose();
    MRPT_END
}
