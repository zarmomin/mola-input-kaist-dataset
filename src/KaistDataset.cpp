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
#include <mola-sensor-kaist-dataset/KaistDataset.h>
#include <mrpt/core/initializer.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>  //ASSERT_DIRECTORY_EXISTS_()
#include <yaml-cpp/yaml.h>

using namespace mola;
using namespace mola::euroc_dataset;

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

    seq_dir_ = base_dir_ + "/"s + sequence_ + "/mav0"s;
    ASSERT_DIRECTORY_EXISTS_(seq_dir_);
    // ASSERT_DIRECTORY_EXISTS_(seq_dir_ + "/cam0"s);

    // Optional params with default values:
    YAML_LOAD_MEMBER_OPT(time_warp_scale, double);

    // Preload everything we may need later to quickly replay the dataset in
    // realtime:
    MRPT_LOG_INFO_STREAM("Loading KAIST dataset from: " << seq_dir_);

    MRPT_TODO("continue");

    // Start at the dataset begin:
    // dataset_next_ = dataset_.begin();

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

    MRPT_END
}
