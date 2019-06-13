/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   KaistDataset.h
 * @brief  RawDataSource from the KAIST urban dataset
 * @author Jose Luis Blanco Claraco
 * @date   May 9, 2019
 */
#pragma once

#include <mola-kernel/RawDataSourceBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPose3D.h>
#include <array>
#include <map>
#include <string>
#include <variant>

namespace mola::kaist_dataset
{
/** \addtogroup mola_sensor_kaist_dataset_grp
 * @{ */

/** Loads a pose from a TXT in KAIST format. Throws on error. */
mrpt::math::TPose3D load_pose_from_kaist_txt(const std::string& filename);

struct SensorCamera
{
    std::string                  sensor_name;
    std::string                  img_file_name;
    uint8_t                      cam_idx;
    mrpt::obs::CObservation::Ptr obs;
};
struct SensorIMU
{
    std::string                  sensor_name;
    double                       wx, wy, wz, accx, accy, accz;
    mrpt::obs::CObservation::Ptr obs;
};
struct SensorVelodyne
{
    std::string                  sensor_name;
    mrpt::obs::CObservation::Ptr obs;
};
struct SensorSICK
{
    std::string                  sensor_name;
    mrpt::obs::CObservation::Ptr obs;
};
using SensorEntry = std::variant<
    std::monostate, SensorCamera, SensorIMU, SensorVelodyne, SensorSICK>;
using kaist_timestamp_t = uint64_t;
using kaist_dataset_t   = std::multimap<kaist_timestamp_t, SensorEntry>;

/** RawDataSource from KAIST dataset.
 * Each "sequence" directory contains these sensor streams:
 * - xxx
 * - Ground truth poses
 *
 */
class KaistDataset : public RawDataSourceBase
{
   public:
    KaistDataset();
    ~KaistDataset() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

   private:
    std::string             base_dir_;  //!< base dir for `xxx/xx/mav0/...`
    std::string             sequence_;  //!< e.g. `machine_hall/MH_01_easy`
    mrpt::Clock::time_point replay_begin_time_{};
    bool                    replay_started_{false};
    double                  time_warp_scale_{1.0};
    bool                    publish_VLP_left{true}, publish_VLP_right{true};
    bool                    publish_SICK_back{true}, publish_SICK_middle{true};

    // All poses are wrt vehicle frame of reference
    // (refer to figures in KAIST paper)
    mrpt::math::TPose3D pose_VLP_left_, pose_VLP_right_;

    std::string seq_dir_;  //!< selected "sensor_data" directory

    kaist_dataset_t           dataset_;  //!< dataset itself
    kaist_dataset_t::iterator dataset_next_;  //!< next item to publish
};

/** @} */

}  // namespace mola::kaist_dataset
