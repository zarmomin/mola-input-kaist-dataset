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

namespace mola
{
namespace kaist_dataset
{
/** RawDataSource from KAIST dataset.
 * Each "sequence" directory contains these sensor streams:
 * - xxx
 * - Ground truth poses
 *
 * \ingroup mola_sensor_kaist_dataset_grp */
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
    std::array<mrpt::img::TCamera, 2>  cam_intrinsics_;
    std::array<mrpt::math::TPose3D, 2> cam_poses_;  //!< wrt vehicle origin

    std::string seq_dir_;
};

}  // namespace kaist_dataset
}  // namespace mola
