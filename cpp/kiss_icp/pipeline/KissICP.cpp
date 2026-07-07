// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "KissICP.hpp"

#include <Eigen/Core>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps,
                                                    std::optional<double> cable_depth) {
    // Preprocess the input cloud
    const auto &preprocessed_frame = preprocessor_.Preprocess(frame, timestamps, last_delta_);

    // Voxelize
    const auto &[source, frame_downsample] = Voxelize(preprocessed_frame);

    // Get adaptive_threshold
    const double sigma = adaptive_threshold_.ComputeThreshold();

    // Compute initial_guess for ICP
    auto initial_guess = last_pose_ * last_delta_;

    // Cable-encoder anchor: only active when enabled in config and a depth sample is available
    const bool cable_anchor_active = config_.use_cable_anchor && cable_depth.has_value();
    const double cable_weight = cable_anchor_active ? config_.cable_anchor_weight : 0.0;
    const double cable_z = cable_depth.value_or(0.0);

    // Strategy A: project the initial guess so its depth along gravity_dir matches the cable
    if (cable_anchor_active) {
        const Eigen::Vector3d &n = config_.gravity_dir;
        const double z_pred = n.dot(initial_guess.translation());
        initial_guess.translation() += (cable_z - z_pred) * n;
    }

    // Run ICP (Strategy B: soft depth anchor added inside the registration)
    const auto new_pose = registration_.AlignPointsToMap(source,               // frame
                                                         local_map_,           // voxel_map
                                                         initial_guess,        // initial_guess
                                                         3.0 * sigma,          // max_corr_dist
                                                         sigma,                // kernel
                                                         config_.gravity_dir,  // n
                                                         cable_z,              // cable_depth
                                                         cable_weight);        // anchor weight

    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // Update step: threshold, local map, delta, and the last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    last_delta_ = last_pose_.inverse() * new_pose;
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (preprocessed_frame) and the points used for registration
    // (source)
    return {preprocessed_frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}
void KissICP::Reset() {
    last_pose_ = Sophus::SE3d();
    last_delta_ = Sophus::SE3d();

    // Clear the local map
    local_map_.Clear();

    // Reset adaptive threshold (it will start fresh)
    adaptive_threshold_ =
        AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

}  // namespace kiss_icp::pipeline
