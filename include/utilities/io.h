#ifndef LASER_LOCALIZATION_IO_H
#define LASER_LOCALIZATION_IO_H

#include "utilities/types.h"

// Saves Poses to disk, and returns whether the writing was successful
bool SavePoses(const std::string& file_path, const ArrayPoses &);

// Saves Trajectory Frames to disk, and returns whether the writing was successful
bool SaveTrajectoryFrame(const std::string& file_path, const std::vector<TrajectoryFrame>&);

// Loads Poses from disk. Raises a std::runtime_error if it fails to do so
ArrayPoses LoadPoses(const std::string& file_path);

std::vector<TrajectoryFrame> LoadTrajectory(const std::string& file_path);


#endif //LASER_LOCALIZATION_IO_H
