#include <iostream> //标准输入输出流
#include "plyFile.h"
#include "unistd.h"
#include "types.h"

#ifdef CT_ICP_WITH_VIZ

#include <viz3d/engine.hpp>
#include <imgui.h>
#include <thread>

struct ControlSlamWindow : viz::ExplorationEngine::GUIWindow {

    explicit ControlSlamWindow(std::string &&winname) :
            viz::ExplorationEngine::GUIWindow(std::move(winname), &open) {}

    void DrawContent() override {
        ImGui::Checkbox("Pause the SLAM", &pause_button);
    };

    [[nodiscard]] bool ContinueSLAM() const {
        return !pause_button;
    }

    bool pause_button = false;
    bool open = true;
};

#endif // CT_ICP_WITH_VIZ

int main(int argc, char** argv)
{
    std::vector<Point3D> frame;
    //read ply frame file
    plyFile plyFileIn("/home/vio/Code/RobotSystem/human/src/localization/laser_localization/map/kitti.ply",
                      fileOpenMode_IN);
    char *dataIn = nullptr;
    int sizeOfPointsIn = 0;
    int numPointsIn = 0;
    plyFileIn.readFile(dataIn, sizeOfPointsIn, numPointsIn);

    for (int i(0); i < numPointsIn; i++) {
        unsigned long long int offset =
                (unsigned long long int) i * (unsigned long long int) sizeOfPointsIn;
        Point3D new_point;
        new_point.raw_pt[0] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[1] = *((float *) (dataIn + offset));
        offset += sizeof(float);
        new_point.raw_pt[2] = *((float *) (dataIn + offset));
        new_point.pt = new_point.raw_pt;
        new_point.alpha_timestamp = 0;
        frame.push_back(new_point);
    }
    frame.shrink_to_fit();

#ifdef CT_ICP_WITH_VIZ
    std::unique_ptr<std::thread> gui_thread = nullptr;
    std::shared_ptr<ControlSlamWindow> window = nullptr;

    gui_thread = std::make_unique<std::thread>(viz::ExplorationEngine::LaunchMainLoop);
    auto &instance = viz::ExplorationEngine::Instance();
    window = std::make_shared<ControlSlamWindow>("Frame");
    instance.AddWindow(window);

    auto model_ptr = std::make_shared<viz::PointCloudModel>();
    auto &model_data = model_ptr->ModelData();
    model_data.xyz.resize(frame.size());
    for (size_t i(0); i < frame.size(); ++i) {
        model_data.xyz[i][0] = frame.at(i).raw_pt[0];
        model_data.xyz[i][1] = frame.at(i).raw_pt[1];
        model_data.xyz[i][2] = frame.at(i).raw_pt[2];
    }
    instance.AddModel(1, model_ptr);

    gui_thread->join();
#endif // CT_ICP_WITH_VIZ
    return (0);
}
