#ifndef SCENE_TRANSFORMATION_DATA_PLAYER_H
#define SCENE_TRANSFORMATION_DATA_PLAYER_H

#include "TrackerBase.h"
#include "SceneTransformationDataRecorder.h"
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_timer.h>
#include <Rcs_graph.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <fstream>
#include <mutex>
#include <vector>
#include "ComponentBase.h"
#include "ActionScene.h"

namespace aff {

class SceneTransformationDataPlayer : public ComponentBase
{
public:
    SceneTransformationDataPlayer(EntityBase* parent);
    ~SceneTransformationDataPlayer();

    // Function to load transformation data from the JSON file
    void loadFromFile(const std::string& fileName);

    // Start playback from the beginning
    void startPlayback();


    // Parse data from a JSON file
    void parse(const nlohmann::json& json);

    BodyTransformation parseTransformation(const nlohmann::json& transformation);

    void getRobotBodies(RcsGraph* graph);

private:
    std::vector<TransformationRecord> recordedTransformations;
    size_t currentIndex;
    bool newDataAvailable;
    std::mutex dataMutex;
    double previousTime;
    bool playData;
    // List of transformations (child or parent names) that you want to filter out
    std::vector<std::string> robotBodies;

    void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);
    void updateGraph(RcsGraph* graph);
    


    // Helper function to apply transformations to the graph
    void applyTransformationToGraph(RcsGraph* graph, const TransformationRecord& record);


};

} // namespace aff

#endif // SCENE_TRANSFORMATION_DATA_PLAYER_H
