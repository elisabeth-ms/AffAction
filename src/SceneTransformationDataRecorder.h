#ifndef AFF_SCENETRANSFORMATIONDATARECORDER_H
#define AFF_SCENETRANSFORMATIONDATARECORDER_H

#include "ComponentBase.h"
#include <json.hpp>
#include <string>
#include <deque>
#include <Rcs_graph.h>
#include "ActionScene.h"
#include <set>

namespace aff
{

    // Structure to represent a parent-child transformation
struct BodyTransformation
{
    std::string parent;
    std::string child;
    HTr relativeTransformation;

    BodyTransformation(const std::string& parent_, const std::string& child_, const HTr& relativeTrf)
        : parent(parent_), child(child_), relativeTransformation(relativeTrf)
    {
        // Initialize transformation values
        HTr_setIdentity(&relativeTransformation);
        HTr_copy(&relativeTransformation, &relativeTrf);
    }
};

// Structure to hold time and all transformations at that time
struct TransformationRecord
{
    double time;
    std::vector<BodyTransformation> transformations;

    TransformationRecord(double t)
        : time(t)
    {
    }

    void addTransformation(const std::string& parent, const std::string& child, const HTr& relativeTrf)
    {
        transformations.emplace_back(parent, child, relativeTrf);
    }
};


class SceneTransformationDataRecorder : public ComponentBase
{
public:

    SceneTransformationDataRecorder(EntityBase* parent, size_t maxNumRecords_);
                       

    /*! \brief Destructor, clears any stored data. */
    virtual ~SceneTransformationDataRecorder();


    void addSceneToRecord(const ActionScene& scene, const RcsGraph* graph);

    void addBodyAndParents(const RcsBody* body, std::map<std::string, std::string>& bodyParentMap, const RcsGraph* graph);

    const std::deque<TransformationRecord>& getRecordedTransformations() const;


private:

    void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
    void recordTransformationsAtTime(double currentTime, const RcsGraph* graph);

    std::map<std::string, std::string> bodyParentMap;
    std::deque<TransformationRecord> recordedTransformations;
    size_t maxNumRecords;  // Maximum number of records to store
};

}   // namespace aff

#endif   // AFF_SCENETRANSFORMATIONDATARECORDER_H
