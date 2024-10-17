#include "SceneTransformationDataRecorder.h"
#include <iostream>
#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_math.h>
#include <Rcs_typedef.h>

namespace aff
{

    SceneTransformationDataRecorder::SceneTransformationDataRecorder(EntityBase *parent, size_t maxNumRecords_) : ComponentBase(parent), maxNumRecords(maxNumRecords_)
    {
        // Initialization or any setup can be done here
        RLOG(1, "SceneTransformationDataRecorder initialized");
        subscribe("PostUpdateGraph", &SceneTransformationDataRecorder::onPostUpdateGraph);
    }

    SceneTransformationDataRecorder::~SceneTransformationDataRecorder()
    {
        RLOG(1, "SceneTransformationDataRecorder destroyed");
    }


    void SceneTransformationDataRecorder::addBodyAndParents(const RcsBody *body, std::map<std::string, std::string> &bodyParentMap, const RcsGraph *graph)
    {
        if (!body)
            return;

        // Check if the body is already in the map
        if (bodyParentMap.find(body->name) == bodyParentMap.end())
        {
            // Get the parent of the current body
            RcsBody *parent = RcsBody_getParent(const_cast<RcsGraph *>(graph), const_cast<RcsBody *>(body));

            // If there is a parent, add the body and its parent to the map
            if (parent)
            {
                RLOG_CPP(0, "Added body: " << body->name << " with parent: " << parent->name);
                bodyParentMap[body->name] = parent->name;

                // Recursively add the parent and its ancestors
                addBodyAndParents(parent, bodyParentMap, graph);
            }
            else
            {
                // If there is no parent, just add the body with an empty parent name
                RLOG_CPP(0, "Added body: " << body->name << " with no parent");
                bodyParentMap[body->name] = ""; // Empty parent indicates no parent
            }
        }
    }

    void SceneTransformationDataRecorder::addSceneToRecord(const ActionScene &scene, const RcsGraph *graph)
    {
        // Clear the bodyParentMap before adding new data
        bodyParentMap.clear();

        // Add agents (skeletons) to record
        std::vector<const aff::Agent *> agents = scene.getAgents<aff::Agent>();
        for (const auto &agent : agents)
        {
            RLOG_CPP(0, "Adding agent " << agent->name << " to SceneTransformationDataRecorder");
            const RcsBody *agentBody = RcsGraph_getBodyByName(graph, agent->bdyName.c_str());
            if (agentBody)
            {
                addBodyAndParents(agentBody, bodyParentMap, graph);
            }
            else
            {
                RLOG_CPP(0, "Agent body not found: " << agent->name);
            }
        }

        // Add scene objects to record
        auto ntts = scene.getSceneEntities();
        for (const auto &ntt : ntts)
        {
            RLOG_CPP(0, "Adding object " << ntt->name << " to SceneTransformationDataRecorder");
            const RcsBody *objectBody = RcsGraph_getBodyByName(graph, ntt->bdyName.c_str());
            if (objectBody)
            {
                addBodyAndParents(objectBody, bodyParentMap, graph);
            }
            else
            {
                RLOG_CPP(0, "Object body not found: " << ntt->name);
            }
        }

        RLOG_CPP(0, "Total bodies and parent transformations recorded: " << bodyParentMap.size());
    }

    void SceneTransformationDataRecorder::onPostUpdateGraph(RcsGraph *desired, RcsGraph *current)
    {
        // You can get skeleton data from the current RcsGraph or any other component
        // Here you should extract skeleton data from the current graph and record it

        const RcsGraph *graph = desired;

        double currentTime = Timer_getSystemTime();

        recordTransformationsAtTime(currentTime, graph);

        RLOG(1, "Recorded transformations at time: %f", currentTime);
        // NUMBER OF RECORDED TRANSFORMATIONS
        RLOG(1, "Number of recorded transformations: %zu", recordedTransformations.size());
    }

    void SceneTransformationDataRecorder::recordTransformationsAtTime(double currentTime, const RcsGraph *graph)
    {
        TransformationRecord record(currentTime);

        // Iterate through the bodyParentMap and compute relative transformations
        for (const auto &pair : bodyParentMap)
        {
            const std::string &child = pair.first;
            const std::string &parent = pair.second;

            const RcsBody *childBody = RcsGraph_getBodyByName(graph, child.c_str());
            const RcsBody *parentBody = RcsGraph_getBodyByName(graph, parent.c_str());

            if (childBody && parentBody)
            {
                HTr relativeTransformation;
                HTr_invTransform(&relativeTransformation, &parentBody->A_BI, &childBody->A_BI);

                // Add the transformation to the record
                record.addTransformation(parent, child, relativeTransformation);
            }
        }

        // Add the new record to the deque
        recordedTransformations.push_back(record);

        // Remove the oldest record if we exceed the maximum number of records
        if (recordedTransformations.size() > maxNumRecords)
        {
            recordedTransformations.pop_front();
        }
    }
    
    // Get the recorded transformations
    const std::deque<TransformationRecord>& SceneTransformationDataRecorder::getRecordedTransformations() const
    {
        return recordedTransformations;
    }


} // namespace aff
