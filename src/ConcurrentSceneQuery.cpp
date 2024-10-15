/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "ConcurrentSceneQuery.h"
#include "SceneJsonHelpers.h"
#include "ExampleActionsECS.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_timer.h>
#include <URDFGenerator.h>

#include <unordered_set>



namespace aff
{


SceneQueryPool::SceneQueryPool(const ExampleActionsECS* sim, size_t nInstances)
{
  for (size_t i = 0; i < nInstances; ++i)
  {
    queries.push_back(std::make_shared<ConcurrentSceneQuery>(sim));
  }
}

std::shared_ptr<ConcurrentSceneQuery> SceneQueryPool::instance()
{
  std::lock_guard<std::mutex> lock(mtx);

  for (size_t i = 0; i < queries.size(); ++i)
  {
    if (queries[i].use_count() == 1)
    {
      NLOG_CPP(0, "Returning query " << i);
      return queries[i];
    }
  }

  NLOG_CPP(0, "No query found");
  return nullptr;
}

bool SceneQueryPool::test(const ExampleActionsECS* sim)
{
  RLOG(0, "SceneQueryFactory::test()");
  SceneQueryPool f(sim, 3);
  bool success = true;

  {
    auto sq1 = f.instance();
    auto sq2 = f.instance();
    if (sq1 && sq2)
    {
      RLOG_CPP(1, "SUCCESS: Getting 2 instances out of 3");
    }
    else
    {
      RLOG_CPP(1, "FAIL: Getting 2 instances out of 3");
      success = false;
    }
  }

  auto sq3 = f.instance();
  auto sq4 = f.instance();
  auto sq5 = f.instance();
  auto sq6 = f.instance();

  if (sq3 && sq4 && sq5 && !sq6)
  {
    RLOG_CPP(1, "SUCCESS: Getting 3 instances out of 3, not getting the 4th one");
  }
  else
  {
    RLOG_CPP(1, "FAIL: Getting 3 instances out of 3, not getting the 4th one");
    success = false;
  }

  return success;
}




ConcurrentSceneQuery::ConcurrentSceneQuery(const ExampleActionsECS* sim_) :
  sim(sim_), graph(NULL), broadphase(NULL)
{
  RCHECK(sim);
  sim->lockStepMtx();
  graph = RcsGraph_clone(sim->getGraph());
  scene = *(sim->getScene());
  sim->unlockStepMtx();
}

ConcurrentSceneQuery::~ConcurrentSceneQuery()
{
  RcsGraph_destroy(graph);
  RcsBroadPhase_destroy(this->broadphase);
}

void ConcurrentSceneQuery::update(bool withBroadphase)
{
  sim->lockStepMtx();
  updateNoMutex(withBroadphase);
  sim->unlockStepMtx();
}

void ConcurrentSceneQuery::updateNoMutex(bool withBroadphase)
{
  RcsGraph_copy(this->graph, sim->getGraph());
  this->scene = *(sim->getScene());

  if (withBroadphase)
  {
    RcsBroadPhase_destroy(this->broadphase);
    this->broadphase = RcsBroadPhase_clone(sim->getBroadPhase(), graph);
  }
}

nlohmann::json ConcurrentSceneQuery::getSceneState()
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  nlohmann::json json;
  aff::getSceneState(json, &scene, graph);
  return json;
}

std::string ConcurrentSceneQuery::getURDF()
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return Rcs::URDFGenerator(graph).toString();
}

nlohmann::json ConcurrentSceneQuery::getOccludedObjectsForAgent(const std::string& agentName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::getOccludedObjectsForAgent(agentName, &scene, graph);
}

nlohmann::json ConcurrentSceneQuery::getObjectOccludersForAgent(const std::string& agentName,
                                                                const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::getObjectOccludersForAgent(agentName, objectName, &scene, graph);
}


nlohmann::json ConcurrentSceneQuery::getObjectInCamera(const std::string& objectName,
                                                       const std::string& cameraName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::getObjectInCamera(objectName, cameraName, &scene, graph);
}

std::string ConcurrentSceneQuery::getParentEntity(const std::string& objectName)
{
  RLOG_CPP(0, "Checking parent for " << objectName);
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  const AffordanceEntity* ntt = scene.getAffordanceEntity(objectName);
  if (!ntt)
  {
    RLOG_CPP(0, "No entity found with name " << objectName);
    return std::string();
  }
  const AffordanceEntity* parent = scene.getParentAffordanceEntity(graph, ntt);

  if (parent)
  {
    return parent->name;
  }

  const Manipulator* holdingHand = scene.getParentManipulator(graph, ntt);

  return holdingHand ? holdingHand->name : std::string();
}

std::string ConcurrentSceneQuery::getClosestParentAffordance(const std::string& objectName,
                                                             const std::string& parentAffordanceType)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  const AffordanceEntity* ntt = scene.getAffordanceEntity(objectName);
  if (!ntt)
  {
    RLOG_CPP(1, "No entity found with name " << objectName);
    return std::string();
  }

  const RcsBody* nttBdy = ntt->body(graph);
  if (!nttBdy)
  {
    RLOG_CPP(1, "Entity " << objectName << " has no RcsBody attached");
    return std::string();
  }

  const AffordanceEntity* parent = scene.getParentAffordanceEntity(graph, ntt);
  if (!parent)
  {
    RLOG_CPP(1, "No parent entity found for " << objectName);
    return std::string();
  }

  // From here, we have a parent. We collect all affordances with the name
  double d_min = DBL_MAX;
  std::string closestFrame;
  for (const auto& parentAff : parent->affordances)
  {
    RLOG_CPP(1, "Checking affordance " << parentAff->frame << " of type " << parentAff->className);

    if (parentAffordanceType!=parentAff->className)
    {
      RLOG_CPP(1, "Ignoring affordance " << parentAff->frame << " of type " << parentAff->className);
      continue;
    }

    const RcsBody* frm_i = parentAff->getFrame(graph);
    if (frm_i)
    {
      double d = Vec3d_distance(frm_i->A_BI.org, nttBdy->A_BI.org);

      if (d < d_min)
      {
        d_min = d;
        closestFrame = parentAff->frame;
        RLOG_CPP(1, "Current closest affordance is " << closestFrame << " of type " << parentAff->className);
      }
    }

  }

  RLOG_CPP(1, "Closest frame of type " << parentAffordanceType << " to entity " << objectName
           << " is " << closestFrame << " with distance " << std::to_string(d_min));

  return closestFrame;
}

bool ConcurrentSceneQuery::isAgentBusy(const std::string& agentName, double distanceThreshold)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  return aff::isAgentBusy(agentName, &scene, graph, distanceThreshold);
}

std::unique_ptr<PredictionTree>
ConcurrentSceneQuery::planActionTree(PredictionTree::SearchType searchType,
                                     const std::vector<std::string>& actions,
                                     double dt,
                                     size_t maxThreads,
                                     bool earlyExitSearch,
                                     bool earlyExitAction)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update(true);

  return PredictionTree::planActionTree(searchType, scene, graph, broadphase, actions,
                                        dt, maxThreads,
                                        earlyExitSearch, earlyExitAction);
}

std::vector<double> ConcurrentSceneQuery::getPanTilt(const std::string& roboAgent,
                                                     const std::string& gazeTarget)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();

  const aff::Agent* robo_ = scene.getAgent(roboAgent);
  const aff::RobotAgent* robo = dynamic_cast<const aff::RobotAgent*>(robo_);
  if (!robo)
  {
    RLOG(0, "Robot agent '%s' not found", roboAgent.c_str());
    return std::vector<double>();
  }

  double panTilt[2], err[2];
  size_t maxIter = 100;
  double eps = 1.0e-3;   // 1mm

  double t_calc = Timer_getSystemTime();

  std::string resolvedTarget;

  // Resolve if gaze Target is a scene entity
  const AffordanceEntity* gazeNtt = scene.getAffordanceEntity(gazeTarget);
  if (gazeNtt)
  {
    resolvedTarget = gazeNtt->bdyName;
  }
  RLOG_CPP(1, "Checking AffordanceEntity: " << resolvedTarget);

  // If no scene entity is found with the gaze target, we look for agents. In this
  // case, we look at the agent's head.
  if (resolvedTarget.empty())
  {
    const Agent* gazeAgent = scene.getAgent(gazeTarget);
    if (gazeAgent)
    {
      auto m = gazeAgent->getManipulatorsOfType(&scene, "head");
      if (!m.empty())
      {
        resolvedTarget = m[0]->bdyName;
      }
    }
  }
  RLOG_CPP(1, "Checking Agents: " << resolvedTarget);

  // If no agent is found with the gaze target, we look for manipulators
  if (resolvedTarget.empty())
  {
    const Manipulator* m = scene.getManipulator(gazeTarget);
    if (m)
    {
      resolvedTarget = m->bdyName;
    }
  }
  RLOG_CPP(1, "Checking Manipulators: " << resolvedTarget);


  // If none of these was found, we pass it straight through and hope it is a RcsBody name
  if (resolvedTarget.empty())
  {
    resolvedTarget = gazeTarget;
  }
  RLOG_CPP(1, "Before pan-tilt: " << resolvedTarget);

  // This calls the actual pan-tilt calculation
  int iter = robo->getPanTilt(graph, resolvedTarget, panTilt, maxIter, eps, err);

  // Can't retrieve joints / bodies etc.
  if (iter==-1)
  {
    RLOG(0, "Pan tilt computation failed due to missing joints / bodies / graph");
    return std::vector<double>();
  }

  // Couldn't converge to precision given in eps
  if ((err[0]>eps) || (err[1]>eps))
  {
    RLOG(0, "Pan tilt computation did not converge for gaze Target '%s' (resolvedTarget: '%s')",
         gazeTarget.c_str(), resolvedTarget.c_str());
    const RcsBody* gazeBdy = RcsGraph_getBodyByName(graph, resolvedTarget.c_str());
    if (gazeBdy)
    {
      RLOG(0, "Goal position: %f %f %f", gazeBdy->A_BI.org[0], gazeBdy->A_BI.org[1], gazeBdy->A_BI.org[2]);
    }
    else
    {
      RLOG(0, "Can't print coordinates - RcsBody not found");
    }

    return std::vector<double>();
  }

  t_calc = Timer_getSystemTime() - t_calc;

  RLOG(1, "pan=%.1f tilt=%.1f err=%f %f took %d iterations and %.1f msec",
       RCS_RAD2DEG(panTilt[0]), RCS_RAD2DEG(panTilt[1]), err[0], err[1], iter, 1.0e3*t_calc);

  return std::vector<double>(panTilt, panTilt+2);
}

nlohmann::json ConcurrentSceneQuery::getObjects()
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  nlohmann::json json;
  json["objects"] = std::vector<nlohmann::json>();

  // Only add each item once in case of duplicate names.
  std::unordered_set<std::string> ntts;
  for (const auto& e : scene.entities)
  {
    ntts.insert(e.name);
  }

  // Assemble the json
  for (const auto& n : ntts)
  {
    json["objects"].push_back(n);
  }

  return json;
}

nlohmann::json ConcurrentSceneQuery::getObjectsHeldBy(const std::string& agentName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  nlohmann::json json;
  json["objects"] = std::vector<nlohmann::json>();

  auto agent = scene.getAgent(agentName);

  if (!agent)
  {
    return json;
  }

  auto hands = agent->getManipulatorsOfType(&scene, "hand");

  for (const auto& h : hands)
  {
    auto graspedNtts = h->getGraspedEntities(scene, graph);

    for (const auto& ntt : graspedNtts)
    {
      json["objects"].push_back(ntt->name);
    }
  }

  return json;
}

nlohmann::json ConcurrentSceneQuery::getAgents()
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  nlohmann::json json;
  json["agents"] = std::vector<nlohmann::json>();

  for (const auto& a : scene.agents)
  {
    json["agents"].push_back(a->name);
  }

  return json;
}

std::string ConcurrentSceneQuery::getHoldingHand(const std::string& objectName)
{
  std::lock_guard<std::mutex> lock(reentrancyLock);
  update();
  auto ntts = scene.getAffordanceEntities(objectName);

  if (ntts.empty())
  {
    return std::string();
  }

  for (const auto& ntt : ntts)
  {
    const Manipulator* hand = scene.getGraspingHand(graph, ntt);
    if (hand)
    {
      return hand->name;
    }
  }

  return std::string();
}


nlohmann::json ConcurrentSceneQuery::getGazeData()
{
  sim->lockStepMtx();
  nlohmann::json gazeData = sim->getGazeData();
  sim->unlockStepMtx();
  return gazeData;
}

void ConcurrentSceneQuery::saveGazeDataToFile(const std::string& directory, const std::string& filename)
{
  sim->lockStepMtx();
  sim->saveGazeDataToFile(directory,filename);
  sim->unlockStepMtx();
}



}   // namespace aff
