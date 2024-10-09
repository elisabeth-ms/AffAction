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

#include "ActionPut.h"
#include "ActionFactory.h"
#include "CollisionModelConstraint.h"
#include "StringParserTools.hpp"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <EulerConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_utilsCPP.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>

#include <algorithm>



#define fingersOpen   (0.01)
#define fingersClosed (0.7)
#define t_fingerMove  (2.0)

#define IS_NEAR_THRESHOLD  (0.4)
#define DEFAULT_ABOVE_DIST (0.2)


namespace aff
{
template<typename T>
static void eraseAffordancesNearerOrFartherAgent(const ActionScene& domain,
                                                 const RcsGraph* graph,
                                                 const Agent* nearToAgent,
                                                 double d_limit,
                                                 bool eraseIfRechable,
                                                 std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  const RcsBody* agentFrame = nearToAgent->body(graph);
  auto it = affordanceMap.begin();

  while (it != affordanceMap.end())
  {
    const Affordance* stackable = std::get<0>(*it);
    const RcsBody* putFrame = stackable->getFrame(graph);
    bool reachable;

    if (d_limit == 0.0)
    {
      reachable = nearToAgent->canReachTo(&domain, graph, putFrame->A_BI.org);
    }
    else
    {
      reachable = Vec3d_distance(agentFrame->A_BI.org, putFrame->A_BI.org) <= d_limit ? true : false;
    }

    if (eraseIfRechable)
    {
      it = reachable ? affordanceMap.erase(it) : it + 1;
    }
    else
    {
      it = reachable ? it + 1 : affordanceMap.erase(it);
    }
  }

}

template<typename T>
static void eraseAffordancesNearerOrFartherNtt(const ActionScene& domain,
                                               const RcsGraph* graph,
                                               const std::vector<const AffordanceEntity*>& nearToNtts,
                                               double d_limit,
                                               bool trueForNearer,
                                               std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  bool trueForFarther = !trueForNearer;

  // Memorize the frames of all entities that are to be checked against
  // closeness. We store them up front to avoid searching them in each
  // iteration of the affordanceMap.
  std::vector<const RcsBody*> nttFrames;
  for (const auto& ntt : nearToNtts)
  {
    nttFrames.push_back(ntt->body(graph));
  }

  auto it = affordanceMap.begin();
  while (it != affordanceMap.end())
  {
    const Affordance* affordance = std::get<0>(*it);
    const T* supportable = dynamic_cast<const T*>(affordance);

    // We only consider affordances with the given template type
    if (!supportable)
    {
      it++;
      continue;
    }

    // Here we go through all relevant entity frames, and check if any of
    // the entities is below the limit distance. If it is the case, we can
    // safely keep the affordance and do an early exit of the loop.
    const RcsBody* putFrame = supportable->getFrame(graph);
    bool eraseSupportable = trueForFarther;
    for (const auto& nttFrame : nttFrames)
    {
      double di = Vec3d_distance(nttFrame->A_BI.org, putFrame->A_BI.org);
      if (di <= d_limit)
      {
        eraseSupportable = !trueForFarther;
        RLOG(5, "%s - %s is %s (d=%f, limit=%f)",
             nttFrames[0]->name, putFrame->name,
             (eraseSupportable ? "erased" : "not erased"), di, d_limit);
        break;
      }
    }

    it = eraseSupportable ? affordanceMap.erase(it) : it + 1;
  }

}

// Prune T's that are farther away than d_limit
static void eraseSupportablesFarther(const ActionScene& domain,
                                     const RcsGraph* graph,
                                     const std::string& ntt,
                                     double d_limit,
                                     std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(ntt);

  if (!ntts.empty())
  {
    eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntts, d_limit,
                                                    false, affordanceMap);
  }

  const Agent* agent = domain.getAgent(ntt);

  if (agent)
  {
    eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, agent, d_limit,
                                                      false, affordanceMap);
  }

}

// Prune T's that are closer than d_limit
static void eraseSupportablesNearer(const ActionScene& domain,
                                    const RcsGraph* graph,
                                    const std::string& ntt,
                                    double d_limit,
                                    std::vector<std::tuple<Affordance*, Affordance*>>& affordanceMap)
{
  std::vector<const AffordanceEntity*> ntts = domain.getAffordanceEntities(ntt);

  if (!ntts.empty())
  {
    eraseAffordancesNearerOrFartherNtt<Supportable>(domain, graph, ntts, d_limit,
                                                    true, affordanceMap);
  }

  const Agent* agent = domain.getAgent(ntt);

  if (agent)
  {
    eraseAffordancesNearerOrFartherAgent<Supportable>(domain, graph, agent, d_limit,
                                                      true, affordanceMap);
  }
}

}   // namespace

namespace aff
{
REGISTER_ACTION(ActionPut, "put");

ActionPut::ActionPut() :
  above(false), putDown(false), isObjCollidable(false), isPincerGrasped(false), putPolar(true),
  supportRegionX(0.0), supportRegionY(0.0), polarAxisIdx(2), distance(0.0), heightAboveGoal(0.0)
{
  Vec3d_setZero(startPoint);
  Vec3d_setZero(midPoint);
  Vec3d_setZero(endPoint);
  Vec3d_setZero(putOri3d);
}

ActionPut::ActionPut(const ActionScene& domain,
                     const RcsGraph* graph,
                     std::vector<std::string> params) : ActionPut()
{
  RLOG_CPP(0, "Calling ActionPut with params: " << Rcs::String_concatenate(params, " "));
  parseArgs(domain, graph, params);

  std::string objectToPut = params[0];
  std::string surfaceToPutOn = params.size() > 1 ? params[1] : std::string();

  if (objectToPut == surfaceToPutOn)
  {
    throw ActionException(ActionException::UnrecoverableError,
                          "The " + objectToPut + " cannot be put on itself.",
                          "Put it onto another object in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  const AffordanceEntity* object = initHands(domain, graph, objectToPut);

  this->affordanceMap = initOptions(domain, graph, object, surfaceToPutOn);

  // Initialize with the best solution.
  bool successInit = initialize(domain, graph, 0);
  RCHECK(successInit);
}

void ActionPut::parseArgs(const ActionScene& domain,
                          const RcsGraph* graph,
                          std::vector<std::string>& params)
{
  parseParams(params);

  int res = getAndEraseKeyValuePair(params, "frame", whereOn);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());

  res = getAndEraseKeyValuePair(params, "distance", distance);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());

  double alignAngleInRad = 0.0;
  res = getAndEraseKeyValuePair(params, "alignAngleZ", alignAngleInRad);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());
  if (res == 0)
  {
    putOri3d[2] = RCS_DEG2RAD(alignAngleInRad);
  }

  res = getAndEraseKeyValuePair(params, "height", heightAboveGoal);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());

  if (getAndEraseKey(params, "above"))
  {
    above = true;
    heightAboveGoal = DEFAULT_ABOVE_DIST;
  }

  if (getAndEraseKey(params, "putDown"))
  {
    putDown = true;
  }

  if (getAndEraseKey(params, "putAligned"))
  {
    putPolar = false;
  }

  res = getAndEraseKeyValuePair(params, "near", nearTo);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());
  if ((res == 0) && (!nearTo.empty()) && domain.getSceneEntities(nearTo).empty())
  {
    throw ActionException(ActionException::UnrecoverableError,
                          "Cannot put an object near " + nearTo + " because " + nearTo + " is unknown",
                          "Put it near another object in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  res = getAndEraseKeyValuePair(params, "far", farFrom);
  RCHECK_MSG(res >= -1, "%s", Rcs::String_concatenate(params, " ").c_str());
  if ((res==0) && (!farFrom.empty()) && domain.getSceneEntities(farFrom).empty())
  {
    throw ActionException(ActionException::UnrecoverableError,
                          "Cannot put an object far from " + farFrom + " because " + farFrom + " is unknown",
                          "Put it far away of another object in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

}

// This function assigns the following member variables:
// - this->objName (initialize)
// - this->isObjCollidable (used in createTrajectory)
// - this->usedManipulators (first one is grasping hand)
// - this->isPincerGrasped (tasks and trajectory generation)
// - this->graspFrame (used in createTasks)
// - this->objGraspFrame (used in createTasks)
const AffordanceEntity* ActionPut::initHands(const ActionScene& domain,
                                             const RcsGraph* graph,
                                             const std::string& objectToPut)
{
  // Initialize object to put
  std::vector<const AffordanceEntity*> objects = domain.getAffordanceEntities(objectToPut);

  if (objects.empty())
  {
    throw ActionException(ActionException::UnrecoverableError,
                          "The " + objectToPut + " is unknown.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  auto gPair = domain.getGraspingHand(graph, objects);
  const Manipulator* graspingHand = std::get<0>(gPair);
  const AffordanceEntity* object = std::get<1>(gPair);

  if (!graspingHand)
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "The " + objectToPut + " is not held in the hand.",
                          "First get the " + objectToPut + " in the hand before performing this command",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  if (getAffordances<Stackable>(object).empty())
  {
    throw ActionException(ActionException::KinematicallyImpossible,
                          "I don't know how to place the " + objectToPut + " on a surface.",
                          "Try to hand it over, or try something else",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  this->objName = object->bdyName;
  this->isObjCollidable = object->isCollideable(graph);
  this->usedManipulators.push_back(graspingHand->name);

  // We keep the other manipulator if it is occupied
  auto occupiedManipulators = domain.getOccupiedManipulators(graph);
  for (auto m : occupiedManipulators)
  {
    if (m->name != graspingHand->name)
    {
      usedManipulators.push_back(m->name);
    }
  }

  // From here on we know that the object is held in a manipulator. We determine the
  // frame from which we retract the hand after we opened the fingers.
  auto graspCapability = graspingHand->getGraspingCapability(graph, object);

  if (!graspCapability)
  {
    throw ActionException(ActionException::ParamInvalid, "Cannot grasp object", "",
                          "graspCapability cannot grasp " + object->name);
  }

  this->isPincerGrasped = dynamic_cast<const PincergraspCapability*>(graspCapability);
  this->graspFrame = graspCapability->frame;
  auto ca = graspingHand->getGrasp(graph, object);

  // Determine the frame of the object at which it is grasped. This is aligned with
  // the hand's grasping frame (not the orientation necessarily if powergrasped \todo(MG)).
  RLOG_CPP(4, "Grasp frame distance is " << std::get<2>(ca));
  const Affordance* a_grasp = std::get<1>(ca);

  if (!a_grasp)
  {
    throw ActionException(ActionException::UnknownError, "Internal error", "",
                          "Internal error: graspingHand->getGrasp failed with entity " + object->name);
  }

  this->objGraspFrame = a_grasp->frame;

  return object;
}

std::vector<std::tuple<Affordance*, Affordance*>> ActionPut::initOptions(const ActionScene& domain,
                                               const RcsGraph* graph,
                                               const AffordanceEntity* object,
                                               const std::string& surfaceToPutOn) const
{
  std::vector<std::tuple<Affordance*, Affordance*>> aMap;

  // Detect surface
  const AffordanceEntity* surface = domain.getAffordanceEntity(surfaceToPutOn);

  // If a non-empty name for the surface has been given, we enforce that it exists
  if ((!surface) && (!surfaceToPutOn.empty()))
  {
    throw ActionException(ActionException::UnrecoverableError,
                          "The surface '" + surfaceToPutOn + "' to put the '" + object->bdyName + "' on is unknown.",
                          "Use an object name that is defined in the environment",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // If no surface name was specified, we search through all the scene's Supportables
  if (!surface)
  {
    AffordanceEntity tmp;
    tmp.affordances = getAffordances<Supportable>(&domain);
    aMap = match<Supportable, Stackable>(&tmp, object);
    tmp.affordances.clear();

    // Since we don't have a surface entity, the Supportable can possibly be a child of
    // the Stackable, leading to put the object on a child of itself. This creates
    // trouble which we resolve in the following.
    auto it = aMap.begin();
    while (it != aMap.end())
    {
      const Affordance* supportable = std::get<0>(*it);
      const Affordance* stackable = std::get<1>(*it);
      const RcsBody* supportBdy = RcsGraph_getBodyByName(graph, supportable->frame.c_str());
      const RcsBody* stackBdy = RcsGraph_getBodyByName(graph, stackable->frame.c_str());
      bool eraseMe = RcsBody_isChild(graph, supportBdy, stackBdy);
      it = eraseMe ? aMap.erase(it) : it + 1;
    }

  }
  else
  {
    // From here on, we have a valid object and surface. We determine all
    // combinations of <Supportable, Stackable> affordances. We already know
    // that the object is stackable
    aMap = match<Supportable, Stackable>(surface, object);
  }

  // If there are none, we give up.
  if (aMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + " because it does not support it.",
                          "Specify another object to put it on",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  RLOG_CPP(1, "Affordance map has " << aMap.size() << " entries");

  // Erase the Supportables that are out of reach. We check for the graspingHand, since
  // derived classes (like magic_put or so) don't have a grasping hand.
  const Manipulator* graspingHand = usedManipulators.empty() ? nullptr : domain.getManipulator(usedManipulators[0]);
  if (graspingHand)
  {
    auto it = aMap.begin();
    while (it != aMap.end())
    {
      const Supportable* place = dynamic_cast<const Supportable*>(std::get<0>(*it));
      if (place)
      {
        bool canReach = graspingHand->canReachTo(&domain, graph, place->getFrame(graph)->A_BI.org);
        it = (!canReach) ? aMap.erase(it) : it + 1;
        if (!canReach)
        {
          RLOG_CPP(4, "Erasing " << place->frame << " - out of reach");
        }
      }
      else
      {
        it++;
      }
    }
    RLOG_CPP(1, "Done erase Supportables out of reach. Remaining: " << aMap.size());
  }

  // Erase the Supportables that don't match the "whereOn" name if it has been specified
  if (!whereOn.empty())
  {
    auto it = aMap.begin();
    while (it != aMap.end())
    {
      const Supportable* s = dynamic_cast<const Supportable*>(std::get<0>(*it));
      const bool eraseMe = !(s && (s->frame == whereOn));
      NLOG(0, "%s Supportable %s", eraseMe ? "Erasing" : "Keeping", s->frame.c_str());
      it = eraseMe ? aMap.erase(it) : it+1;
    }
  }
  RLOG_CPP(1, "Done erase Supportables on-top (frame: '" << whereOn << "'). Remaining: "
           << aMap.size());

  // Erase the Supportables that are farther away from an entity or agent than d_limit
  if (!nearTo.empty())
  {
    bool isAgent = domain.getAgent(nearTo);
    double d_limit = ((!isAgent) && (distance == 0.0)) ? IS_NEAR_THRESHOLD : distance;
    eraseSupportablesFarther(domain, graph, nearTo, d_limit, aMap);
  }
  RLOG_CPP(1, "Done erase Supportables farther than. Remaining: " << aMap.size());

  // Erase the Supportables that are more close to an entity or agent than d_limit
  if (!farFrom.empty())
  {
    bool isAgent = domain.getAgent(farFrom);
    double d_limit = ((!isAgent) && (distance == 0.0)) ? IS_NEAR_THRESHOLD : distance;
    eraseSupportablesNearer(domain, graph, farFrom, d_limit, aMap);
  }
  RLOG_CPP(1, "Done erase Supportables nearer than. Remaining: " << aMap.size());

  // Erase the Supportables that are already occupied wit something. We only do it if the
  // object to put is collideable.
  if (object->isCollideable(graph))
  {
    auto it = aMap.begin();
    while (it != aMap.end())
    {
      const Supportable* s = dynamic_cast<const Supportable*>(std::get<0>(*it));

      if (s)
      {
        auto childrenOfAff = domain.getDirectChildren(graph, s);
        const bool eraseMe = !childrenOfAff.empty();
        RLOG(1, "Occupancy check: %s Supportable %s",
             eraseMe ? "Erasing" : "Keeping", s->frame.c_str());
        it = eraseMe ? aMap.erase(it) : it + 1;
      }
      else
      {
        it++;
      }
    }
  }


  // If the map is empty after removing all non-frame Supportables, we give up.
  if (aMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + "'s frame " + whereOn + ".",
                          "", "The affordance map is empty after removing all non-frame supportables");
  }

  // Erase the Supportables that are already occupied with a collideable
  {
    auto it = aMap.begin();
    while (it != aMap.end())
    {
      // If element is even number then delete it
      const Affordance* supportable = std::get<0>(*it);
      const Supportable* s = dynamic_cast<const Supportable*>(supportable);
      RCHECK(s);

      // \todo: Make better geometric check here
      if (s->extentsX>0.0 || s->extentsY>0.0)
      {
        it++;
        continue;
      }

      // Traverse children of support frame and look for collideable entities
      const RcsBody* supportFrame = supportable->getFrame(graph);
      RcsBody* child = RCSBODY_BY_ID(graph, supportFrame->firstChildId);
      bool foundCollideable = false;

      while (child)
      {
        const AffordanceEntity* childNTT = domain.getAffordanceEntity(child->name);

        if (childNTT && childNTT->isCollideable(graph))
        {
          foundCollideable = true;
          break;
        }

        child = RCSBODY_BY_ID(graph, child->nextId);
      }

      it = foundCollideable ? aMap.erase(it) : it + 1;
    }
  }
  RLOG_CPP(1, "Done erase Supportables that are already occupied with a collideable. Remaining: " << aMap.size());

  // There's already something on all supportables
  if (aMap.empty())
  {
    std::string surfaceName = surface ? surface->name : "surface";
    throw ActionException(ActionException::ParamNotFound,
                          "I can't put the " + object->bdyName + " on the " + surfaceName + ". There is already something on it.",
                          "Put the object somewhere else, or remove the blocking object.",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  // Create a vector of tuples with the third argument being the cost to be sorted for
  RLOG(5, "Creating sortMap");
  std::vector<std::tuple<Affordance*, Affordance*,double>> sortMap;
  for (auto& pair : aMap)
  {
    Affordance* place = std::get<0>(pair);
    sortMap.push_back(std::make_tuple(place, std::get<1>(pair),
                                      actionCost(domain, graph, place->frame)));
  }

  // Sort with lambda compare function, lower cost at the beginning
  RLOG(5, "Sorting sortMap");
  std::sort(sortMap.begin(), sortMap.end(),
            [](std::tuple<Affordance*, Affordance*, double>& a,
               std::tuple<Affordance*, Affordance*, double>& b)
  {
    return std::get<2>(a) < std::get<2>(b);
  });

  // Copy back to affordance map
  RLOG(5, "Recomputing aMap");
  aMap.clear();
  for (auto& pair : sortMap)
  {
    RLOG_CPP(1, "Sorted: " << std::get<0>(pair)->frame << " - "
             << std::get<1>(pair)->frame << " : " << std::get<2>(pair));
    aMap.push_back(std::make_tuple(std::get<0>(pair), std::get<1>(pair)));
  }
  RLOG_CPP(1, "Done initOptions() with heuristic re-sorting affordance map with "
           << aMap.size() << " entries");

  return aMap;
}

ActionPut::~ActionPut()
{
}

bool ActionPut::initialize(const ActionScene& domain,
                           const RcsGraph* graph,
                           size_t solutionRank)
{
  if (solutionRank >= affordanceMap.size())
  {
    return false;
  }

  // These are the "winning" affordances.
  Affordance* surfaceAff = std::get<0>(affordanceMap[solutionRank]);
  RCHECK_MSG(surfaceAff, "Affordance at solution index %zu is NULL", solutionRank);
  Supportable* supportable = dynamic_cast<Supportable*>(surfaceAff);
  RCHECK_MSG(supportable, "%s is not a Supportable", surfaceAff->frame.c_str());   // never happens
  surfaceFrameName = surfaceAff->frame;
  supportRegionX = supportable->extentsX;
  supportRegionY = supportable->extentsY;

  Affordance* bottomAff = std::get<1>(affordanceMap[solutionRank]);
  RCHECK_MSG(bottomAff, "affordanceMap.size()=%zu, solutionRank=%zu",
             affordanceMap.size(), solutionRank);   // never happens
  Stackable* stackable = dynamic_cast<Stackable*>(bottomAff);
  RCHECK_MSG(stackable, "'%s' has no Stackable affordance",
             bottomAff->frame.c_str());   // never happens
  this->polarAxisIdx = stackable->normalDir;
  this->objBottomName = bottomAff->frame;

  // This is the put position (world coords).
  const RcsBody* surfaceBdy = resolveBodyName(graph, surfaceFrameName);
  RCHECK_MSG(surfaceBdy, "%s", surfaceFrameName.c_str());
  Vec3d_copy(endPoint, surfaceBdy->A_BI.org);
  endPoint[2] += heightAboveGoal;

  // Object bottom initial position (world coords)
  const RcsBody* objBottomBdy = RcsGraph_getBodyByName(graph, this->objBottomName.c_str());
  RCHECK(objBottomBdy);
  Vec3d_copy(startPoint, objBottomBdy->A_BI.org);

  // Point in the middle of the movement (world coords)
  Vec3d_addAndConstMul(midPoint, startPoint, endPoint, 0.5);
  midPoint[2] = 0.05 + std::max(startPoint[2], endPoint[2]);

  // Transform into surface frame
  Vec3d_invTransformSelf(endPoint, &surfaceBdy->A_BI);
  Vec3d_invTransformSelf(startPoint, &surfaceBdy->A_BI);
  Vec3d_invTransformSelf(midPoint, &surfaceBdy->A_BI);


  // Assemble finger task name. usedManipulators may be empty for derived classes
  if (!usedManipulators.empty())
  {
    const Manipulator* graspingHand = domain.getManipulator(usedManipulators[0]);

    fingerJoints.clear();
    for (auto& f : graspingHand->fingerJoints)
    {
      fingerJoints += f;
      fingerJoints += " ";
    }

    handOpen = std::vector<double>(graspingHand->getNumFingers(), fingersOpen);
  }

  // Task naming
  this->taskObjHandPos = objGraspFrame + "-" + graspFrame + "-XYZ";
  this->taskHandSurfacePos = graspFrame + "-" + surfaceFrameName + "-XYZ";
  this->taskObjSurfacePosX = objBottomName + "-" + surfaceFrameName + "-X";
  this->taskObjSurfacePosY = objBottomName + "-" + surfaceFrameName + "-Y";
  this->taskObjSurfacePosZ = objBottomName + "-" + surfaceFrameName + "-Z";
  this->taskObjSurfaceOri = objBottomName + "-" + surfaceFrameName + "-ORI";
  this->taskHandInclination = graspFrame + "-Inclination";
  this->taskHandObjPolar = graspFrame + "-" + objBottomName + "-POLAR";
  this->taskSurfaceOri = surfaceFrameName + "-POLAR";
  this->taskFingers = graspFrame + "_fingers";

  //auto surfNtt = domain.getParentAffordanceEntity(graph, surfaceBdy);
  auto surfNtt = domain.getAffordanceEntity(RCSBODY_NAME_BY_ID(graph, surfaceBdy->parentId));
  detailedActionCommand = "put " + objName;

  if (surfNtt)
  {
    detailedActionCommand += " " + surfNtt->bdyName;
  }

  detailedActionCommand += " frame " + surfaceFrameName;

  if (above)
  {
    detailedActionCommand += " above";
  }

  if (heightAboveGoal != 0.0)
  {
    detailedActionCommand += " height " + std::to_string(heightAboveGoal);
  }

  if (putDown)
  {
    detailedActionCommand += " putDown";
  }

  if (!putPolar)
  {
    detailedActionCommand += " putAligned";
  }

  if (putOri3d[2]!=0.0)
  {
    detailedActionCommand += " alignAngleZ " + std::to_string(RCS_RAD2DEG(putOri3d[2]));
  }

  // We add the duration in the getActionCommand() function, since initialize() is not
  // called after predict(), and we might adapt the duration after that.
  return true;
}

std::vector<std::string> ActionPut::createTasksXML() const
{
  std::vector<std::string> tasks;

  bool useTaskRegion = false;
  if (((supportRegionX != 0.0) || (supportRegionY != 0.0)))
  {
    useTaskRegion = true;
  }

  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  std::string xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
                        "controlVariable=\"XYZ\" " + "effector=\"" +
                        objGraspFrame + "\" " + "refBdy=\"" + graspFrame + "\" />";
  tasks.push_back(xmlTask);

  // taskHandSurfacePos: XYZ-task with effector=hand, refBdy=object and refFrame=surface
  // Used for vertical retract after ballgrasp
  xmlTask = "<Task name=\"" + taskHandSurfacePos + "\" " +
            "controlVariable=\"XYZ\" " + "effector=\"" +
            graspFrame + "\" " + "refBdy=\"" + objGraspFrame + "\" refFrame=\"" +
            surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfacePosition z and sideways velocities
  xmlTask = "<Task name=\"" + taskObjSurfacePosX + "\" " +
            "controlVariable=\"X\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" >";
  if (useTaskRegion)
  {
    xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
    xmlTask += "min=\"" + std::to_string(-0.5*supportRegionX) + "\" ";
    xmlTask += "max=\"" + std::to_string(0.5*supportRegionX) + "\" ";
    xmlTask += "dxScaling=\"0\" />";
  }
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosY+ "\" " +
            "controlVariable=\"Y\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" >";
  if (useTaskRegion)
  {
    xmlTask += "\n<TaskRegion type=\"BoxInterval\" ";
    xmlTask += "min=\"" + std::to_string(-0.5*supportRegionY) + "\" ";
    xmlTask += "max=\"" + std::to_string(0.5*supportRegionY) + "\" ";
    xmlTask += "dxScaling=\"0\" />";
  }
  xmlTask += "</Task>";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosZ + "\" " +
            "controlVariable=\"Z\" " + "effector=\"" + objBottomName + "\" " +
            "refBdy=\"" + surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfaceOri: Relative Polar angle orientation between
  // object and surface   polarAxisIdx
  std::string taskDir = putPolar ? "POLAR" : "ABC";
  xmlTask = "<Task name=\"" + taskObjSurfaceOri + "\" " +
            "controlVariable=\"" + taskDir + "\" " + "effector=\"" +
            objBottomName + "\" refBdy=\"" + surfaceFrameName + "\" ";

  if (putPolar)
  {
    if (polarAxisIdx==0)
    {
      xmlTask += "axisDirection=\"X\" ";
    }
    else if (polarAxisIdx == 1)
    {
      xmlTask += "axisDirection=\"Y\" ";
    }
  }

  xmlTask += " />";


  tasks.push_back(xmlTask);

  // taskHandInclination
  xmlTask = "<Task name=\"" + taskHandInclination + "\" " +
            "controlVariable=\"Inclination\" " + "effector=\"" + graspFrame + "\" axisDirection=\"X\" />";
  tasks.push_back(xmlTask);

  // taskHandObjPolar
  if (isPincerGrasped)
  {
    xmlTask = "<Task name=\"" + taskHandObjPolar + "\" " +
              "controlVariable=\"Inclination\" " + "effector=\"" + graspFrame + "\" " +
              "refBdy=\"" + objBottomName + "\" axisDirection=\"X\" />";
  }
  else
  {
    xmlTask = "<Task name=\"" + taskHandObjPolar + "\" " +
              "controlVariable=\"POLAR\" " + "effector=\"" + graspFrame + "\" " +
              "refBdy=\"" + objBottomName + "\" />";
  }
  tasks.push_back(xmlTask);

  // Orientation of surface frame (if held in hand)
  xmlTask = "<Task name=\"" + taskSurfaceOri + "\" " +
            "controlVariable=\"POLAR\" " + "effector=\"" + surfaceFrameName + "\" />";
  tasks.push_back(xmlTask);


  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

tropic::TCS_sptr ActionPut::createTrajectory(double t_start, double t_end) const
{
  const double t_put = t_start + 0.66 * (t_end - t_start);

  auto a1 = std::make_shared<tropic::ActivationSet>();
  a1->add(createTrajectory(t_start, t_put, t_end));

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionPut::createTrajectory(double t_start,
                            double t_put,
                            double t_release) const
{
  const double afterTime = 0.5;

  // Grasp the bottle and lift it up
  auto a1 = std::make_shared<tropic::ActivationSet>();


  a1->addActivation(t_start, true, 0.5, taskSurfaceOri);
  a1->addActivation(t_release, false, 0.5, taskSurfaceOri);

  // Put object on surface
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_start, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_put, false, 0.5, taskObjSurfacePosZ);

  if (getOptimDim()==3)
  {
    const double t_optim = t_start + 0.33 * (t_release - t_start);
    auto oState = getOptimState();
    RLOG(1, "Adding optimization constraint [%.4f %.4f %.4f] at t=%f",
         oState[0], oState[1], oState[2], t_optim);
    a1->add(t_optim, oState[0], 0.0, 0.0, 1, taskObjSurfacePosX + " 0");
    a1->add(t_optim, oState[1], 0.0, 0.0, 1, taskObjSurfacePosY + " 0");
    a1->add(t_optim, oState[2], 0.0, 0.0, 1, taskObjSurfacePosZ + " 0");
  }

  const double* x = endPoint;

  a1->add(t_put, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
  a1->add(t_put, x[1], 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
  a1->add(t_put, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");

  // Move in a little bit of a curve above other objects
  if (!putDown)
  {
    a1->add(0.5*(t_start+t_put), midPoint[2], 0.0, 0.0, 1, taskObjSurfacePosZ + " 0");
  }

  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_put, objName, surfaceFrameName));

  // Retract hand from object
  if (isPincerGrasped)
  {
    const double releaseUp = 0.15;
    a1->addActivation(t_put, true, 0.5, taskHandSurfacePos);
    a1->addActivation(t_release + afterTime, false, 0.5, taskHandSurfacePos);
    a1->add(t_release, releaseUp, 0.0, 0.0, 7, taskHandSurfacePos + " 2");
    a1->addActivation(t_put, true, 0.5, taskHandInclination);
    a1->addActivation(t_put + 0.5*(t_release-t_put), false, 0.5, taskHandInclination);
  }
  else
  {
    const double releaseDistance = 0.15;
    const double releaseUp = -0.05;
    a1->addActivation(t_put, true, 0.5, taskObjHandPos);
    a1->addActivation(t_release + 0*afterTime, false, 0.5, taskObjHandPos);
    a1->add(t_release, releaseDistance, 0.0, 0.0, 7, taskObjHandPos + " 0");
    a1->add(t_release, releaseUp, 0.0, 0.0, 7, taskObjHandPos + " 2");
  }

  // Object orientation wrt world frame. The object is re-connected to the
  // table at t=t_put. Therefore we must switch of the hand-object relative
  // orientation to avoid conflicting constraints. If we want the hand to
  // remain upright a bit longer, we would need to activate an orientation
  // task that is absolute with respect to the hand, for instance taskHandObjPolar.
  if (!isPincerGrasped)
  {
    a1->addActivation(t_start, true, 0.5, taskObjSurfaceOri);
    a1->addActivation(t_put, false, 0.5, taskObjSurfaceOri);
    if (putPolar)
    {
      a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjSurfaceOri));
    }
    else
    {
      a1->add(std::make_shared<tropic::EulerConstraint>(t_put, putOri3d, taskObjSurfaceOri));
    }
  }
  else
  {
    // Here we only keep the polar orientation for a short portion of the movement, and then
    // change towards an inclination constraint. This gives more dof and makes the approach
    // to the put place more robust.
    double t_mid = t_start + 0.25 * (t_put-t_start);
    a1->addActivation(t_start, true, 0.5, taskObjSurfaceOri);
    a1->addActivation(t_mid, false, 0.5, taskObjSurfaceOri);
    a1->addActivation(t_mid, true, 0.5, taskHandInclination);
    a1->addActivation(t_put, false, 0.5, taskHandInclination);
    if (putPolar)
    {
      a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjSurfaceOri));
    }
    else
    {
      a1->add(std::make_shared<tropic::EulerConstraint>(t_put, putOri3d, taskObjSurfaceOri));
    }

  }



  if (!isPincerGrasped)
  {
    a1->addActivation(t_put, true, 0.5, taskHandObjPolar);
    a1->addActivation(t_put + 0.5 * (t_release - t_put), false, 0.5, taskHandObjPolar);
  }

  // Open fingers. The fingers are not affected by any null space gradient,
  // therefore ther angles don't change without activation. We use this
  // to activate them only for the opening phase.
  if (!handOpen.empty())
  {
    a1->addActivation(t_put-0.5 * t_fingerMove, true, 0.5, taskFingers);
    a1->addActivation(t_release, false, 0.5, taskFingers);
    a1->add(std::make_shared<tropic::VectorConstraint>(t_put+0.5 * t_fingerMove, handOpen, taskFingers));
  }

  // Deactivate object collisions when released, and re-activate once the hand has been retracted.
  if (isObjCollidable)
  {
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_put, objName, false));
    a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_release, objName, true));
  }

  return a1;
}

std::vector<std::string> ActionPut::getManipulators() const
{
  return usedManipulators;
}

std::vector<double> ActionPut::getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                                 double duration) const
{
  const double t_optim = 0.33 *duration;
  double pos[3];
  tc->getTrajectory(taskObjSurfacePosX)->getPosition(t_optim, pos+0);
  tc->getTrajectory(taskObjSurfacePosY)->getPosition(t_optim, pos+1);
  tc->getTrajectory(taskObjSurfacePosZ)->getPosition(t_optim, pos+2);
  std::vector<double> o(pos, pos+3);
  return o;
}

void ActionPut::print() const
{
  // Objects and frames
  std::cout << "graspFrame: " << graspFrame << std::endl;
  std::cout << "objName: " << objName << std::endl;
  std::cout << "objBottomName: " << objBottomName << std::endl;
  std::cout << "objGraspFrame: " << objGraspFrame << std::endl;
  std::cout << "surfaceFrameName: " << surfaceFrameName << std::endl;
  std::cout << "fingerJoints: " << fingerJoints << std::endl;

  // Tasks
  std::cout << "taskObjHandPos: " << taskObjHandPos << std::endl;
  std::cout << "taskHandSurfacePos: " << taskHandSurfacePos << std::endl;
  std::cout << "taskObjSurfacePosX: " << taskObjSurfacePosX << std::endl;
  std::cout << "taskObjSurfacePosY: " << taskObjSurfacePosY << std::endl;
  std::cout << "taskObjSurfacePosZ: " << taskObjSurfacePosZ << std::endl;
  std::cout << "taskObjSurfaceOri: " << taskObjSurfaceOri << std::endl;
  std::cout << "taskHandObjPolar: " << taskHandObjPolar << std::endl;
  std::cout << "taskHandInclination: " << taskHandInclination << std::endl;
  std::cout << "taskSurfaceOri: " << taskSurfaceOri << std::endl;
  std::cout << "taskFingers: " << taskFingers << std::endl;

  // Spatial references
  std::cout << "whereOn: " << whereOn << std::endl;
  std::cout << "nearTo: " << nearTo << std::endl;
  std::cout << "farFrom: " << farFrom << std::endl;
  std::cout << "putOri3d: " << putOri3d[0] << putOri3d[1] << putOri3d[2] << std::endl;
  std::cout << "putPolar: " << putPolar << std::endl;

  ActionBase::print();

  for (size_t i = 0; i < affordanceMap.size(); ++i)
  {
    RLOG(0, "Solution %zu: %s - %s", i,
         std::get<0>(affordanceMap[i])->frame.c_str(),
         std::get<1>(affordanceMap[i])->frame.c_str());
  }

}

size_t ActionPut::getNumSolutions() const
{
  return affordanceMap.size();
}

std::unique_ptr<ActionBase> ActionPut::clone() const
{
  return std::make_unique<ActionPut>(*this);
}

double ActionPut::actionCost(const ActionScene& domain,
                             const RcsGraph* graph) const
{
  return actionCost(domain, graph, surfaceFrameName);
}

double ActionPut::actionCost(const ActionScene& domain,
                             const RcsGraph* graph,
                             const std::string& place) const
{
  double cost = 0.0;

  if (!nearTo.empty())
  {
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, place.c_str());
    RCHECK_MSG(putLocation, "'%s' unknown", place.c_str());
    auto ntts = domain.getSceneEntities(nearTo);
    //RLOG(0, "Found %zu SceneEntities", ntts.size());
    double sumD = 0.0;
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      sumD += Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org);
      //RLOG(0, "Adding action cost for %s - %s: %f", nttLocation->name, place.c_str(), sumD);
    }
    cost += sumD / (1.0 + sumD);
  }

  if (!farFrom.empty())
  {
    const RcsBody* putLocation = RcsGraph_getBodyByName(graph, place.c_str());
    RCHECK_MSG(putLocation, "'%s' unknown", place.c_str());
    auto ntts = domain.getSceneEntities(farFrom);
    double sumD = 0.0;
    for (const auto& ntt : ntts)
    {
      const RcsBody* nttLocation = ntt->body(graph);
      sumD += Vec3d_distance(putLocation->A_BI.org, nttLocation->A_BI.org);
    }
    cost += 1.0 / (1.0 + sumD);
  }

  return cost;
}

std::string ActionPut::getActionCommand() const
{
  std::string str = detailedActionCommand;

  if (getDuration()!=getDefaultDuration())
  {
    str += " duration " + std::to_string(getDuration());
  }

  return str;
}

double ActionPut::getDefaultDuration() const
{
  return 15.0;
}











/*******************************************************************************
 *
 ******************************************************************************/
class ActionMove : public ActionPut
{
public:

  ActionMove(const ActionScene& domain,
             const RcsGraph* graph,
             std::vector<std::string> params) : ActionPut(domain, graph, params)
  {
  }

  bool initialize(const ActionScene& domain,
                  const RcsGraph* graph,
                  size_t solutionRank)
  {
    bool success = ActionPut::initialize(domain, graph, solutionRank);

    // Duration not needed to be set, this comes through the ActionPut::getActionCommand()
    // Self-assignment is unsafe, therefore we do a swap.
    auto tmp = "move" + detailedActionCommand.substr(3);
    detailedActionCommand = tmp;
    return success;
  }

  virtual std::shared_ptr<tropic::ConstraintSet> createTrajectory(double t_start, double t_end) const
  {
    const double t_put = t_start + 0.66 * (t_end - t_start);
    const double afterTime = 0.5;
    auto a1 = std::make_shared<tropic::ActivationSet>();

    a1->addActivation(t_start, true, 0.5, taskSurfaceOri);
    a1->addActivation(t_end, false, 0.5, taskSurfaceOri);

    // Put object on surface
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosY);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosY);
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosZ);
    a1->addActivation(t_put, false, 0.5, taskObjSurfacePosZ);

    const double* x = endPoint;

    a1->add(t_put, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
    a1->add(t_put, x[1], 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
    a1->add(t_put, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");

    // Move in a little bit of a curve above other objects
    if (!putDown)
    {
      a1->add(0.5*(t_start+t_put), midPoint[2], 0.0, 0.0, 1, taskObjSurfacePosZ + " 0");
    }

    // Object orientation wrt world frame. The object is re-connected to the
    // table at t=t_put. Therefore we must switch of the hand-object relative
    // orientation to avoid conflicting constraints. If we want the hand to
    // remain upright a bit longer, we would need to activate an orientation
    // task that is absolute with respect to the hand, for instance taskHandObjPolar.
    a1->addActivation(t_start, true, 0.5, taskObjSurfaceOri);
    a1->addActivation(t_put, false, 0.5, taskObjSurfaceOri);
    if (putPolar)
    {
      a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjSurfaceOri));
    }
    else
    {
      a1->add(std::make_shared<tropic::EulerConstraint>(t_put, putOri3d, taskObjSurfaceOri));
    }

    if (!isPincerGrasped)
    {
      a1->addActivation(t_put, true, 0.5, taskHandObjPolar);
      a1->addActivation(t_put + 0.5 * (t_end - t_put), false, 0.5, taskHandObjPolar);
    }

    return a1;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionMove>(*this);
  }

};

REGISTER_ACTION(ActionMove, "move");










/*******************************************************************************
 *
 ******************************************************************************/
class ActionRelease : public ActionPut
{
public:

  ActionRelease(const ActionScene& domain,
                const RcsGraph* graph,
                std::vector<std::string> params) : ActionPut()
  {
    parseArgs(domain, graph, params);

    std::string objectToPut = params[0];

    if (params.size()>1)
    {
      surfaceFrameName = params[1];
    }

    const AffordanceEntity* object = initHands(domain, graph, objectToPut);

    // Initialize with the best solution.
    bool successInit = initialize(domain, graph, 0);
    RCHECK(successInit);
  }

  bool initialize(const ActionScene& domain,
                  const RcsGraph* graph,
                  size_t solutionRank)
  {
    // Assemble finger task name. usedManipulators may be empty for derived classes
    if (!usedManipulators.empty())
    {
      const Manipulator* graspingHand = domain.getManipulator(usedManipulators[0]);

      fingerJoints.clear();
      for (auto& f : graspingHand->fingerJoints)
      {
        fingerJoints += f;
        fingerJoints += " ";
      }

      handOpen = std::vector<double>(graspingHand->getNumFingers(), fingersOpen);
    }

    auto ntt = domain.getAffordanceEntities(objName);
    if (ntt.empty())
    {
      RLOG_CPP(0, "Entity " << objName << " not found");
      return false;
    }

    auto bottoms = getAffordances<Stackable>(ntt[0]);
    if (bottoms.empty())
    {
      RLOG_CPP(0, "Entity " << objName << " has no Supportable affordance");
      return false;
    }

    const Affordance* bottomAff = bottoms[0];
    this->objBottomName = bottomAff->frame;

    // Auto-detect surface in case it was not explicitely given. This might fail.
    if (surfaceFrameName.empty())
    {
      RLOG(0, "Auto-detecting surface to release on");
      const RcsBody* objBdy = RcsGraph_getBodyByName(graph, ntt[0]->bdyName.c_str());
      RCHECK(objBdy);
      double xyzMin[3], xyzMax[3], castFrom[3], surfPt[3], dMin = 0.0;
      bool aabbValid = RcsGraph_computeBodyAABB(graph, objBdy->id, RCSSHAPE_COMPUTE_DISTANCE, xyzMin, xyzMax, NULL);
      if (aabbValid)
      {
        Vec3d_set(castFrom, 0.5 * (xyzMin[0] + xyzMax[0]), 0.5 * (xyzMin[1] + xyzMax[1]), xyzMin[2] - 1.0e-8);
      }
      else
      {
        RLOG(0, "Could not determine AABB - setting to body origin: %f %f %f",
             objBdy->A_BI.org[0], objBdy->A_BI.org[1], objBdy->A_BI.org[2]);
        Vec3d_copy(castFrom, objBdy->A_BI.org);   // Body origin in case no AABB can be determined.
      }

      double downDir[3];
      Vec3d_set(downDir, 0.0, 0.0, -1.0);
      const RcsBody* surfaceBdy = RcsBody_closestRigidBodyInDirection(graph, castFrom, downDir, surfPt, &dMin);

      if (surfaceBdy)
      {
        surfaceFrameName = surfaceBdy->name;
      }
      else
      {
        RLOG(0, "Failed to auto-detect support surface: Cast point: %f %f %f",
             castFrom[0], castFrom[1], castFrom[2]);
        return false;
      }
    }

    if (surfaceFrameName.empty())
    {
      RLOG(0, "Couldn't find surface to release on");
      return false;
    }

    if (surfaceFrameName==ntt[0]->bdyName)
    {
      RLOG(0, "Surface is same as object");
      return false;
    }

    // Task naming
    this->taskObjHandPos = objGraspFrame + "-" + graspFrame + "-XYZ";
    this->taskHandSurfacePos = graspFrame + "-" + surfaceFrameName + "-XYZ";
    this->taskObjSurfaceOri = objBottomName + "-" + surfaceFrameName + "-ORI";
    this->taskHandInclination = graspFrame + "-Inclination";
    this->taskHandObjPolar = graspFrame + "-" + objBottomName + "-POLAR";
    this->taskFingers = graspFrame + "_fingers";

    detailedActionCommand = "release " + objName + " " + surfaceFrameName;

    // We add the duration in the getActionCommand() function, since initialize() is not
    // called after predict(), and we might adapt the duration after that.
    return true;
  }

  virtual std::shared_ptr<tropic::ConstraintSet> createTrajectory(double t_start, double t_end) const
  {
    const double afterTime = 0.5;
    auto a1 = std::make_shared<tropic::ActivationSet>();
    a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_start, objName, surfaceFrameName));

    // Retract hand from object
    if (isPincerGrasped)
    {
      const double releaseUp = 0.15;
      a1->addActivation(t_start, true, 0.5, taskHandSurfacePos);
      a1->addActivation(t_end + afterTime, false, 0.5, taskHandSurfacePos);
      a1->add(t_end, releaseUp, 0.0, 0.0, 7, taskHandSurfacePos + " 2");
      a1->addActivation(t_start, true, 0.5, taskHandInclination);
      a1->addActivation(t_start + 0.5 * (t_end - t_start), false, 0.5, taskHandInclination);
    }
    else
    {
      const double releaseDistance = 0.15;
      const double releaseUp = -0.05;
      a1->addActivation(t_start, true, 0.5, taskObjHandPos);
      a1->addActivation(t_end + 0 * afterTime, false, 0.5, taskObjHandPos);
      a1->add(t_end, releaseDistance, 0.0, 0.0, 7, taskObjHandPos + " 0");
      a1->add(t_end, releaseUp, 0.0, 0.0, 7, taskObjHandPos + " 2");
      a1->addActivation(t_start, true, 0.5, taskHandObjPolar);
      a1->addActivation(t_start + 0.5 * (t_end - t_start), false, 0.5, taskHandObjPolar);
    }

    // Open fingers. The fingers are not affected by any null space gradient,
    // therefore ther angles don't change without activation. We use this
    // to activate them only for the opening phase.
    if (!handOpen.empty())
    {
      a1->addActivation(t_start, true, 0.5, taskFingers);
      a1->addActivation(t_end, false, 0.5, taskFingers);
      a1->add(std::make_shared<tropic::VectorConstraint>(t_start+t_fingerMove, handOpen, taskFingers));
    }

    // Deactivate object collisions when released, and re-activate once the hand has been retracted.
    if (isObjCollidable)
    {
      a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_start, objName, false));
      a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end, objName, true));
    }

    return a1;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionRelease>(*this);
  }

  size_t getNumSolutions() const
  {
    return 1;
  }

};

REGISTER_ACTION(ActionRelease, "release");












/*******************************************************************************
 *
 ******************************************************************************/
class ActionMagicPut : public ActionPut
{
public:

  ActionMagicPut(const ActionScene& domain,
                 const RcsGraph* graph,
                 std::vector<std::string> params) : ActionPut()
  {
    parseArgs(domain, graph, params);

    objName = params[0];
    std::string surfaceToPutOn = params.size() > 1 ? params[1] : std::string();
    const AffordanceEntity* object = domain.getAffordanceEntity(objName);
    this->affordanceMap = initOptions(domain, graph, object, surfaceToPutOn);

    // Initialize with the best solution.
    bool successInit = initialize(domain, graph, 0);
    RCHECK(successInit);
  }

  virtual std::vector<std::string> createTasksXML() const
  {
    std::vector<std::string> tasks;

    // Dummy task with no meaning, just needed for the activation points
    std::string xmlTask = "<Task name=\"dummy\" controlVariable=\"XYZ\" effector=\"" +
                          surfaceFrameName + "\" />";
    tasks.push_back(xmlTask);

    return tasks;
  }

  virtual std::shared_ptr<tropic::ConstraintSet> createTrajectory(double t_start, double t_end) const
  {
    auto a1 = std::make_shared<tropic::ActivationSet>();

    auto cbc = std::make_shared<tropic::ConnectBodyConstraint>(t_start, objName, surfaceFrameName);
    cbc->setConnectTransform(HTr_identity());
    a1->add(cbc);

    a1->addActivation(t_start, true, 0.5, "dummy");
    a1->addActivation(t_end, false, 0.5, "dummy");

    return a1;
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionMagicPut>(*this);
  }

  std::string getActionCommand() const
  {
    return "magic_" + ActionPut::getActionCommand();
  }

  double getDefaultDuration() const
  {
    return 0.1;
  }

  double getDuration() const
  {
    return getDefaultDuration();
  }

  bool turboMode() const
  {
    return false;
  }

};

REGISTER_ACTION(ActionMagicPut, "magic_put");










/*******************************************************************************
 *
 ******************************************************************************/
class ActionWipe : public ActionPut
{
public:

  ActionWipe(const ActionScene& scene,
             const RcsGraph* graph,
             std::vector<std::string> params) : ActionPut(scene, graph, params)
  {
  }

  virtual ~ActionWipe()
  {
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionWipe>(*this);
  }

  std::string getActionCommand() const
  {
    auto words = Rcs::String_split(ActionPut::getActionCommand(), " ");
    RCHECK(!words.empty());
    words[0] = "wipe";

    std::string res;
    for (const auto& w : words)
    {
      res += w + " ";
    }

    return res;
  }

  double getDefaultDuration() const
  {
    return 10.0;
  }

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const
  {
    const double afterTime = 0.5;
    const double t_put = t_start + 0.33 * (t_end - t_start);
    RLOG_CPP(0, "t_start: " << t_start << " t_put: " << t_put << " t_end: " << t_end);

    auto a1 = std::make_shared<tropic::ActivationSet>();

    a1->addActivation(t_start, true, 0.5, taskSurfaceOri);
    a1->addActivation(t_end, false, 0.5, taskSurfaceOri);

    // Put object on surface
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePosX);
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosY);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePosY);
    a1->addActivation(t_start, true, 0.5, taskObjSurfacePosZ);
    a1->addActivation(t_end, false, 0.5, taskObjSurfacePosZ);

    const double* x = endPoint;

    a1->add(t_put, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
    a1->add(t_put, x[1], 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
    a1->add(t_put, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");

    // Wiping moves
    // t_put t_back1 t_forth1 t_back1 t_forth1 t_back1 t_forth1 t_end
    std::vector<double> t_wipe;
    const int n_wipes = 3;
    const double dt_wipe = (t_end-t_put) / (double)(2*n_wipes+1);
    const double wipeAmplitues = 0.05;
    for (size_t i = 0; i < 2 * n_wipes + 1; ++i)
    {
      double wipeSgn = i % 2 == 0 ? 1.0 : -1.0;
      const double t_wipe = t_put + (i+1) * dt_wipe;
      //a1->add(t_put + i * t_dt_wipe, x[0], 0.0, 0.0, 7, taskObjSurfacePosX + " 0");
      a1->add(t_wipe, x[1]+ wipeAmplitues * wipeSgn, 0.0, 0.0, 7, taskObjSurfacePosY + " 0");
      RLOG_CPP(0, "t_wipe[" << i << "]: " << t_wipe);
    }


    // Lift it a little bit up after wiping
    a1->add(t_end-dt_wipe, x[2], 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");
    a1->add(t_end, x[2] + 0.15, 0.0, 0.0, 7, taskObjSurfacePosZ + " 0");

    // Object orientation wrt world frame. The object is re-connected to the
    // table at t=t_put. Therefore we must switch of the hand-object relative
    // orientation to avoid conflicting constraints. If we want the hand to
    // remain upright a bit longer, we would need to activate an orientation
    // task that is absolute with respect to the hand, for instance taskHandObjPolar.
    a1->addActivation(t_start, true, 0.5, taskObjSurfaceOri);
    a1->addActivation(t_end - dt_wipe, false, 0.5, taskObjSurfaceOri);
    if (putPolar)
    {
      a1->add(std::make_shared<tropic::PolarConstraint>(t_put, 0.0, 0.0, taskObjSurfaceOri));
    }
    else
    {
      a1->add(std::make_shared<tropic::EulerConstraint>(t_put, putOri3d, taskObjSurfaceOri));
    }

    //if (!isPincerGrasped)
    {
      a1->addActivation(t_put, true, 0.5, taskHandObjPolar);
      a1->addActivation(t_put + 0.5 * (t_end - t_put), false, 0.5, taskHandObjPolar);
    }

    return a1;
  }

};

REGISTER_ACTION(ActionWipe, "wipe");










/*******************************************************************************
 *
 ******************************************************************************/
class ActionPutAligned : public ActionPut
{
public:

  ActionPutAligned(const ActionScene& scene,
                   const RcsGraph* graph,
                   std::vector<std::string> params) : ActionPut(scene, graph, params)
  {
    putPolar = false;
  }

  virtual ~ActionPutAligned()
  {
  }

  std::unique_ptr<ActionBase> clone() const
  {
    return std::make_unique<ActionPutAligned>(*this);
  }

};

REGISTER_ACTION(ActionPutAligned, "put_aligned");

}   // namespace aff
