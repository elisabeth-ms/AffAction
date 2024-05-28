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

#include "ActionComposite.h"
#include "ActionFactory.h"
#include "ActionGet.h"
#include "ActionPut.h"
#include "ActionGaze.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_graphParser.h>
#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>

#include <limits>
#include <algorithm>


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
ActionComposite::ActionComposite()
{
}

ActionComposite::~ActionComposite()
{
}

ActionComposite::ActionComposite(const ActionComposite& other)
{
  for (const auto& action : other.actions)
  {
    actions.push_back(action->clone());
  }
}

std::unique_ptr<ActionBase> ActionComposite::clone() const
{
  return std::unique_ptr<ActionComposite>();
}

void ActionComposite::addAction(ActionBase* action)
{
  actions.push_back(std::unique_ptr<ActionBase>(action));
}


std::vector<std::string> ActionComposite::createTasksXML() const
{
  std::vector<std::string> tasks;

  for (const auto& a : actions)
  {
    std::vector<std::string> ti = a->createTasksXML();
    //tasks.insert(tasks.end(), ti.begin(), ti.end());

    // Check for duplicates
    for (const auto& tsk_i : ti)
    {
      if (std::find(tasks.begin(), tasks.end(), tsk_i) != tasks.end())
      {
        //RLOG_CPP(0, "Skipping duplicate task:\n" << tsk_i);
      }
      else
      {
        tasks.push_back(tsk_i);
      }
    }
  }

  return tasks;
}

tropic::TCS_sptr ActionComposite::createTrajectory(double t_start, double t_end) const
{
  auto cset = std::make_shared<tropic::ActivationSet>();

  for (const auto& a : actions)
  {
    cset->add(a->createTrajectory(t_start, t_end));
  }

  return cset;
}

double ActionComposite::getDuration() const
{
  double duration = 0.0;

  for (const auto& a : actions)
  {
    duration = std::max(duration, a->getDuration());
  }

  return duration;
}

double ActionComposite::getDefaultDuration() const
{
  double duration = 0.0;

  for (const auto& a : actions)
  {
    duration = std::max(duration, a->getDefaultDuration());
  }

  return duration;
}

std::vector<std::string> ActionComposite::getManipulators() const
{
  std::vector<std::string> mVec;

  for (const auto& a : actions)
  {
    std::vector<std::string> mi = a->getManipulators();
    mVec.insert(mVec.end(), mi.begin(), mi.end());
  }

  return mVec;
}

std::string ActionComposite::getActionCommand() const
{
  std::string compositeCmd;
  RMSG("CHECK DURATION DISASTER!!!");
  for (size_t i=0; i<actions.size(); ++i)
  {
    compositeCmd += actions[i]->getActionCommand();
    if (i < actions.size() - 1)
    {
      compositeCmd += "+";
    }
  }

  return compositeCmd;
}

size_t ActionComposite::getNumSolutions() const
{
  size_t nSols = 1;

  for (const auto& a : actions)
  {
    nSols *= a->getNumSolutions();
  }

  return nSols;
}


/* Test

   size_t nsols = 12;
   size_t nsols0 = 4;
   size_t nsols1 = 3;
   size_t query = 7;

   for (size_t i = 0; i < nsols; ++i)
   {
       RLOG(0, "Query %zu gets: %zu %zu", i, i / nsols1, i % nsols1);
   }

   PAUSE();

End test

get glass_red; put glass_red far bottle_of_cola; get bottle_of_cola + get glass_red; pour bottle_of_cola glass_red duration 25;put glass_red near Felix + put bottle_of_cola box_of_cereal; pose default

get glass_red; put glass_red far bottle_of_cola + get bottle_of_cola; get glass_red; pour bottle_of_cola glass_red duration 25;put glass_red near Felix + put bottle_of_cola box_of_cereal; pose default
*/

static void getTuples(size_t index, const std::vector<int>& totals,
                      std::vector<int>& current,
                      std::vector<std::vector<int>>& result)
{
  if (index == totals.size())
  {
    result.push_back(current);
    return;
  }

  for (int i = 0; i < totals[index]; ++i)
  {
    current.push_back(i);
    getTuples(index + 1, totals, current, result);
    current.pop_back();
  }
}

static void printTuple(const std::vector<int>& tuple)
{
  std::cout << "(";
  for (size_t i = 0; i < tuple.size(); ++i)
  {
    std::cout << tuple[i];
    if (i != tuple.size() - 1)
    {
      std::cout << ", ";
    }
  }
  std::cout << ")";
}

static std::vector<std::vector<int>> createTuples(std::vector<int> solutions)
{
  std::vector<int> current;
  std::vector<std::vector<int>> result;

  getTuples(0, solutions, current, result);

  return result;
}

/*
get glass_red; put glass_red far bottle_of_cola; get bottle_of_cola + get glass_red; pour bottle_of_cola glass_red duration 25;put glass_red near Felix + put bottle_of_cola box_of_cereal; pose default

get glass_red; put glass_red far bottle_of_cola + get bottle_of_cola; get glass_red; pour bottle_of_cola glass_red duration 25;put glass_red near Felix + put bottle_of_cola box_of_cereal; pose default
 */
bool ActionComposite::initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank)
{
  if (solutionRank >= getNumSolutions())
  {
    return false;
  }

  // RCHECK_MSG(actions.size()==2, "Fixme");

  // if (actions.size() == 2)
  // {
  //   size_t sols0 = actions[0]->getNumSolutions();
  //   size_t sols1 = actions[1]->getNumSolutions();
  //   actions[0]->initialize(domain, graph, solutionRank/sols1);
  //   actions[1]->initialize(domain, graph, solutionRank%sols1);
  // }

  // std::vector<size_t> denom(actions.size(), 1);
  // for (size_t i=0; i<actions.size(); ++i)
  // {
  //   denom[i] = denom[i-1] * actions[0]->getNumSolutions();
  // }

  std::vector<int> numSolutions;
  for (const auto& a : actions)
  {
    numSolutions.push_back(a->getNumSolutions());
  }

  auto tuples = createTuples(numSolutions);


  // Print all generated tuples
  REXEC(1)
  {
    RLOG(0, "Found these tuples:");
    for (const auto& t : tuples)
    {
      printTuple(t);
      std::cout << std::endl;
    }
  }

  RCHECK(tuples.size()==getNumSolutions());

  for (size_t i=0; i<actions.size(); ++i)
  {
    actions[i]->initialize(domain, graph, tuples[solutionRank][i]);
  }


  return true;
}




/*******************************************************************************
 *
 ******************************************************************************/
class ActionDoubleGet : public ActionComposite
{
public:

  ActionDoubleGet(const ActionDoubleGet& other) : ActionComposite(other)
  {
  }

  std::unique_ptr<ActionBase> clone() const override
  {
    return std::make_unique<ActionDoubleGet>(*this);
  }

  ActionDoubleGet(const ActionScene& domain,
                  const RcsGraph* graph,
                  std::vector<std::string> params)
  {
    if (params.size() != 2)
    {
      throw ActionException(ActionException::ParamInvalid,
                            "Action command received with " + std::to_string(params.size()) + " arguments, but 2 are expected");
    }

    if (domain.manipulators.size() < 2)
    {
      throw ActionException(ActionException::ParamInvalid, "Action created with " + std::to_string(domain.manipulators.size()) + " manipulators, but 2 or more are expected");
    }

    // Compute the two closest manipulators to the two objects
    // m1-o1   m1-o2
    // m2-o1   m2-o2
    // m3-o1   m3-o2
    //  ...     ...
    // mn-o1   mn-o2
    // The optimal pairing is the sum of a column1 value and a column 2 value
    // that must not be in the same row.
    const RcsBody* obj1 = RcsGraph_getBodyByNameNoCase(graph, params[0].c_str());
    const RcsBody* obj2 = RcsGraph_getBodyByNameNoCase(graph, params[1].c_str());

    if (!obj1)
    {
      throw ActionException(ActionException::ParamNotFound, "Object " + params[0] + " is unknown");
    }

    if (!obj2)
    {
      throw ActionException(ActionException::ParamNotFound, "Object " + params[1] + " is unknown");
    }

    std::vector<std::vector<double>> res;
    for (const auto& m : domain.manipulators)
    {
      const RcsBody* mBdy = RcsGraph_getBodyByNameNoCase(graph, m.bdyName.c_str());
      std::vector<double> res_i;
      res_i.push_back(Vec3d_distance(mBdy->A_BI.org, obj1->A_BI.org));
      res_i.push_back(Vec3d_distance(mBdy->A_BI.org, obj2->A_BI.org));
      res.push_back(res_i);
    }

    double dMin = std::numeric_limits<double>::max();
    std::string m1, m2;

    for (size_t c1 = 0; c1 < res.size(); ++c1)
    {
      for (size_t c2 = 0; c2 < res.size(); ++c2)
      {
        if (c1 == c2)
        {
          continue;
        }

        const double di = res[c1][0] + res[c2][1];
        if (di < dMin)
        {
          dMin = di;
          m1 = domain.manipulators[c1].name;
          m2 = domain.manipulators[c2].name;
        }
      }
    }

    try
    {
      std::vector<std::string> mParams = {params[0], m1};
      addAction(new ActionGet(domain, graph, mParams));
      mParams = {params[0], m2};
      addAction(new ActionGet(domain, graph, mParams));
    }
    catch (const ActionException& ex)
    {
      throw ActionException(ex);
    }
    catch (...)
    {
      throw std::invalid_argument("Failed to create ActionDoubleGet");
    }

  }

};

REGISTER_ACTION(ActionDoubleGet, "double_get");








/*******************************************************************************
 *
 ******************************************************************************/
class ActionDoublePut : public ActionComposite
{
public:

  ActionDoublePut(const ActionDoubleGet& other) : ActionComposite(other)
  {
  }

  std::unique_ptr<ActionBase> clone() const override
  {
    return std::make_unique<ActionDoublePut>(*this);
  }

  ActionDoublePut(const ActionScene& domain,
                  const RcsGraph* graph,
                  std::vector<std::string> params)
  {
    if ((params.size() != 2) && (params.size() != 3) && (params.size() != 4))
    {
      throw ActionException("ERROR REASON: Action received " + std::to_string(params.size()) +
                            " arguments, but 2, 3 or 4 are expected", ActionException::ParamInvalid);
    }

    try
    {
      if (params.size() == 2)
      {
        std::vector<std::string> newParams = {params[0], std::string()};
        addAction(new ActionPut(domain, graph, newParams));
        newParams = {params[1], std::string()};
        addAction(new ActionPut(domain, graph, newParams));
      }
      else if (params.size() == 3)
      {
        std::vector<std::string> newParams = {params[0], params[2]};
        addAction(new ActionPut(domain, graph, newParams));
        newParams = {params[1], params[2]};
        addAction(new ActionPut(domain, graph, newParams));
      }
      else if (params.size() == 4)
      {
        std::vector<std::string> newParams = {params[0], params[2]};
        addAction(new ActionPut(domain, graph, newParams));
        newParams = {params[1], params[3]};
        addAction(new ActionPut(domain, graph, newParams));
      }
    }
    catch (const ActionException& ex)
    {
      throw ActionException(ex);
    }
    catch (...)
    {
      throw std::invalid_argument("ERROR REASON: Failed to create sub-action.");
    }

  }

};

REGISTER_ACTION(ActionDoublePut, "double_put");








/*******************************************************************************
 *
 ******************************************************************************/
class ActionMultiString : public ActionComposite
{
public:

  ActionMultiString(const ActionDoubleGet& other) : ActionComposite(other)
  {
  }

  std::unique_ptr<ActionBase> clone() const override
  {
    return std::make_unique<ActionMultiString>(*this);
  }

  ActionMultiString(const ActionScene& domain,
                    const RcsGraph* graph,
                    std::vector<std::string> params)   // Each string is space-separated parameters
  {

    try
    {

      for (size_t i = 0; i < params.size(); ++i)
      {
        std::vector<std::string> words = Rcs::String_split(params[i], " ");
        ActionResult errMsg;
        addAction(ActionFactory::create(domain, graph, words, errMsg));
      }
    }
    catch (const ActionException& ex)
    {
      throw ActionException(ex);
    }
    catch (...)
    {
      throw std::invalid_argument("Failed to create ActionMultiString");
    }

  }

};

REGISTER_ACTION(ActionMultiString, "multi_string");








/*******************************************************************************
 *
 ******************************************************************************/
class ActionGazeAndGet : public ActionComposite
{
public:

  ActionGazeAndGet(const ActionDoubleGet& other) : ActionComposite(other)
  {
  }

  std::unique_ptr<ActionBase> clone() const override
  {
    return std::make_unique<ActionGazeAndGet>(*this);
  }

  ActionGazeAndGet(const ActionScene& domain,
                   const RcsGraph* graph,
                   std::vector<std::string> params)
  {

    try
    {
      addAction(new ActionGet(domain, graph, params));
      addAction(new ActionGaze(domain, graph, params));
      addAction(new ActionGaze(domain, graph, params));
    }
    catch (const ActionException& ex)
    {
      throw ActionException(ex);
    }
    catch (...)
    {
      throw std::invalid_argument("FATAL_ERROR REASON: Failed to create sub-action");
    }

  }

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const
  {
    const double afterTime = 0.5;
    ActionGet* actionGet = dynamic_cast<ActionGet*>(actions[0].get());
    ActionGaze* ag1 = dynamic_cast<ActionGaze*>(actions[1].get());
    ActionGaze* ag2 = dynamic_cast<ActionGaze*>(actions[2].get());
    RCHECK(actionGet && ag1 && ag2);
    const double t_get = actionGet->getDuration();
    const double t_gaze = ag1->getDuration();
    const double duration = t_end - t_start;
    const double t_mid = t_start+t_get/(t_get+t_gaze)*duration;

    auto cset = std::make_shared<tropic::ActivationSet>();

    cset->add(ag1->createTrajectory(t_start, t_mid));
    ag2->isGazeTargetInHand = true;
    ag2->keepTasksActiveAfterEnd = false;
    cset->add(ag2->createTrajectory(t_mid, t_end));

    cset->add(actionGet->createTrajectory(t_start, t_mid));

    // Object orientation with respect to world frame. We keep it upright, but
    // don't enforce the orientation around the up-axis. For something like an
    // apple (Inclination constraint only, we don't need to enforce any object
    // orientation. \todo(MG): Do this.
    if (actionGet->graspType != ActionGet::GraspType::BallGrasp)
    {
      //cset->addActivation(t_mid+afterTime, true, 0.5, actionGet->taskObjOri);
      //cset->addActivation(t_end+afterTime, false, 0.5, actionGet->taskObjOri);
    }

    return cset;
  }

  double getDuration() const
  {
    return 1.0*(actions[0]->getDuration() + actions[1]->getDuration());
  }

  size_t getNumSolutions() const
  {
    return actions[0]->getNumSolutions();
  }

  bool initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank)
  {
    return actions[0]->initialize(domain, graph, solutionRank);
  }

};

REGISTER_ACTION(ActionGazeAndGet, "gaze_and_get");





}   // namespace aff
