/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef AFF_GAZECOMPONENT_H
#define AFF_GAZECOMPONENT_H

#include "ComponentBase.h"
#include "ActionScene.h"

#include <Rcs_graph.h>

#include <tuple>
#include <iostream>
#include <fstream>

#define DEFAULT_MAX_GAZE_ANGLE_DIFF 50.0

namespace aff
{


struct GazeDataPoint {
    double time;  // Time in seconds
    // std::string agentName; // Name of the agent
    std::vector<std::string> objectNames;  // Names of the objects
    std::vector<double> angleDiffs;  // Angle differences to objects in deg
    std::vector<double> distances;   // Distance to each object
    std::vector<double> angleDiffsXY;  // Gaze angles in XY plane
    std::vector<double> angleDiffsXZ;  // Gaze angles in YZ plane
    double gazeVel; // Gaze velocity in deg/s

    GazeDataPoint(double t, const std::vector<std::string>& names, const std::vector<double>& angleDiffs_,
     const std::vector<double>& distances_, double vel, const std::vector<double>& angleDiffsXY_, const std::vector<double>& angleDiffsXZ_)
        : time(t), objectNames(names), angleDiffs(angleDiffs_),distances(distances_), gazeVel(vel),
          angleDiffsXY(angleDiffsXY_), angleDiffsXZ(angleDiffsXZ_) {}
};


class GazeComponent : public ComponentBase
{
public:

  /*! \brief Constructs GazeComponent.
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   */
  GazeComponent(EntityBase* parent, const std::string& agentName, const std::string& gazingBody,
                int dirIdx = 1, double maxDurationGazeData=20, bool saveData=false, double maxGazeAngleDiff=DEFAULT_MAX_GAZE_ANGLE_DIFF);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         There is no thread that needs to be stopped.
   */
  virtual ~GazeComponent();

  void addSceneToAttend(const ActionScene& scene, const RcsGraph* graph);
  
  void saveInFile(const std::string& filename);
  
  // Pointer to gazeData
  const std::deque<GazeDataPoint>* getGazeData() {
      return &gazeData;  
  }

  std::string getAgentName() const {
      return agentName;
  }

private:

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);
  const RcsBody* getBody(const RcsGraph* graph, const std::string& bdyName, int& bdyId);
  


  std::string gazingBody;
  int id_gazeBody;
  int gazeDirectionIdx;   // 0: x, 1: y, 2: z
  bool saveData;
  double prevHeadDirection[3];
  std::string agentName;

  double maxGazeAngleDiff;

  struct BodyIntersection
  {
    BodyIntersection()
    {
      bdyId = -1;
      gazeAngle = -1.0;
    }

    std::string name;
    std::string bdyName;
    int bdyId;
    double gazeAngle;
    double objectPointDistance;
    double gazeAngleXY;
    double gazeAngleXZ;
  };

  std::vector<BodyIntersection> objectsToAttend;
  std::ofstream file;  // File stream object

  
  
  void writeSortedData(const double time, const std::vector<BodyIntersection>& objectsToAttend, const double gazeVel);

  // void addGazeDataPoint(double time, const std::vector<std::string>& objectNames, const std::vector<double>& distances, double gazeVel);

  std::deque<GazeDataPoint> gazeData;
  double maxDurationGazeData;
  double totalDurationGazeData;

  void addGazeDataPoint(double time, const std::vector<std::string>& objectNames, const std::vector<double>& diffAngles, 
                        const std::vector<double>& distances, double gazeVel, const std::vector<double>& angleDiffsXY, 
                        const std::vector<double>& angleDiffsXZ);

  void removeOldestGazeDataPoint();

  void getPointsAABBSurface(const double (&xyzMin)[3], const double (&xyzMax)[3], std::vector<std::array<double,3>>& pointsObject, const double & distance);


  
};

}

#endif   // AFF_GAZECOMPONENT_H
