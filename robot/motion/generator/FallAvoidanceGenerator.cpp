/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <fstream>
#include <limits>
#include <cctype>
#include "motion/generator/FallAvoidanceGenerator.hpp"
#include "utils/Logger.hpp"
#include "utils/angles.hpp"
#include "utils/body.hpp"


using namespace std;
using namespace Sensors;
using boost::program_options::variables_map;

FallAvoidanceGenerator::FallAvoidanceGenerator(std::string filename) : file_name(filename) {
   max_iter = 0;
   current_time = NOT_RUNNING;
   prev_sag_ang = 0;

   crouch = true;
   stable = false;
   original_file_name = filename;
};

FallAvoidanceGenerator::~FallAvoidanceGenerator() {
   llog(INFO) << "FallAvoidanceGenerator destroyed" << std::endl;
};

bool FallAvoidanceGenerator::isActive() {
   return current_time != NOT_RUNNING;
};

void FallAvoidanceGenerator::reset() {
   current_time = 0;
}

void FallAvoidanceGenerator::stop(){
}

JointValues FallAvoidanceGenerator::makeJoints(ActionCommand::All* request,
                                        Odometry* odometry,
                                        const SensorValues &sensors,
                                        BodyModel &bodyModel,
                                        float ballX,
                                        float ballY) {
   JointValues j;

   // Get the current saggital angle and angular velocity
   float sag_ang = RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY]);
   sag_ang_vel = sag_ang - prev_sag_ang;
   prev_sag_ang = sag_ang;

   // If nao is wobbling, correct the hip joints proportional
   // to the wobble to stabilise
   if (isUnstable(sag_ang) && current_time != NOT_RUNNING) {
      // Set up joints
      JointValues newJoints = sensors.joints;
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
         newJoints.stiffnesses[i] = 1.0f;
      }
      if (current_time == 0) interpolate(newJoints);
      if (current_time == (signed int)joints.size()) {
         j = joints[current_time-1];

         // Calculate necessary hip correction
         float correction = -1*sag_ang*ANGLE_ADJUSTMENT/SCALE;

         std::cout << "CORRECTION:  " << correction << std::endl;

         // Correct hips
         //if (ABS(correction) > 0.07) {
         //if (sag_ang > 5 || sag_ang < 0) {
            j.angles[Joints::RHipPitch] += correction;
            j.angles[Joints::LHipPitch] += correction;
         //}

      } else {
         j = joints[current_time++];
      }

      if (stable)
         stable = false;
   // If nao is stabilised, do the rest of the movement to get slowly
   // back to a stand position
   } else {
      JointValues newJoints = sensors.joints;
      float stable_hip = newJoints.angles[Joints::RHipPitch];
      if (!stable) {
         stable = true;
         stable_hip = newJoints.angles[Joints::RHipPitch];
      }

      if (current_time == NOT_RUNNING) {
         // current_time = 0;
         active = request->body;
         j = joints[joints.size() - 1];
      } else {
         JointValues newJoints = sensors.joints;
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            j.stiffnesses[i] = 1.0f;
         }
         if (current_time == 0) interpolate(newJoints);
         if (current_time == (signed int)joints.size() && crouch) {
            //Then stand up
            //If we have already done the slowstand, don't do it again
            if (original_file_name != "slowStand") {
               crouch = false;
               reset();
               joints.clear();
               file_name = "slowStand";
               readOptions(configuration);
            } else {
               current_time = NOT_RUNNING;
            }
            
         } else if (current_time == (signed int)joints.size() && !crouch){
            current_time = NOT_RUNNING;
            crouch = true;
            joints.clear();
            file_name = original_file_name;
            readOptions(configuration);

         } else {
            j = joints[current_time++];
         
         }
      }

      //j.angles[Joints::RHipPitch] = stable_hip;
      //j.angles[Joints::LHipPitch] = stable_hip;
   }

   // JointValues j = joints[current_time++];
   return j;
};

bool FallAvoidanceGenerator::isUnstable(float angle) {
   // If the pose has not started yet, we are standing.
   // In this case, instability occurs at +/- 10 degrees
   if (current_time == 0 || current_time == (signed int)joints.size()) {
      return ABS(angle) > STABLE_ANG;
   } else {
      // If the pose has started, then we can lean more forwards
      // and still be stable than we can backwards
      if (angle > 0)
         return angle > POS_STABLE_ANG;
      else
         return angle < NEG_STABLE_ANG;
   }

   return false;
}

void FallAvoidanceGenerator::interpolate(JointValues newJoint, int duration) {
   if (joints.empty()) {
      max_iter = duration / 10;
      // Reserve space for the interpolation when the generator
      // first called
      for (int i = 0; i < max_iter; i++) {
         joints.push_back(newJoint);
      }
      joints.push_back(newJoint);
   } else {
      int inTime = 0;
      float offset[Joints::NUMBER_OF_JOINTS];

      if (duration != 0) {
         inTime = duration / 10;
         JointValues currentJoint = joints.back();

         // Calculate the difference between new joint and the previous joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
         }

         for (int i = 0; i < inTime; i++) {
            JointValues inJoint;
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               inJoint.angles[j] = joints.back().angles[j] + offset[j];
               inJoint.stiffnesses[j] = newJoint.stiffnesses[j];
            }
            joints.push_back(inJoint);
         }
      } else {

         JointValues firstJoint = joints.at(max_iter);
         // Calculate the difference between the joint at MAX_ITER position
         // with the new joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
         }

         joints[0] = newJoint;
         for (int i = 1; i < max_iter; i++) {
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
               joints[i].stiffnesses[j] = firstJoint.stiffnesses[j];
            }
         }
      }
   }
}

void FallAvoidanceGenerator::constructPose(std::string path) {
   ifstream in(string(path + "/" + file_name + ".pos").c_str());
   llog(INFO) << "FallAvoidanceGenerator(" << file_name << ") creating" << endl;

   if (!in.is_open()) {
      llog(FATAL) << "FallAvoidanceGenerator can not open " << file_name << endl;
   } else {
      int duration = 0;
      float stiffness = 1.0;
      float angles = 0.0;
      while (isspace(in.peek())) {
         in.ignore();
      }
      while (!in.eof()) {
         // Ignore comments, newlines and ensure not eof
         if (in.peek() == '#' || in.peek() == '\n' || in.peek() == EOF) {
            in.ignore(std::numeric_limits<int>::max(), '\n');
            continue;
         }
         JointValues newJoint;
         // Read the angles in the file to create a new JointValue
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            while (isspace(in.peek())) {
               in.ignore();
            }
            if (in.peek() == '#' || in.peek() == '$' || in.peek() == '\n' || in.peek() == EOF) {
               std::cout << "You're missing a joint value in " << file_name << ".pos" << std::endl;
               exit(1);
            }
            in >> angles;
            // Convert degree to radian because the values in the file
            // are in degree
            newJoint.angles[i] = DEG2RAD(angles);
            newJoint.stiffnesses[i] = 1.0;
         }

         // read in the duration
         while (isspace(in.peek())) {
            in.ignore();
         }
         if (in.peek() == '#' || in.peek() == '$' || in.peek() == '\n' || in.peek() == EOF) {
            std::cout << "You're missing a duration in " << file_name << ".pos" << std::endl;
            exit(1);
         }
         in >> duration;

         // Ignore comments, newlines and ensure not eof
         while (isspace(in.peek())) {
            in.ignore();
         }
         while (in.peek() == '#') {
            in.ignore(std::numeric_limits<int>::max(), '\n');
         }
         while (isspace(in.peek())) {
            in.ignore();
         }

         // Stiffnesses are specified by a line beginning with "$"
         if (in.peek() == '$') {
            in.ignore(std::numeric_limits<int>::max(), '$');
            for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
               while (isspace(in.peek())) {
                  in.ignore();
               }
               if (in.peek() == '#' || in.peek() == '\n' || in.peek() == EOF) {
                  std::cout << "You're missing a stiffness value in " << file_name << ".pos" << std::endl;
                  exit(1);
               }
               in >> stiffness;
               newJoint.stiffnesses[i] = stiffness;
            }
            // Ignore comments, newlines and ensure not eof
            while (isspace(in.peek())) {
               in.ignore();
            }
            while (in.peek() == '#') {
               in.ignore(std::numeric_limits<int>::max(), '\n');
            }
            while (isspace(in.peek())) {
               in.ignore();
            }
         }
         interpolate(newJoint, duration);
         while (isspace(in.peek())) {
            in.ignore();
         }
      }
      in.close();
   }
   llog(INFO) << "FallAvoidanceGenerator(" << file_name << ") created" << endl;
}

void FallAvoidanceGenerator::readOptions(const boost::program_options::variables_map &config) {
   configuration = config;
   std::string path = config["motion.path"].as<std::string>();
   constructPose(path);
}
