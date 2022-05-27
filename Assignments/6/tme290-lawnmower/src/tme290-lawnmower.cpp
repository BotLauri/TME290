/*
 * Copyright (C) 2019 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include "cluon-complete.hpp"
#include "tme290-sim-grass-msg.hpp"

// Define the states.
const int chargingState = 0;
const int returningState = 1;
const int cuttingState = 2;
const int movingState = 3;

// Our parameters.
double returnBatteryPerStep{0.005};
double batteryMargin{0.04};
double minGrassToCut{0.75};
int probAreaNW{10};
int probAreaNE{30};
int probAreaSE{65};
//probAreaSW = 100 - probAreaNW - probAreaNE - probAreaSE. (Divided by 100.)

// The wall. 
uint16_t xWall = 21;
uint16_t yWall = 17;

// Initialization. 
int state = chargingState;
int command;
int targetI;
int targetJ;

int returning(int i, int j){
  if (i == 0 && j == 0){
    state = chargingState;
    return 0;
  }
  // Above wall.
  if (j < yWall) {
    if (j == 0) {
      // At top -> Move left.
      return 8;
    } else { 
      // Else move top left. 
      return 1;
    }
  }
  // Below wall. 
  else if (j > yWall) {
    if (j == 18 && i < xWall) {
      // Just below wall -> Move right.
      return 4;
    } else {
      // Else move up right.
      return 3;
    }
  } else { 
    // Beside wall. 
    return 1;
  }
}

void newTarget(){
  // Choose a area between four possible to start cutting. 
  int r = rand() % 100; // Random number between 0 and 99.
  if (r < probAreaNW){
    targetI = rand() % 5 + 1; // Random number between 1 and 5.
    targetJ = rand() % 5 + 10; // Random number between 10 and 14.
  }
  else if (r < probAreaNE){
    targetI = rand() % 10 + 30; // Random number between 30 and 39.
    targetJ = rand() % 10 + 1; // Random number between 1 and 10.
  }
  else if (r < probAreaSE){
    targetI = rand() % 15 + 25; // Random number between 25 and 39.
    targetJ = rand() % 15 + 25; // Random number between 25 and 39.
  } else {
    targetI = rand() % 15 + 1; // Random number between 1 and 15.
    targetJ = rand() % 15 + 25; // Random number between 25 and 39.
  }
}

int cutting(float g0, float g1, float g2, float g3, float g4, float g5, float g6,
            float g7, float g8, float battery, int i, int j){
  // Use i and j to predict how much battery is needed. 
  double distanceToChargingStation;
  if (targetJ < yWall) {
    distanceToChargingStation = sqrt(pow(i,2) + pow(j,2));
  } else {
    distanceToChargingStation = sqrt(pow(i,2) + pow(j,2)) + abs(i - xWall);
  }
  if (battery < (distanceToChargingStation*returnBatteryPerStep + batteryMargin)) {
    state = returningState;
    return 0;
  }
  // If battery fine, look if there is a lot of grass at nearby positions. 
  else if (g0 > minGrassToCut) {
    return 0;
  }
  else if (g1 > minGrassToCut) {
    return 1;
  }
  else if (g2 > minGrassToCut) {
    return 2;
  }
  else if (g3 > minGrassToCut) {
    return 3;
  }
  else if (g4 > minGrassToCut) {
    return 4;
  }
  else if (g5 > minGrassToCut) {
    return 5;
  }
  else if (g6 > minGrassToCut) {
    return 6;
  }
  else if (g7 > minGrassToCut) {
    return 7;
  }
  else if (g8 > minGrassToCut) {
    return 8;
  } else {
    // Move randomly. 
    int r = rand() % 8 + 1; // Random number between 1 and 8.
    return r;
  }
}

int moveTowardTarget(int i, int j){
  // Check if at target.
  if (i == targetI && j == targetJ){
    state = cuttingState;
    return 0;
  }
  // Target above wall or we are below the wall.
  else if (targetJ < yWall || j > yWall) {
    if (i < targetI) {
      if (j < targetJ) {
        return 5;
      } else {
        return 3;
      }
    } else if (i == targetI) {
      if (j < targetJ) {
        return 6;
      } else {
        return 2;
      }
    } else {
      if (j < targetJ) {
        return 7;
      } else {
        return 1;
      }
    }
  } else {
    // Target below wall, focus on the wall first. 
    if (j == 16 && i < xWall) {
      return 4;
    } else {
      return 5;
    }
  }
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] 
      << " is a lawn mower control algorithm." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session>" 
      << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --verbose" << std::endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    
    cluon::OD4Session od4{cid};

    auto onSensors{[&od4](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Sensors>(
            std::move(envelope));
        
        tme290::grass::Control control;

        switch (state)
        {
        case chargingState:
          std::cout << "Charging." << std::endl;
          // If charging make sure battery is full. 
          if (msg.i() == 0 && msg.j() == 0 && msg.battery() < 1.0) {
            command = 0;
          } else {
            newTarget();
            state = movingState;
          }
          break;
        
        case returningState:
          std::cout << "Returning." << std::endl;
          command = returning(msg.i(), msg.j());
          break;

        case cuttingState:
          std::cout << "Cutting." << std::endl;
          command = cutting(msg.grassCentre(), msg.grassTopLeft(), msg.grassTopCentre(), 
                            msg.grassTopRight(), msg.grassRight(), msg.grassBottomRight(), 
                            msg.grassBottomCentre(), msg.grassBottomLeft(), msg.grassLeft(), 
                            msg.battery(), msg.i(), msg.j());
          break;
        
        case movingState:
          std::cout << "Moving." << std::endl;
          command = moveTowardTarget(msg.i(), msg.j());
          break;

        default:
          break;
        }

        control.command(command);
        od4.send(control);
      }};

    auto onStatus{[&verbose](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Status>(
            std::move(envelope));
        if (verbose) {
          std::cout << "Status at time " << msg.time() << ": " 
            << msg.grassMean() << "/" << msg.grassMax() << std::endl;
        }
      }};

    od4.dataTrigger(tme290::grass::Sensors::ID(), onSensors);
    od4.dataTrigger(tme290::grass::Status::ID(), onStatus);

    if (verbose) {
      std::cout << "All systems ready, let's cut some grass!" << std::endl;
    }

    tme290::grass::Control control;
    control.command(0);
    od4.send(control);

    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}
