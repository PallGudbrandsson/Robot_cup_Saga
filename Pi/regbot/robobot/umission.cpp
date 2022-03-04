/***************************************************************************
 *   Copyright (C) 2016 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/



#include <sys/time.h>
#include <cstdlib>

#include "umission.h"


UMission::UMission(URegbot * regbot, UCamera * camera, UJoy * joystick)
{
  cam = camera;
  bot = regbot;
  joy = joystick;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < MAX_LINES; i++)
  { // terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
    // add to line list 
    lineList[i] = lines[i];    
  }
}


UMission::~UMission()
{
  printf("Mission class finished\n");
}


void UMission::missionInit()
{
  bot->clearEvents();
  // stop any not-finished mission
  bot->send("stop\n");
  // clear old mission
  bot->send("<clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initilisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  /* idle thread */
  bot->send("<add thread=1\n");
  // Irsensor should be activated a good time before use 
  // otherwise first samples will produce "false" positive (too short/negative).
  bot->send("<add irsensor=1,vel=0:dist=0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bot->send("<add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bot->send("<add vel=0 : time=10\n");
  //
              


  bot->send("<add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bot->send("<add vel=0\n");
  //
  bot->send("start\n"); // Two consecutive starts necessary...
  // wait a bit
  usleep(100000);
  bot->send("start\n"); // Two consecutive starts necessary...

}


void UMission::missionSendAndRun(const char ** missionLines, int missionLineCnt)
{
  // Calling missionSendAndRun automatically toggles between thread 100 and 101. 
  // Modifies the currently inactive thread and then makes it active. 
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  if (threadActive == 101)
  {
    threadToMod = 100;
    startEvent = 30;
  }
  for (int i = 0; i < missionLineCnt; i++)
  { // send lines one at a time
    if (strlen((char*)missionLines[i]) > 0)
    { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i+1, missionLines[i]);
      bot->send(s); 
    }
    else
      break;
  }
  usleep(20000);
  // debug
  // printf("Sending event %d\n",startEvent);
  // debug end
  // Activate this thread and stop the other  
  snprintf(s, MSL, "<event=%d\n", startEvent);
  // wait a bit (20ms) befor sending start event
  bot->send(s);
  threadActive = threadToMod;
}


//////////////////////////////////////////////////////////

void UMission::runMission(int fromMission, int toMission)
{
  int mission = fromMission;
  int state = 0;
  finished = false;
  bool started = false;
  bool ended = false;
  bool inManualControl = false;
  int missionState = 0, missionStateOld = 0;;
  // initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  //
  printf("Mission:: Waiting for start button or manual control\n");
  // clear old event
  sleep(1);
  bot->clearEvents();
  while (not finished)
  {
    // handle joystick control
    inManualControl = joy->testJoy(inManualControl);
    // manuel RC control is in state 0
    if (inManualControl)
      state = 0;
    switch(state)
    {
      case 0: // waiting for start button or manual control
        if (inManualControl)
        { // do nothing here - mission is paused (or not started)
        }
        else if (started)
        { // resuming paused mission
          state = mission;
          printf("Mission:: resuming mission %d (until %d)\n", mission, toMission);
        }
        else if (bot->eventSet(33))
        { // starting first mission
          printf("Mission:: starting auto mission part from %d to %d\n", fromMission, toMission);
          mission = fromMission;
          state = mission;
          started = true;
          printf("Mission %d started\n", mission);
        }
        else
        { // nothing to do
          printf("Mission:: sleeping\n");
          sleep(1);
        }
        break;
      case 1: // running auto mission
        ended = mission1(missionState);
        break;
      case 2:
        ended = mission2(missionState);
        break;
      default:
        finished = true;
        break;
    }
    if (ended)
    { // mission part ended
      mission++;
      state++;
      ended = false;
      missionState = 0;
      missionStateOld = -1;
    }
    // debug print
    if (missionState != missionStateOld)
    { // debug print of mission state
      printf("Mission %d state %d\n", mission, missionState);
      missionStateOld = missionState;
    }
    // debug print end
    // release CPU a bit (10ms)
    usleep(10000);
    // are we finished
    if (bot->eventSet(0))
    { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission or finished)
    { // stop robot
      // make an event 0
      bot->send("<event=0\n");
      // stop mission loop
      finished = true;
    }
  }
}


////////////////////////////////////////////////////////////

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission1(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  //
//   float startDist = bot->dist;
  // Primary loop for robobot mission:
  // run the desired mission
  switch (state)
  {
    case 0: // first PART 
      snprintf(lines[0], MAX_LEN, "vel=0.5:dist=0.6");
      snprintf(lines[1], MAX_LEN, "tr=0.2:turn=90");
      // last line should never end, as robot then think we are finished
      // so therefore a timeout of 1 second, to allow next set of
      // commands to be delivered
      snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 3);
      state++;
      break;
    case 1:
      if (bot->eventSet(1))
      { // finished first drive
        state = 10;
//         printf("mission finished first part\n");
      }
      break;
    case 10: // go back to start position and stop
      snprintf(lines[0], MAX_LEN, ": dist=0.6");
      snprintf(lines[1], MAX_LEN, "tr=0.05:turn=90");
      snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 3);
      state++;        
      break;
    case 11:
      if (bot->eventSet(1))
      { // finished
        state = 20;
        printf("mission ended\n");
      }
      break;
    case 999:
    default:
      finished = true;
      break;
  }
  return finished;
}






