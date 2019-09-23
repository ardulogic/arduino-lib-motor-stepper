/*
   FILE:    MotorWaypoint.h
   DATE:    09/07/15
   VERSION: 0.1
   AUTHOR:  Andrew Davies

11/03/15 version 0.1: Original version  
   
Library header for driving DC and stepper motors.

You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used directly for the purpose of selling products that
directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.

 */
#ifndef MotorWaypoints_h
#define MotorWaypoints_h

//#define DEBUG_WAYPOINT_SET
//#define DEBUG_WAYPOINT_GRAPH

// This makes uint8_t work, otherwise its undefined
#include "Arduino.h"
#include "MotorWaypoint.h";

#ifdef DEBUG_WAYPOINT_GRAPH
#include "Plotter.h"
#endif

#define WAYPOINT_COUNT 6

#define WPTS_STATE_STOPPED     0
#define WPTS_STATE_IN_PROGRESS 1
#define WPTS_STATE_PAUSED      2
#define WPTS_STATE_ON_DELAY    3

class MotorWaypoints {
public:
    MotorWaypoints();
    void Set(uint8_t idx, long position, int frequency, unsigned int delay_ms, bool enabled);
    void Start();
    void Pause();
    void UnPause();
    void Stop();
    
    void Tick();
    void Step();
    void Right();
    void Left();
    void ResetPosition();
    
    int8_t GetState();
    bool InProgress();
    bool IsPaused();
    bool IsOnDelay();
    
    bool Delay();
    void SkipDelay();

    long   GetPositionDiff();
    int8_t GetNextWaypoint();   
    void   SetActiveWaypoint(int8_t idx); // Can't use uint8_t, If clause doesnt work with 0

    int8_t       NextStep();
    unsigned int NextFreq();

    #ifdef DEBUG_WAYPOINT_GRAPH
    Plotter p;
    float t_x1 = 0;
    float t_x2 = 0;   
    long t_y1 = 0;
    long t_y2 = 0;
    #endif;    

    bool in_progress = false;
    bool paused      = false;
       
    int8_t        active_wpt;
    unsigned long position;

    // Default Parameters
    unsigned int d_freq_travel;   
    
    // Current Position Parameters
    long         c_pos_diff;
    int          c_freq_diff;
    int          c_freq_adj; 
    unsigned int c_delay_left;
        
    MotorWaypoint *wpts[WAYPOINT_COUNT];
    
    // Destructor
    ~MotorWaypoints(void) {
        for (int i = 0; i < WAYPOINT_COUNT; i++) {
            delete wpts[i];
        }
    };
};

#endif