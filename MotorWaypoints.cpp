/* FILE:    SingleStepper.h
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

#include "MotorWaypoints.h"
#include "MotorStepper.h"

MotorWaypoints::MotorWaypoints() {
    position       = 0;
    c_delay_left   = 0;
    d_freq_travel  = 1000;
   
    for (int i = 0; i < WAYPOINT_COUNT + 1; i++) {
        wpts[i] = new MotorWaypoint(0, 0, 0, false);
    }
}

void MotorWaypoints::Set(uint8_t idx, long position, int frequency, unsigned int delay_ms, bool enabled) {
    wpts[idx]->position = position;
    wpts[idx]->frequency = frequency;
    wpts[idx]->delay_ms = delay_ms;
    wpts[idx]->enabled = enabled;
    
    #ifdef DEBUG_WAYPOINT_SET
    Serial.print("NEW WPT #");
    Serial.print(idx);   
    Serial.print(" FREQ:");       
    Serial.print(frequency);   
    Serial.print(" KEEP:");       
    Serial.print(delay_ms);   
    Serial.print(" EN:");       
    Serial.print(enabled);   
    Serial.print("\n");      
    #endif
}

void MotorWaypoints::Start() {   
    // Start plotter
    #ifdef DEBUG_WAYPOINT_GRAPH
    p.Begin();      
    p.AddXYGraph( "Position/Time", 10000, "time",  t_x1, "position", t_y1);
    p.AddXYGraph( "Frequency/Time", 10000, "time",  t_x2, "frequency", t_y2);
    #endif        

    active_wpt = 0;
    
    // We need to use GetNextWaypoint for determining
    // next waytpoint. It is not necessarily 1, since it
    // can be disabled
    Set(0, wpts[GetNextWaypoint()]->position, d_freq_travel, 0, true);
    SetActiveWaypoint(0);
    
    in_progress = true;
    paused = false;
}

void MotorWaypoints::Tick() {
    if (InProgress()) {
        #ifdef DEBUG_WAYPOINT_GRAPH
        t_x1 += 1/(float)((int)MotorStepper::Frequency());
        t_x2 = t_x1;
        t_y1 = (long) position;   
        t_y2 = (int)MotorStepper::Frequency();        
        p.Plot();       
        #endif
                    
        if (IsPaused()) return;
        if (IsOnDelay()) { MotorStepper::SetFrequency(Delay() ? 1000 : 1); return; };
       
        Step();  
    }
}

void MotorWaypoints::Right() {
    MotorStepper::SetDirection(FORWARD);
    MotorStepper::SetFrequency(500); //52us                               
    MotorStepper::Step();
}

void MotorWaypoints::Left() {
    
}

void MotorWaypoints::ResetPosition() {
    position = 0;
}

void MotorWaypoints::Step() {
    int8_t next_step = NextStep(); //8us

    if (next_step) {
        MotorStepper::SetDirection(next_step > 0); //4us         
        MotorStepper::SetFrequency(NextFreq()); //52us                               
        MotorStepper::Step();
        
        position += next_step;
    }    
}

int8_t MotorWaypoints::NextStep() {
    c_pos_diff = GetPositionDiff();

    if (c_pos_diff > 0) {
        return 1;
    } else if (c_pos_diff < 0) {
        return -1;
    } else {
        if (uint8_t idx = GetNextWaypoint()) {
            SetActiveWaypoint(idx);
            
            if (!IsOnDelay()) {
                return NextStep();
            }
        } else {
            in_progress = false;
        }
    }
    
    return 0;
}

unsigned int MotorWaypoints::NextFreq() { 
    int f_diff = wpts[active_wpt]->frequency - (int)MotorStepper::Frequency();  
    
    if (abs(f_diff - c_freq_adj) > 0) {
        return (int)MotorStepper::Frequency() + c_freq_adj;
    }
    
    return (int)MotorStepper::Frequency();
}

void MotorWaypoints::SetActiveWaypoint(int8_t idx) {
    active_wpt = idx;
    
    c_pos_diff   = wpts[active_wpt]->position - position;    
    c_freq_diff  = wpts[active_wpt]->frequency - (active_wpt ? wpts[active_wpt-1]->frequency : 1);        
    c_delay_left = wpts[active_wpt]->delay_ms;

    if (abs(c_freq_diff) > abs(c_pos_diff)) {
        c_freq_adj = 1 + abs(c_freq_diff/c_pos_diff);
        if (c_freq_diff < 0) c_freq_adj *= -1;
    } else {
        c_freq_adj = c_freq_diff > 0 ? 1 : -1;
    }  
    
    #ifdef DEBUG_WAYPOINT_GRAPH
    t_x1 = 0;
    #endif;
    
    #ifdef DEBUG_WAYPOINT_ACTIVE    
    Serial.print("WPT #");
    Serial.print(active_wpt);   
    Serial.print(" C-POS:");       
    Serial.print(position);   
    Serial.print(" T-POS:");       
    Serial.print(wpts[active_wpt]->position);     
    Serial.print(" D-POS:");       
    Serial.print(c_pos_diff);   
    Serial.print(" T-FREQ:");       
    Serial.print(wpts[active_wpt]->frequency);   
    Serial.print(" T-FREQ-DIFF:");       
    Serial.print(c_freq_diff);       
    Serial.print(" T-FREQ-ADJ:");       
    Serial.print(c_freq_adj);         
    Serial.print(" T-ON-KEEP:");       
    Serial.print(c_delay_left);           
    Serial.print("\n");        
    #endif
}

long MotorWaypoints::GetPositionDiff() {
    return wpts[active_wpt]->position - position;
}

int8_t MotorWaypoints::GetNextWaypoint() {
    uint8_t i = active_wpt;

    while(i < WAYPOINT_COUNT - 1) {
        if (wpts[++i]->enabled) {
            return i;
        }
    }

    return 0;
}

bool MotorWaypoints::IsPaused() { return paused; }

void MotorWaypoints::Pause() { paused = true; }

void MotorWaypoints::UnPause() { paused = false; }

void MotorWaypoints::Stop() {  active_wpt = 0; in_progress = false; }

bool MotorWaypoints::InProgress() { return in_progress; }


bool MotorWaypoints::IsOnDelay() {return (c_delay_left > 0); }

bool MotorWaypoints::Delay() { c_delay_left--; return c_delay_left > 0 ? true : false; }

void MotorWaypoints::SkipDelay() { c_delay_left = 0; }

int8_t MotorWaypoints::GetState() {
    if (InProgress()) {
        if (IsPaused()) return WPTS_STATE_PAUSED;
        if (IsOnDelay()) return WPTS_STATE_ON_DELAY;
        
        return WPTS_STATE_IN_PROGRESS;
    }
    
    return WPTS_STATE_STOPPED;
}
