// Test oracle only: compile against the FC-included library, never the standalone sketch.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#define private public
#include "airbrakes.h"
#include "rollcontrol.h"
#undef private
static uint32_t nowMs;
uint32_t millis() { return nowMs; }
static airbrakes controller; // Matches FC static-storage initialization.
static rollcontrol roll;
int main(int argc,char** argv) {
    int scenario=argc>1?std::atoi(argv[1]):0;
    roll.begin();
    std::puts("boot_us,flight_s,altitude,velocity,acceleration,apogee,state,samples,deployment,predicted,target,integral,roll_deg");
    for(int ms=0;ms<=40000;ms+=10) {
        nowMs=1000+ms;
        float physical=ms/1000.0f;
        float t=scenario==0?float(ms/1000):physical;
        float h=500+350*physical-5*physical*physical;
        float v=350-10*physical;
        if(scenario==2 && physical<12) v=450; // Partial sample set at timeout.
        if(scenario==3 && physical<14) v=450; // No prep before timeout.
        AirbrakesData input={h,v,-10,physical>=35};
        controller.update(t,input);
        roll.update(physical,h,v,2.0f,-0.5f);
        std::printf("%u,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g\n",nowMs*1000,t,h,v,input.accel_z,int(input.apogeeReached),int(controller.getState()),controller.datIndex,controller.getDeployment(),controller.predictedAlt,controller.desiredAlt,controller.lastI,roll.getAngle());
    }
}
