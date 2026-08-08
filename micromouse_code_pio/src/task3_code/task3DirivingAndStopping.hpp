#include <Arduino.h>
#include "Micromouse.hpp"
#include "task3_code/task3Tracking.hpp"

void drivingAndStopping(Micromouse& mouse)
{
    int currDist = mouse.getMedianDistance();
    // int currDist = getLidarDistanceFront();

    Serial.print("\tLidar Front: ");
    Serial.println(currDist);

    if (currDist == -1)
    {
        return;
    }

    int error = currDist - 100;
    if (abs(error) <= 7.5)
    {
        mouse.move(0, 0);
        return;
    }

    if (error > 0)
    {
        // travelDistance(error, 75);
        // travelDistanceModified(error, 75);
        Modified_Tracking(mouse, error, 75);
    }
    else
    {
        // travelDistance(-error, -75);
        // travelDistanceModified(-error, -75);
        Modified_Tracking(mouse, abs(error), -75);
    }

    // if (error > 0) {
    //   if (error > 20) {
    //     task3_Tracking(20, 75);
    //   } else {
    //     task3_Tracking(error, 30);
    //   }
    // } else {
    //   if (abs(error) > 20) {
    //     task3_Tracking(20, -75);
    //   } else {
    //     task3_Tracking(abs(error), -30);
    //   }
    // }
}