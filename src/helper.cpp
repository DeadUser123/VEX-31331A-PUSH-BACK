#include "main.h"
#include "globals.hpp"

// helper functions go here, make sure to update helper.hpp when adding new functions
bool flap_state = false;
bool matchloader_state = false;

void exampleFunction() {
    // Example function that does nothing
    pros::lcd::set_text(2, "Example function called");
}

void setIntake(int speed) {

}

void setHopperMotor(int speed) {
    
}

void set2ndStage(int speed) {

}

void set3rdStage(int speed) {
    
}

void toggleflap() {
    flap.set_value(!flap_state);
    flap_state = !flap_state;
}

void toggleMatchloader() {
    matchloader.set_value(!matchloader_state);
    matchloader_state = !matchloader_state;
}

/*
<SC6> An Autonomous Win Point is awarded to any Alliance that ends the Autonomous Period with all of the following tasks completed, and that has committed no Violations during the Autonomous Period:

At least seven (7) Blocks of the Alliance’s color are Scored.
At least three (3) different Goals include at least one (1) Scored Block of the Alliance’s color.
At least three (3) Blocks of the Alliance’s color have been removed from Loaders adjacent to the Alliance’s Alliance Station.
Neither Robot is contacting the Park Zone barrier.
For events which qualify directly to the World Championship (e.g., Event Region Championships and Signature Events), the following tasks must be completed for an Alliance to receive an Autonomous Win Point. The standard criteria above still apply to all other events.

At least ten (10) Blocks of the Alliance’s color are Scored.
At least three (3) different Goals include at least two (2) Scored Blocks of the Alliance’s color.
At least three (3) Blocks of the Alliance’s color have been removed from Loaders adjacent to the Alliance’s Alliance Station.
Neither Robot is contacting the Park Zone barrier.
*/
void autons() {

}

/*
Each Block Scored in a Goal - 1 Point
Each filled Control Zone in a Long Goal - 5 Points
Each filled Control Zone in a Center Goal - 10 Points
Each Cleared Park Zone - 5 Points
Each Cleared Loader - 5 Points
Parked Robot - 15 Points
*/
void skills() {

}