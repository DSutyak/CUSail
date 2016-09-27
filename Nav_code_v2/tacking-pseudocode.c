
/*--------Pseudocode for a function that allows for boat to proactively determine how many times, and when it should turn, as well as what turning method it should use*/
void dynamic_turning() {
    /*
    All variables expected to be necessary (which need to be explicitly passed in?):
        * number of turns (int?)
        * course direction (double?)
        * current heading (double?)
        * course distance (float?)
        * current course progress (float?)
    */

    create variables that hold lines orthogonal to the course of travel
    make line intersections with course of travel at points roughly equal to (total distance to travel)/(number of times to turn)
    when boat crosses one of the lines of intersection: initiate turn
    keep counter to prevent boat from going in a loop where it restarts a turn whenever it falls back and crosses the line again


    // ideas for turn function calls

    jibe() /*
        All variables expected to be necessary (which need to be explicitly passed in?):
            * current heading
            * heading to change to
            * wind direction
        Do I need to pass in the change in angle? Or should I simply pass the heading we should be changing to?
    */

    tack() /*
        All variables expected to be necessary (which need to be explicitly passed in?):
            * current heading
            * heading to change to
            * wind direction
        Do I need to pass in the change in angle? Or should I simply pass the heading we should be changing to?

        Call new tack to change direction
    */

    whatever_the_hell_the_other_one_is_called() /*
        All variables expected to be necessary (which need to be explicitly passed in?):
            * current heading
            * heading to change to
            * wind direction
        Do I need to pass in the change in angle? Or should I simply pass the heading we should be changing to?
    */
}

// how is our heading determined?
// how should I approach calling turns to go to specific angles?
