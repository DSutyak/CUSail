#include "PID.h"
#include <stdlib.h>

int current_error;
int integral_past_errors;
int rate_of_error;

// output of the PID
int output;
double *error_array;
int prev1_error;
int prev2_error;
int prev3_error;
int prev4_error;
int prev5_error;

/*The constant for P*/
int pi = 1;
/*The constant for I*/
int ki = 1;
/*The constant for D*/
int di = 1;

int read_current_position(){
    //returns current position from oscilloscope
    return 0;
}

void correct_path(){
    // moves to the correct path
}

void feedback(){
//    while(true){
//        // read the error every 10 seconds
//        delay(10000);
//        int current_error = target - read_current_position();
//        integral_past_errors = integral_past_errors + current_error;
//        rate_of_error = current_error - prev5_error;
//        prev5_error = prev4_error;
//        prev4_error = prev3_error;
//        prev3_error = prev2_error;
//        prev2_error = prev1_error;
//        prev1_error = current_error;
//        output = pi*current_error + ii*integral_past_errors + di*rate_of_error;
//        correct_path(output);
//    }
    
    // this didn't compile... also it was an infinite loop that would have prevented
    // anything else from running. Maybe put it into a thread or only have it run
    // when something is changed. - Courtney
    
}