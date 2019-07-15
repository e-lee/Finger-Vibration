

void fadeaway()
{
  if ((faded == false) && (timerCount != FINISHED) && (pwm > 0)) { /* only enter this part of the funct iff a real touch has been detected AND timer is not finished counting */
    Serial.print("const touch detected ");
    if (timerCount == NOT_STARTED) { /*time not started*/
      /* start time */
      if (abs(converted_val - old_converted_val) < CAP_STATIONARY) { 
        /* save the initial segment we are in: */
        // first_touch = converted_val; // save the first val for comparison with later cap values		
		first_touch = checkTouch(converted_val);
		
		Timer1.restart(); //start time at the beginning of new period
        timerCount = STARTED; //signal timer has been started but not finished counting
        Serial.println("timer started!!!!!!!!!!");
        Serial.println(first_touch);
      }
    }
    else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
      // if (abs(converted_val - first_touch) > CAP_STATIONARY) {
		if(checkTouch(converted_val) != first_touch) {
			Serial.print("timer stopped???");
			Serial.println(old_converted_val);
			Serial.println(converted_val);

        /*stop counting*/
        Timer1.stop();
        /*reset the timer*/
        timerCount = NOT_STARTED; // internal timer count is reset when time is started again
      }
    }
  }
  else { /* either there is no touch, we have faded away, or we have finished counting and still need to fade */
    if (faded == true) { /*we have already faded pwm away, so the pwm is zero*/
      //      if(abs(converted_val - old_converted_val) > CAP_STATIONARY) { // if we are outside of what we consider a "constant touch"
      Serial.print("faded, first_touch = ");
      Serial.print(first_touch);
      if (checkTouch(converted_val) != first_touch || (converted_val < baseline + CAP_TOUCH)) {
        //revert to using regular pwm:
        /*reset timer count*/
        timerCount = NOT_STARTED;
        /*revert to using regular pwm*/
        faded = false;
        Serial.println("breaking out of fade");
      }
      /*else keep fadeaway*/
      pwm = 0;
    }
    else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
      Serial.print("timer stopped?????????????????? ");
      /*stop counting*/
      Timer1.stop();
      /*reset the timer*/
      timerCount = NOT_STARTED; // internal timer count is reset when time is started again
    }
    else if (timerCount == FINISHED) { /*timer has finished counting 4s (and we have not faded yet)*/
      /* start fadeaway */
      Serial.println("timer up, beginning to fade");
      touchfade();
    }
    /*else there is no touch*/
  }

}

int checkTouch(capVal) {
	if (capVal <= baseline + CAP_TOUCH + 0.3) {
		return LIGHT_TOUCH;			
	}
	else if (capVal <= baseline + CAP_TOUCH + 0.6) {
		return MED_TOUCH;
	}
	else {
		return HARD_TOUCH; 
	}
}