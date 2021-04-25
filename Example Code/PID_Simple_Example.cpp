//PID constants
double kp = 2
double ki = 5
double kd = 1

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

void setup() {
	setPoint = 0;                          //set point at zero degrees
}

void loop() {
	input = analogRead(A0);                //read from rotary encoder connected to A0
	output = computePID(input);
	delay(100);
	analogWrite(3, output);                //control the motor based on PID value

}

double computePID(double inp) {
	currentTime = millis();                //get current time
	elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

	error = Setpoint - inp;                                // determine error
	cumError += error * elapsedTime;                // compute integral
	rateError = (error - lastError) / elapsedTime;   // compute derivative

	double out = kp * error + ki * cumError + kd * rateError;                //PID output               

	lastError = error;                                //remember current error
	previousTime = currentTime;                        //remember current time

	return out;                                        //have function return the PID output
}