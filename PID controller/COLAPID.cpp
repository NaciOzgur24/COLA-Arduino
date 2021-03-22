#include PID_v1

void gymbalPID(double inpin = 3, double outpin = 0, double A = 0, double target = 0, double Kp = 1.1, double Ki = 0.1, double Kd = 0.5) {
	v = analogRead(inpin);
	PID myPID(v,A,target,Kp,Ki,Kd,DIRECT);
	myPID.SetMode(AUTOMATIC);
}

void loop() {
	myPID.Compute();
    	}