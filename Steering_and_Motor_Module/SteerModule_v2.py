# Code to initialize and control Steering Wheel Angle (SWA) of WillowCar
from time import sleep
import pigpio # Get rid of gitter problem when using GPIO, http://abyz.me.uk/rpi/pigpio/index.html

class Steer():
    def __init__(self,ServoMotorPin,OncenterAngle):
        self.ServoMotorPin = ServoMotorPin;
        self.pwm = pigpio.pi();
        self.pwm.set_mode(self.ServoMotorPin, pigpio.INPUT)
        self.pwm.set_servo_pulsewidth(self.ServoMotorPin, 1500);

    def turn(self,angle_deg=0,t=0):
        angle_in = -angle_deg*33+1500; # Requires calibration
        if angle_in > 2500:
            angle_in = 2500;
        if angle_in < 500:
            angle_in = 500;
        self.pwm.set_servo_pulsewidth(self.ServoMotorPin, angle_in)
        sleep(t);

    def return_to_center(self,t=0):
        self.pwm.set_servo_pulsewidth(self.ServoMotorPin, 1500);  # Requires calibration
        sleep(t);

    def stop():
        self.pwm.stop();

def main():
    ServoMotorPin = 6;
    SteeringWheel = Steer(ServoMotorPin,0);
    sleep(2);
    angle_deg_input = 30;
    while angle_deg_input >= -30:
        print(f"Steering wheel angle: {angle_deg_input} deg.");
        SteeringWheel.turn(angle_deg_input,0.2);
        angle_deg_input = angle_deg_input - 2;
    sleep(2);
    while angle_deg_input <= 30:
        print(f"Steering wheel angle: {angle_deg_input} deg.");
        SteeringWheel.turn(angle_deg_input,0.2);
        angle_deg_input = angle_deg_input + 2;
    sleep(2);
    print("Return to center.");
    SteeringWheel.return_to_center(2);
    SteeringWheel.stop;
    sleep(2);

if __name__ == '__main__':
    main()
