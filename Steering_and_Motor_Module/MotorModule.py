# Code to initialize and control speed and direction of the rear wheel motor in WillowCar
import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor():
    def __init__(self,enA,In1,In2):
        self.enA = enA;
        self.In1 = In1;
        self.In2 = In2;
        GPIO.setup(self.enA,GPIO.OUT);
        GPIO.setup(self.In1,GPIO.OUT);
        GPIO.setup(self.In2,GPIO.OUT);
        self.pwmA = GPIO.PWM(self.enA,1000);
        self.pwmA.start(0);

    def moveF(self,speed=50,t=0):
        GPIO.output(self.In1,GPIO.HIGH);
        GPIO.output(self.In2,GPIO.LOW);
        self.pwmA.ChangeDutyCycle(speed);
        sleep(t);

    def moveB(self,speed=20,t=0):
        GPIO.output(self.In1,GPIO.LOW);
        GPIO.output(self.In2,GPIO.HIGH);
        self.pwmA.ChangeDutyCycle(speed);
        sleep(t);

    def stop(self,t=0):
        self.pwmA.ChangeDutyCycle(0);
        #GPIO.output(self.In1,GPIO.LOW);
        #GPIO.output(self.In2,GPIO.LOW);
        sleep(t);

def main():
    in3 = 23;
    in4 = 24;
    enB = 25;
    motor = Motor(enB,in3,in4);
    print('Motor command 10');
    motor.moveF(10,2);
    print('Motor command 20');
    motor.moveF(20,2);
    print('Motor command 30');
    motor.moveF(30,2);
    print('Motor command 40');
    motor.moveF(40,2);
    print('Motor command 50');
    motor.moveF(50,2);
    print('Motor command 60');
    motor.moveF(60,2);
    print('Motor command 70');
    motor.moveF(70,2);
    print('Motor command 80');
    motor.moveF(80,2);
    print('Motor command 90');
    motor.moveF(90,2);
    print('Motor command 100');
    motor.moveF(100,2);
    motor.stop(2);
    print('Motor command -80');
    motor.moveB(80,3);
    motor.stop(2);
    GPIO.cleanup();

if __name__ == '__main__':
    main()
