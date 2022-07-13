import gpio
import time

DIR = 12
PUL = 18

gpio.setmode(gpio.BOARD)
gpio.setup([PUL, DIR], gpio.OUT)


    
pwmPUL = gpio.PWM(PUL, 10)  
pwmPUL.start(0)

def rotate(angle, direction):
    """
    旋转操作，需要指定旋转角度和方向
    :param angle: 正整型数据，旋转角度
    :param direction: 字符串数据，旋转方向，取值为："ccw"或"cw".ccw:逆时针旋转，cw:顺时针旋转
    :return:None
    """
    if direction == "ccw":
        gpio.output(DIR, gpio.LOW)
    elif direction == "cw":
        gpio.output(DIR, gpio.HIGH)
    else:
        return
    pwmPUL.ChangeDutyCycle(50)
    time.sleep(angle / 360)
    pwmPUL.ChangeDutyCycle(0)

time.sleep(0)
rotate(3600000, "ccw")

pwmPUL.stop()
gpio.cleanup()


