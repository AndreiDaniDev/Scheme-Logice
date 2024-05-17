import runloop, hub, motor, motor_pair, color

async def gyroFataNoShow(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, stop: bool = True, resetGyro: bool = True):
    d = int(distanta * 20.45); lastError = 0
    if(resetGyro): hub.motion_sensor.reset_yaw(0);
    motor.reset_relative_position(hub.port.B, 0)
    while(abs(motor.relative_position(hub.port.B)) < d):
        eroare = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        proportionala = eroare * -kp
        diferentiala = (eroare - lastError) * -kd
        lastError = eroare + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(viteza + proportionala + diferentiala), int(viteza - proportionala - diferentiala))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def gyroSpateNoShow(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3 , stop: bool = True, resetGyro: bool = True):
    d = int(distanta * 20.45); lastError = 0; viteza *= -1
    if(resetGyro): hub.motion_sensor.reset_yaw(0);
    motor.reset_relative_position(hub.port.B, 0)
    while(abs(motor.relative_position(hub.port.B)) < d):
        eroare = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        proportionala = eroare * kp
        diferentiala = (eroare - lastError) * kd
        lastError = eroare + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(viteza - proportionala - diferentiala), int(viteza + proportionala + diferentiala))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnLeftNoShow(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(resetGyro): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    eroare = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(eroare < -5 or eroare > 5):
        eroare = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * eroare + kd * (eroare - lastError)
        lastError = eroare
        if(abs(output) > vitezaMax): output = vitezaMax
        if(abs(output) < vitezaMin): output = vitezaMin
        motor_pair.move_tank(motor_pair.PAIR_1, -int(output), int(output))
    motor_pair.stop(motor_pair.PAIR_1)

async def turnRightNoShow(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(resetGyro): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    eroare = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(eroare < -5 or eroare > 5):
        eroare = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * eroare + kd * (eroare - lastError)
        lastError = eroare
        if(abs(output) > vitezaMax): output = vitezaMax
        if(abs(output) < vitezaMin): output = vitezaMin
        motor_pair.move_tank(motor_pair.PAIR_1, int(output), -int(output))
    motor_pair.stop(motor_pair.PAIR_1)

async def gyroFata(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, ki: float = 0, iterator : int = 5, stop: bool = True, resetGyro: bool = True):
    d = int(distanta * 20.45); lastError = 0
    if(resetGyro): hub.motion_sensor.reset_yaw(0);
    motor.reset_relative_position(hub.port.B, 0)
    integral = 0; i = 0;
    print("Gyro Forward", "Distance: ", distanta, "Speed: ", viteza,"Target: ", target,"Kp: ", kp,"Kd: ", kd, "Ki: ", ki, ",Reset:", resetGyro, ",Stop:", stop, ",Iterator", iterator)
    await runloop.sleep_ms(250)
    while(abs(motor.relative_position(hub.port.B)) < d):
        eroare = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        integral += eroare; i += 1
        if(i % iterator == 0):
            print(str(eroare), end = " ")
        proportionala = eroare * -kp
        diferentiala = (eroare - lastError) * -kd
        lastError = eroare + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(viteza + proportionala + diferentiala), int(viteza - proportionala - diferentiala))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    print(end = '\n')

async def gyroSpate(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, ki: float = 0, iterator : int = 5, stop: bool = True, resetGyro: bool = True):
    d = int(distanta * 20.45); lastError = 0; viteza *= -1
    if(resetGyro): hub.motion_sensor.reset_yaw(0);
    motor.reset_relative_position(hub.port.B, 0)
    #print("Gyro Spate", distanta, viteza, target, kp, kd)
    i = 0; integral = 0;
    print("Gyro Backwards", "Distance: ", distanta, "Speed: ", viteza,"Target: ", target,"Kp: ", kp,"Kd: ", kd, "Ki: ", ki, "Reset:", resetGyro, ",Stop:", stop, ",Iterator", iterator)
    await runloop.sleep_ms(250)
    while(abs(motor.relative_position(hub.port.B)) < d):
        eroare = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        integral += eroare; i += 1
        if(i % iterator == 0):
            print(str(eroare), end = " ")
        proportionala = eroare * kp
        diferentiala = (eroare - lastError) * kd
        lastError = eroare + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(viteza - proportionala - diferentiala), int(viteza + proportionala + diferentiala))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)
    print(end = '\n')

async def turnLeft(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35, iterator : int = 5):
    if(resetGyro): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    eroare = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target; i = 0
    print("Turn Left", "Degrees: ", grade, "Speed Max: ", vitezaMax,"Speed Min: ", vitezaMin,"Kp: ", kp,"Kd: ", kd, ",Reset: ", resetGyro, "Iterator: ", iterator)
    await runloop.sleep_ms(250)
    while(eroare < -5 or eroare > 5):
        eroare = target - abs(hub.motion_sensor.tilt_angles()[0]); i += 1
        output = kp * eroare + kd * (eroare - lastError)
        lastError = eroare
        if(abs(output) > vitezaMax): output = vitezaMax
        if(abs(output) < vitezaMin): output = vitezaMin
        if(i % iterator == 0):
            print(str(eroare), str(output), end = " ")
        motor_pair.move_tank(motor_pair.PAIR_1, -int(output), int(output))
        await runloop.sleep_ms(2)
    motor_pair.stop(motor_pair.PAIR_1)
    print(end = '\n')

async def turnRight(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35, iterator : int = 5):
    if(resetGyro): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    eroare = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target; i = 0
    print("Turn Right", "Degrees: ", grade, "Speed Max: ", vitezaMax,"Speed Min: ", vitezaMin,"Kp: ", kp,"Kd: ", kd, ",Reset: ", resetGyro, "Iterator: ", iterator)
    while(eroare < -5 or eroare > 5):
        eroare = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * eroare + kd * (eroare - lastError)
        lastError = eroare; i += 1
        if(abs(output) > vitezaMax): output = vitezaMax
        if(abs(output) < vitezaMin): output = vitezaMin
        if(i % iterator == 0):
            print(str(eroare), str(output), end = " ")
        motor_pair.move_tank(motor_pair.PAIR_1, int(output), -int(output))
        await runloop.sleep_ms(2)
    motor_pair.stop(motor_pair.PAIR_1)
    print("\n")
async def main():
    motor_pair.pair(motor_pair.PAIR_1, hub.port.B, hub.port.D)
    #await gyroFataNoShow(15, 150, kp = 0.2, kd = 0.1)
    #await turnLeft(145, 750, 100, kp = 1 / 4, kd = 1 / 2)
    await gyroFata(25, 500, kp = 1.25, kd = 0.4, ki = 0, iterator = 20)
    #await gyroSpateNoShow(51, 1000)
    await runloop.sleep_ms(500)
    #print("Actual value -> ", abs(hub.motion_sensor.tilt_angles()[0]), sep = "")
    print("Finish")
runloop.run(main())
