import runloop, hub, motor, motor_pair, color

async def gyroFata(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, stop: bool = True, resetGyro: bool = True):
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

async def gyroSpate(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3 , stop: bool = True, resetGyro: bool = True):
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

async def turnLeft(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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

async def turnRight(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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

async def turnLeftOneWheel(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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
        motor_pair.move_tank(motor_pair.PAIR_1, 0, int(output))
    motor_pair.stop(motor_pair.PAIR_1)

async def turnRightOneWheel(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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
        motor_pair.move_tank(motor_pair.PAIR_1, int(output), 0)
    motor_pair.stop(motor_pair.PAIR_1)

async def turnLeftOtherWheel(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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
        motor_pair.move_tank(motor_pair.PAIR_1, -int(output), 0)
    motor_pair.stop(motor_pair.PAIR_1)

async def turnRightOtherWheel(grade: float, vitezaMax: int = 500, vitezaMin: int = 150, * , resetGyro: bool = True, kp: float = 0.25, kd: float = 0.35):
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
        motor_pair.move_tank(motor_pair.PAIR_1, 0, -int(output))
    motor_pair.stop(motor_pair.PAIR_1)


async def run1_Mondiala():
    hub.light_matrix.write("1")
    #inceput modificat
    print("Run 1 - Parte 1")

    await gyroFata(60, 1000)
    await runloop.sleep_ms(100)
    await gyroSpate(2, 400)
    await gyroFata(5, 1000)
    await runloop.sleep_ms(100)
    await motor.run_for_degrees(hub.port.A, -475, 1000)
    await runloop.sleep_ms(100)
    await gyroSpate(57.5, 1000)

    hub.light.color(0, color.ORANGE)
    await runloop.until(pressPlay)
    hub.light.color(0, color.RED)

    print("Run 1 - Parte 2")

    await motor.run_for_degrees(hub.port.C, -10, 200)
    motor.run_for_degrees(hub.port.A, 200, 1000)
    motor.run_for_degrees(hub.port.C, 350, 400)
    await gyroFata(50, 500, -18, stop=False, kp = 0.4)
    await gyroFata(16.75, 275)
    await runloop.sleep_ms(500)
    await motor.run_for_degrees(hub.port.C, -500, 400)
    await gyroFata(7, 200)
    await runloop.sleep_ms(500)
    await gyroSpate(70, 1000, 750)
    #sfarsit modificat
    return

async def run2_Mondiala():
    hub.light_matrix.write("2")
    #inceput modificat
    print("Run 2")
    await gyroFata(72.5, 1000, 150)
    await runloop.sleep_ms(1000)
    await gyroSpate(57, 1000, -175)
    #sfarsit modificat
    return

async def run3_Mondiala():
    hub.light_matrix.write("3")
    #inceput modificat
    print("Run 3")

    # ---> Perpendicular cu peretele <---
    await gyroFata(52.5, 750, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(100)
    await turnRightOneWheel(42, 500, 75, kp = 0.47, kd = 0.3, resetGyro = False)
    await runloop.sleep_ms(250)
    await gyroFata(18.5, 375, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(100)
    await turnRightOneWheel(42, 500, 100, kp = 0.5, kd = 0.3)
    await runloop.sleep_ms(250)

    # ---> Misiunea din centru <---
    await gyroFata(51.6, 750, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(200)
    await turnRightOtherWheel(81.25, 500, 100, kp = 0.5, kd = 0.3, resetGyro = False)
    await runloop.sleep_ms(250)
    await gyroFata(12.35, 250, kp = 0.4, kd = 0.3)
    await motor.run_for_degrees(hub.port.A, 500, 1000)

    # ---> Misiune Floare <---
    await gyroSpate(14.5, 250, kp = 0.4, kd = 0.3)
    await turnLeft(85, 400, 100, resetGyro=False, kp = 0.5, kd = 0.3)
    await gyroFata(10.5, 300, kp = 0.4, kd = 0.3)
    await motor.run_for_degrees(hub.port.C, 75, 1000)

    await turnRight(40, 500, 100, resetGyro=False, kp = 0.5, kd = 3) #
    await gyroFata(10, 250)
    await turnLeft(40, 500, 100, resetGyro=False, kp = 0.3, kd = 0)
    await gyroFata(7.5, 500, stop = False)
    await gyroFata(115, 800, 200)

    #sfarsit modificat
    return


async def run4_Mondiala():
    hub.light_matrix.write("4")
    #inceput modificat
    print("Run 4")
    await gyroFata(21.5, 800)
    await gyroFata(10, 400, resetGyro=True)
    await runloop.sleep_ms(50)
    await motor.run_for_degrees(hub.port.A, 500, 325)
    await gyroFata(9.2, 300)
    await gyroSpate(1, 200)
    await gyroFata(1.1, 200)
    await motor.run_for_degrees(hub.port.C, -2500, 1000)
    await gyroSpate(50, 1000)
    #sfarsit modificat
    return

async def run5_Mondiala():
    #inceput modificat
    print("Run 5")

    hub.motion_sensor.reset_yaw(0)
    await motor.run_for_degrees(hub.port.C, 300, -1100)
    await runloop.sleep_ms(500)
    # ---> Comanada de 5,50 lei <---
    await gyroFata(45, 600, kp = 0.4, kd = 0.25, resetGyro = False)
    await runloop.sleep_ms(250)
    await turnLeft(42.5, 650, 125, kp = 0.75, kd = 1)
    await runloop.sleep_ms(250)

    # ---> Comanda de 2 lei <---
    await gyroFata(5.5, 400, kp = 0.3, kd = 0.25, stop = False)
    await gyroFata(14.5, 500, kp = 0.3, kd = 0.25, resetGyro = False, stop = False)
    await gyroFata(5, 250, kp = 0.3, kd = 0.25, resetGyro = False)

    # ---> Vorbim de margina de eroare <---
    await runloop.sleep_ms(500)
    await turnRight(77.5, 500, 175, kp = 0.75, kd = 1.2, resetGyro = False)

    # ---> Dummy proof <---
    #    ->>> 20 lei <<<-
    await motor.run_for_degrees(hub.port.C, 300, 1100)
    await runloop.sleep_ms(250)
    await gyroFata(14, 500, target = 35, kp = 0.4, kd = 0.25)
    await gyroFata(10, 800, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(250)
    await gyroSpate(14.5, 500, kp = 0.4, kd = 0.25)

    await turnLeft(60, 500, 250, kp = 0.4, kd = 0.25)
    await gyroSpate(75, 1000, kp = 0.4, kd = 0.25)

    #sfarsit modificat
    return

async def run6_Mondiala():
    hub.light_matrix.write("6")
    #inceput modificat
    print("Run 6")
    await gyroFata(50, 1000, kp = 0.2, kd = 0.3)
    await turnRight(30, 500, 100, kp = 0.2, kd = 0)
    await gyroFata(120, 1000, -425, kp = 0.2, kd = 0.3)
    #sfarsit modificat
    return

async def run7_Mondiala():
    hub.light_matrix.write("7")
    #inceput modificat
    print("Run 7")
    await gyroFata(27, 600, -35)
    await gyroSpate(35, 1000)
    #sfarsit modificat
    return

async def run8_Mondiala():
    hub.light_matrix.write("8")
    #inceput modificat
    print("Run 8")
    motor.run_for_degrees(hub.port.A, 375, 300)
    await gyroFata(20, 500, kp = 0.2, kd = 0.3)
    await turnLeft(10, 400, 150, resetGyro = False, kp = 0.02, kd = 0)
    await gyroFata(37.5, 500, kp = 0.2, kd = 0.3)
    await turnLeft(65, 500, 150, resetGyro = False)
    await gyroFata(45, 750, 50, kp = 0.2, kd = 0.3)

    motor.run_for_degrees(hub.port.A, -360, 500)
    await motor.run_for_degrees(hub.port.C, 500, 1000)
    await gyroSpate(5, 500)
    #sfarsit modificat
    return

async def playProgram(p: int):
    if(p == 1): await run1_Mondiala(); return;
    elif(p==2): await run2_Mondiala(); return;
    elif(p==3): await run3_Mondiala(); return;
    elif(p==4): await run4_Mondiala(); return;
    elif(p==5): await run5_Mondiala(); return;
    elif(p==6): await run6_Mondiala(); return;
    elif(p==7): await run7_Mondiala(); return;
    elif(p==8): await run8_Mondiala(); return;
    else: return
global program, releasedButton, frecventa
def play():
    return not (hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
def pressPlay():
    return (bool)(hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
async def main():
    motor_pair.pair(motor_pair.PAIR_1, hub.port.B, hub.port.D)
    #await playProgram(3)
    program = 1
    hub.light_matrix.write("1")
    hub.light.color(0, color.YELLOW)
    while(1):
        if(hub.button.pressed(hub.button.LEFT)):
            await runloop.sleep_ms(250)
            if(hub.button.pressed(hub.button.RIGHT)):
                hub.light.color(0, color.ORANGE)
                await runloop.until(play)
                await runloop.sleep_ms(50)
                hub.light.color(0, color.RED)
                await playProgram(program)
                hub.light_matrix.write(str(program))
                hub.light.color(0, color.YELLOW)
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
            else:
                program-=1
                if(program == 0): program = 8
                hub.light_matrix.write(str(program))
        elif(hub.button.pressed(hub.button.RIGHT)):
            await runloop.sleep_ms(250)
            if(hub.button.pressed(hub.button.LEFT)):
                hub.light.color(0, color.ORANGE)
                await runloop.until(play)
                await runloop.sleep_ms(50)
                hub.light.color(0, color.RED)
                await playProgram(program)
                hub.light_matrix.write(str(program))
                hub.light.color(0, color.YELLOW)
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
            else:
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
        await runloop.sleep_ms(50)

runloop.run(main())
