import runloop, hub, motor, motor_pair, color

async def gyroFata(distanta: float, viteza: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, stop: bool = True, resetGyro: bool = True):
    d = distanta * 20.45; lastError = 0
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
    d = distanta * 20.45; lastError = 0; viteza *= -1
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

async def run1_Mondiala():
    hub.light_matrix.write("1")
    #inceput modificat
    
    await gyroFata(60, 1000)
    await runloop.sleep_ms(100)
    await gyroSpate(2, 400)
    await gyroFata(5, 1000)
    await runloop.sleep_ms(100)
    await motor.run_for_degrees(hub.port.A, -475, 1000)
    await runloop.sleep_ms(100)
    await gyroSpate(65, 1000)
    
    hub.light.color(0, color.ORANGE)
    await runloop.until(pressPlay)
    hub.light.color(0, color.RED)
    
    motor.run_for_degrees(hub.port.C, 20, 200)
    await gyroFata(50, 900, stop=False)
    await gyroFata(15, 250, resetGyro=False)
    await motor.run_for_degrees(hub.port.C, -500, 250)
    await gyroFata(10, 500)
    await gyroSpate(100, 1000, -100)
    #sfarsit modificat
    return

async def run2_Mondiala():
    hub.light_matrix.write("2")
    #inceput modificat
    await gyroFata(60, 1000, target=2, stop=False)
    await gyroFata(10, 750)
    await runloop.sleep_ms(1000)
    await gyroSpate(57, 750)
    #sfarsit modificat
    return

async def run3_Mondiala():
    hub.light_matrix.write("3")
    #inceput modificat
    
    await gyroFata(50, 800)
    await runloop.sleep_ms(200)
    await turnRight(45, 500, 100, resetGyro=False, kp = 0.08, kd = 0)
    await gyroFata(32, 350)
    await runloop.sleep_ms(200)
    await turnRight(45, 500, 100, resetGyro=False, kp = 0.1, kd = 0.2)
    await gyroFata(40.5, 800)
    await runloop.sleep_ms(200)
    await turnRight(90, 500, 100, resetGyro=False, kp = 0.1, kd = 0.2)
    await gyroFata(17.5, 250)
    await motor.run_for_degrees(hub.port.A, 525, 1000)
    
    await gyroSpate(10.5, 500)
    await turnLeft(90, 500, 100, resetGyro=False, kp = 0.08, kd = 0.3)
    await gyroFata(11.5, 250)
    await motor.run_for_degrees(hub.port.C, 75, 1000)
    
    await runloop.sleep_ms(50)

    await turnRight(40, 500, 100, resetGyro=False, kp = 0.08, kd = 0) #
    await gyroFata(10, 250) #
    await turnLeft(40, 500, 100, resetGyro=False, kp = 0.08, kd = 0) #
    await gyroFata(7.5, 500, stop = False)
    await gyroFata(100, 800, 350)
    #sfarsit modificat
    return


async def run4_Mondiala():
    hub.light_matrix.write("4")
    #inceput modificat

    await gyroFata(20, 1000)
    await gyroFata(10, 400, resetGyro=True)
    await motor.run_for_degrees(hub.port.A, 500, 1000)
    await gyroFata(9.2, 300)
    await gyroSpate(1, 200)
    await gyroFata(1.1, 200)
    await motor.run_for_degrees(hub.port.C, -2500, 1000)
    await gyroSpate(50, 1000)
    #sfarsit modificat
    return

async def run5_Mondiala():
    hub.light_matrix.write("5")
    #inceput modificat
    motor.run_for_degrees(hub.port.C, 500, -1000)
    await runloop.sleep_ms(1000)
    await gyroFata(72, 500, 17)
    await gyroSpate(65, 1000, -200)
    #sfarsit modificat
    return

async def run6_Mondiala():
    hub.light_matrix.write("6")
    await gyroFata(50, 1000)
    await turnRight(15, 500, 150)
    await gyroFata(150, 1000, -200)
    return

async def run7_Mondiala():
    hub.light_matrix.write("7")
    #inceput modificat
    await gyroFata(27, 900)
    await runloop.sleep_ms(200)
    await gyroSpate(35, 1000)
    #sfarsit modificat
    return


async def run8_Mondiala():
    hub.light_matrix.write("8")
    #inceput modificat 
    await gyroFata(57.5, 350, -60)
    await turnLeft(50, 500, 150)
    await gyroFata(60, 800, -20)
    motor.run_for_degrees(hub.port.A, 480, -1000)
    await motor.run_for_degrees(hub.port.C, -480, -1000)
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
    await playProgram(7)
    '''
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
    '''
runloop.run(main())