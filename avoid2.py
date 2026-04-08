'''
avoid.py

Sample client for the Pioneer P3DX mobile robot that implements a
kind of heuristic, rule-based controller for collision avoidance.

Copyright (C) 2023 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import robotica

# Distribución de sensores ultrasónicos Pioneer P3DX (CoppeliaSim)
# Índices: 0 → 15 en sentido horario alrededor del robot

# FRONT (frontal)
# [0] frontal izquierdo extremo
# [1] frontal izquierdo
# [2] frontal centro-izquierda
# [3] frontal centro
# [4] frontal centro-derecha
# [5] frontal derecha
# [6] frontal derecho extremo

# RIGHT SIDE (lateral derecho)
# [7] lateral derecho

# BACK (trasero)
# [8]  trasero derecho
# [9]  trasero centro-derecha
# [10] trasero centro
# [11] trasero centro-izquierda
# [12] trasero izquierdo

# LEFT-BACK (entre trasero y lateral izquierdo)
# [13] trasero-izquierda

# LEFT SIDE (lateral izquierdo)
# [14] delantero-izquierda
# [15] lateral izquierdo

ultimo_error = 0.0

def avoid(readings, side):
    global ultimo_error
    
    # Parámetros: ¡MÁS RÁPIDO!
    target_dist = 0.50  
    VEL_MAX = 1.0       # Velocidad máxima aumentada al doble
    KP = 4.0
    KD = 2.0
    UMBRAL_VACIO = 1.2  
    
    if side is None:
        return VEL_MAX, VEL_MAX

    front_center = readings[3]

    # --- LÓGICA PARA LADO DERECHO ---
    if side == 'right':
        d_diag = readings[6]
        front_side = readings[5]
        lat_derecho = readings[7]   
        tras_derecho = readings[8]  
        
        # 1. ¡APAGAR TODO Y HACER GIRO EN U! (OVERRIDE)
        if lat_derecho > UMBRAL_VACIO:
            if tras_derecho < UMBRAL_VACIO:
                # FASE 1: El morro asoma, pero el culo sigue en la pared.
                # OBLIGATORIO ir recto a toda velocidad para pasar el filo rápido.
                return VEL_MAX, VEL_MAX
            else:
                # FASE 2: Ya pasamos la pared por completo.
                # APAGAMOS TODO y hacemos un giro en U brusco.
                # Rueda izquierda a tope, rueda derecha hacia atrás (-0.5).
                return VEL_MAX, -VEL_MAX * 0.5

        # 2. EVASIÓN FRONTAL RÁPIDA (Esquina interior)
        if front_center < 0.5 or front_side < 0.4:
            # Frenazo y giro brusco a la izquierda
            return -VEL_MAX * 0.5, VEL_MAX

        # 3. SEGUIMIENTO DE PARED NORMAL (PD)
        # Si llegamos aquí, es que hay pared y no estamos en una esquina.
        error = d_diag - target_dist
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error

    # --- LÓGICA PARA LADO IZQUIERDO ---
    elif side == 'left':
        d_diag = readings[1]
        front_side = readings[2]
        lat_izquierdo = readings[15]
        tras_izquierdo = readings[12]

        # 1. ¡APAGAR TODO Y HACER GIRO EN U! (OVERRIDE)
        if lat_izquierdo > UMBRAL_VACIO:
            if tras_izquierdo < UMBRAL_VACIO:
                return VEL_MAX, VEL_MAX
            else:
                # Rueda derecha a tope, rueda izquierda hacia atrás
                return -VEL_MAX * 0.5, VEL_MAX

        # 2. EVASIÓN FRONTAL RÁPIDA
        if front_center < 0.5 or front_side < 0.4:
            return VEL_MAX, -VEL_MAX * 0.5

        # 3. SEGUIMIENTO DE PARED NORMAL (PD)
        error = d_diag - target_dist
        ajuste = (error * KP) + ((error - ultimo_error) * KD)
        ultimo_error = error
        ajuste = -ajuste 

    # --- FINALIZACIÓN DEL PD ---
    # Solo afecta si no hemos hecho "return" arriba
    ajuste = max(min(ajuste, VEL_MAX * 0.6), -VEL_MAX * 0.6)
    lspeed = VEL_MAX + ajuste
    rspeed = VEL_MAX - ajuste

    return lspeed, rspeed


def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    
    wall_side = None 
    
    while coppelia.is_running():
        readings = robot.get_sonar()
        
        if wall_side is None:
            if readings[3] < 0.6: 
                if readings[2] < readings[5]:
                    wall_side = "left"
                else:
                    wall_side = "right"

        lspeed, rspeed = avoid(readings, side=wall_side)
        robot.set_speed(lspeed, rspeed)
        
    coppelia.stop_simulation()

if __name__ == '__main__':
    main()