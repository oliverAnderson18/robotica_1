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

def follow_wall(readings):
    # --- PARÁMETROS DE CONFIGURACIÓN ---
    VEL_BASE = 1.0       # Velocidad de crucero en línea recta
    DIST_DESEADA = 0.4   # A qué distancia quieres que vaya de la pared (metros)
    KP = 2.0            # "Fuerza" de la corrección (Ganancia Proporcional)
    
    # 1. SEGURIDAD FRONTAL: Si hay algo delante, giramos sí o sí
    # Usamos los sensores centrales (3 y 4)
    if readings[3] < 0.5 or readings[4] < 0.5:
        return -0.2, 0.8  # Giro rápido sobre su eje hacia la izquierda

    # 2. SEGUIMIENTO DE PARED (DERECHA):
    # Usamos el sensor 6 o 7 que son los que miran hacia la derecha en el P3DX
    dist_actual = readings[6]
    
    # Si el sensor no detecta nada (lectura muy alta), el robot busca pared
    if dist_actual > 1.0:
        return VEL_BASE, VEL_BASE * 0.8 # Gira suave a la derecha para buscar pared

    # 3. EL TRUCO DE LA LÍNEA RECTA: El Error
    # Error positivo: estoy muy lejos de la pared. Error negativo: estoy muy cerca.
    error = dist_actual - DIST_DESEADA
    
    # Calculamos el ajuste proporcional
    ajuste = error * KP
    
    # Aplicamos el ajuste a las ruedas
    # Si error es positivo (lejos), la rueda izquierda corre más para acercarse
    lspeed = VEL_BASE + ajuste
    rspeed = VEL_BASE - ajuste

    return lspeed, rspeed

def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    while coppelia.is_running():
        readings = robot.get_sonar()
        lspeed, rspeed = follow_wall(readings)
        robot.set_speed(lspeed, rspeed)
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()