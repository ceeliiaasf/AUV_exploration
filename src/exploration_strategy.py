#!/usr/bin/env python
import rospy
import numpy as np
from shapely.geometry import Polygon, LineString
from cola2_msgs.srv import Section, SectionRequest
from cola2_msgs.msg import PilotActionResult, NavSts

class Navegacion:
    def __init__(self):
        rospy.init_node('estrategia_navegacion')
        self.distancia_pasadas = rospy.get_param('~distancia_pasadas', 3.0)

        self.pos_actual = None
        rospy.Subscriber('/sparus2/navigator/navigation', NavSts, self.actualizar_posicion)

        # Suscriptor para el resultado de la sección
        self.final_status = -1
        rospy.Subscriber('/sparus2/pilot/actionlib/result', PilotActionResult, self.update_section_result)

        self.service_name = '/sparus2/captain/enable_section'
        rospy.wait_for_service(self.service_name)
        self.client_section = rospy.ServiceProxy(self.service_name, Section)

        while not rospy.has_param('/entorno/entorno_ned') and not rospy.is_shutdown():
            rospy.sleep(0.5)

        self.poligono_entorno = Polygon(rospy.get_param('/entorno/entorno_ned'))
        
        self.generar_recorrido()

    # Actualizar posicion actual del robot
    def actualizar_posicion(self, msg):
        self.pos_actual = [msg.position.north, msg.position.east]

    # Actualizar el resultado final de la mision
    def update_section_result(self, msg):
        self.final_status = msg.result.state

    def esperar_resultado(self):
        # Si el status final es 0 es que ha terminado la seccion correctamente
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.final_status == 0:
                rospy.loginfo("Seccion terminada")
                break
            elif self.final_status > 0:
                rospy.loginfo(f"Fallo: {self.final_status}")
                break
            rate.sleep()

    def hacer_seccion(self, x_in, y_in, x_fin, y_fin):
        rospy.loginfo(f"Enviando sección: N={x_in:.2f} -> N={x_fin:.2f}")
        
        req = SectionRequest()
        req.initial_x, req.initial_y = float(x_in), float(y_in)
        req.final_x, req.final_y = float(x_fin), float(y_fin)
        req.final_depth = 2.0
        req.surge_velocity = 0.5
        req.tolerance_xy = 3.0
        req.heave_mode = 0
        req.timeout = 250.0

        self.final_status = -1 # Reset antes de empezar
        
        try:
            self.client_section(req)
            rospy.sleep(1.0) 
            self.esperar_resultado()
            
            # Margen extra para que no de error de capitan
            rospy.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"Error: {e}")

    def generar_recorrido(self):
        # Limites del poligono de entorno
        min_x, min_y, max_x, max_y = self.poligono_entorno.bounds

        # Y cada x metros seleccionados en el launch
        y_puntos = np.arange(min_y + 1, max_y + 1, self.distancia_pasadas)
        ultima_pos = self.pos_actual 
        ida = True
        for y in y_puntos:
            # Linea para ver donde corta con el poligono de entorno
            linea = LineString([(min_x - 10, y), (max_x + 10, y)])
            interseccion = linea.intersection(self.poligono_entorno)
            if not interseccion.is_empty:
                puntos = list(interseccion.coords)
                A_real, B_real = [puntos[0][0], -puntos[0][1]], [puntos[-1][0], -puntos[-1][1]]
                punto_inicio, punto_fin = (A_real, B_real) if ida else (B_real, A_real)
                
                if ultima_pos:
                    self.hacer_seccion(ultima_pos[0], ultima_pos[1], punto_inicio[0], punto_inicio[1])
                self.hacer_seccion(punto_inicio[0], punto_inicio[1], punto_fin[0], punto_fin[1])
                ultima_pos = punto_fin
                ida = not ida

if __name__ == '__main__':
    try:
        Navegacion()
    except rospy.ROSInterruptException:
        pass