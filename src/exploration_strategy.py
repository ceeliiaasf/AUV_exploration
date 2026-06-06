#!/usr/bin/env python
import rospy
import numpy as np
from shapely.geometry import Polygon, LineString, Point, MultiPoint
from cola2_msgs.srv import Section, SectionRequest
from cola2_msgs.msg import PilotActionResult, NavSts
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as PointROS
from std_srvs.srv import Trigger, TriggerRequest

class Navegacion:
    def __init__(self):
        rospy.init_node('estrategia_navegacion')
        self.distancia_pasadas = rospy.get_param('~distancia_pasadas', 3.0)
        self.tipo_deteccion = rospy.get_param('~tipo_deteccion', 1)

        # Variable para guardar los puntos detectados de la posidonia
        self.puntos_posidonia = [[]]
        self.indice_posidonia_actual = 0
        self.modo_seguimiento = False
        self.detener_seccion = False
        self.punto_interrupcion_barrido = None
        self.hubo_deteccion_en_pasada = False
        self.clapas_detectadas = []

        # Lado cuadrado para el modo 2
        self.lado = 6.0

        if (self.tipo_deteccion == 1 or self.tipo_deteccion == 3):
            self.distancia_min_entre_puntos = 0.8
        elif self.tipo_deteccion == 2:
            self.distancia_min_entre_puntos = self.lado/4.0 + 0.1

        self.pos_actual = None

        rospy.Subscriber('/sparus2/navigator/navigation', NavSts, self.actualizar_posicion)

        # Cliente para detener al robot
        self.service_disable = '/sparus2/captain/disable_section'
        rospy.wait_for_service(self.service_disable)
        self.client_disable_section = rospy.ServiceProxy(self.service_disable, Trigger)

        # Suscriptor para el resultado de la sección
        self.final_status = -1
        rospy.Subscriber('/sparus2/pilot/actionlib/result', PilotActionResult, self.update_section_result)

        self.service_name = '/sparus2/captain/enable_section'
        rospy.wait_for_service(self.service_name)
        self.client_section = rospy.ServiceProxy(self.service_name, Section)

        while (not rospy.has_param('/entorno/entorno_ned') or not rospy.has_param('/entorno/posidonia_ned')) and not rospy.is_shutdown():
            rospy.sleep(4.0)

        self.poligono_entorno = Polygon(rospy.get_param('/entorno/entorno_ned'))
        datos_posidonia = rospy.get_param('/entorno/posidonia_ned')
        self.poligonos_posidonia = [Polygon(puntos) for puntos in datos_posidonia]

        rospy.sleep(0.5) 

        self.publicador_marker = rospy.Publisher('visualizacion_deteccion_posidonia', Marker, queue_size=10)
        self.marker_posidonia = Marker()
        self.marker_posidonia.header.frame_id = "world" # O "map"
        self.marker_posidonia.ns = "deteccion"
        self.marker_posidonia.id = 0
        self.marker_posidonia.type = Marker.POINTS # Points para ver todos los puntos a la vez
        self.marker_posidonia.action = Marker.ADD
        self.marker_posidonia.pose.orientation.w = 1.0
        
        # Tamaño del punto
        self.marker_posidonia.scale.x = 0.5 
        self.marker_posidonia.scale.y = 0.5
        self.marker_posidonia.color.r = 1.0
        self.marker_posidonia.color.a = 1.0

        self.generar_recorrido()
        self.calcular_area()

    # Actualizar posicion actual del robot
    def actualizar_posicion(self, msg):
        self.pos_actual = [msg.position.north, msg.position.east]
        self.yaw_actual = msg.orientation.yaw
        
        # Se comprueba si hay detección con posidonia
        if self.poligonos_posidonia:
            si_hay_deteccion = self.comprobar_deteccion_posidonia()
            # Si se detecta un punto nuevo y estamos en modo 2 hay que detener la seccion, en el 3 solo si es el incio del seguimiento
            if si_hay_deteccion:
                if self.tipo_deteccion == 2:
                    self.detener_seccion = True
                    self.modo_seguimiento = True
                elif self.tipo_deteccion == 3:
                    if not self.modo_seguimiento:
                        self.detener_seccion = True
                    self.modo_seguimiento = True

    # Comprueba si se ha detectado posidonia
    def comprobar_deteccion_posidonia(self):
        x_now, y_now = self.pos_actual[0], -self.pos_actual[1]
        
        if self.esta_sobre_posidonia():
            
            # Si se detecta posidonia, se mira si es un punto lo suficientemente alejado respecto a uno detectado como para considerarlo nuevo
            punto_actual = Point(self.pos_actual)
            es_punto_nuevo = True
            if self.clapas_detectadas:
                for clapa in self.clapas_detectadas:
                    if clapa.buffer(3.0).contains(punto_actual):
                        return False
            
            for clapa in self.puntos_posidonia:
                for p in clapa:
                    distancia = punto_actual.distance(Point(p))

                    if distancia < self.distancia_min_entre_puntos:
                        es_punto_nuevo = False
                        break

                if not es_punto_nuevo:
                    break

            if es_punto_nuevo:
                self.hubo_deteccion_en_pasada = True
                self.dibujar_punto(x_now, y_now)
                return True
                
        return False
    
    def esta_sobre_posidonia(self):
        if (self.tipo_deteccion == 1 or self.tipo_deteccion == 3):
            # Si es el modo 1 ha de mirar si la posicion actual se encuentra en cualquier parte de la posidonia
            punto_actual = Point(self.pos_actual[0], -self.pos_actual[1])
            
            for poligono in self.poligonos_posidonia:
                # Tolerancia de margen (buffer) si es necesario, o contains directo
                if poligono.contains(punto_actual):
                    return True
            return False
        
        # Si es el modo 2, solo se mira si intersecta el robot con el contorno
        elif (self.tipo_deteccion == 2):
            x_now, y_now = self.pos_actual[0], -self.pos_actual[1]
            linea = LineString([(x_now - 0.2, y_now), (x_now + 0.2, y_now)])

            for poligono in self.poligonos_posidonia:
                if linea.intersects(poligono.boundary):
                    return True
            return False
    
    # Actualizar el resultado final de la sección
    def update_section_result(self, msg):
        self.final_status = msg.result.state

    def esperar_resultado(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            # Detener la seccion si se ha activado el flag
            if self.detener_seccion:
                try:
                    self.client_disable_section(TriggerRequest())
                    rospy.sleep(1.5)
                except:
                    pass
                break
            
            # Si el resultado final es 0, ha acabado la sección correctamente
            elif self.final_status == 0:
                break

            # Si el resultado es mayor a 1 ha dado algún tipo de error
            elif self.final_status > 0:
                rospy.loginfo(f"Fallo: {self.final_status}")
                break
            rate.sleep()

    def hacer_seccion(self, x_in, y_in, x_fin, y_fin):
        self.detener_seccion = False
        req = SectionRequest()
        req.initial_x, req.initial_y = float(x_in), float(y_in)
        req.final_x, req.final_y = float(x_fin), float(y_fin)
        req.final_depth = 2.0
        req.initial_depth = 2.0
        req.surge_velocity = 0.5
        req.tolerance_xy = 1.0
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

    def dibujar_punto(self, x_robot_respecto_mapa, y_robot_respecto_mapa):
        self.puntos_posidonia[self.indice_posidonia_actual].append(self.pos_actual)
        p = PointROS()
        p.x = x_robot_respecto_mapa
        p.y = y_robot_respecto_mapa
        p.z = -2.0
        
        self.marker_posidonia.points.append(p)
        self.marker_posidonia.header.stamp = rospy.Time.now()
        self.publicador_marker.publish(self.marker_posidonia)

    def generar_recorrido(self):
        # Bordes del poligono de entorno
        min_x, min_y, max_x, max_y = self.poligono_entorno.bounds

        # y cada x distancia
        y_puntos = np.arange(min_y + 1, max_y + 1, self.distancia_pasadas)
        ida = True

        for y in y_puntos:
            if rospy.is_shutdown():
                break
            
            linea = LineString([(min_x - 10, y), (max_x + 10, y)])
            interseccion = linea.intersection(self.poligono_entorno)
            
            if not interseccion.is_empty:
                puntos = list(interseccion.coords)
                A_real, B_real = [puntos[0][0], -puntos[0][1]], [puntos[-1][0], -puntos[-1][1]]
                punto_inicio, punto_fin = (A_real, B_real) if ida else (B_real, A_real)
                
                # Ir al inicio de la línea
                self.hacer_seccion(self.pos_actual[0], self.pos_actual[1], punto_inicio[0], punto_inicio[1])
                
                destino_alcanzado = False
                
                while not destino_alcanzado and not rospy.is_shutdown():
                    self.hubo_deteccion_en_pasada = False
                    
                    self.hacer_seccion(punto_inicio[0], punto_inicio[1], punto_fin[0], punto_fin[1])
                    
                    # Comprobamos si la sección fue interrumpida y se ha de generar el cuadrado del modo 2
                    if self.detener_seccion and self.modo_seguimiento:
                        if self.tipo_deteccion == 2:
                            rospy.loginfo("Iniciando cuadrado de seguimiento")
                            self.generar_cuadrado() 
                            
                        elif self.tipo_deteccion == 3:
                            rospy.loginfo("Iniciando modo wall following")
                            self.wall_following()
                        
                        # Reset de detener_seccion y aumentar el indice de la posidonia
                        self.detener_seccion = False
                        self.indice_posidonia_actual += 1
                        self.puntos_posidonia.append([])
                    
                    # Si no se ha detenido, se ha acabado la seccion
                    elif not self.detener_seccion:
                        destino_alcanzado = True
                        
                        # Al terminar la seccion ha de comprobar si ha habido deteccion de posidonia
                        if len(self.puntos_posidonia[self.indice_posidonia_actual]) > 0:
                            if not self.hubo_deteccion_en_pasada:
                                # Si no se ha detectado y esta en el modo 1, se han de ejecutar pasadas verticales
                                if self.tipo_deteccion == 1:
                                    self.detener_seccion = True
                                    self.realizar_pasadas_verticales()
                                    self.detener_seccion = False
                                
                                # Despues de realizar las pasadas verticales se ha de incrementar el indice
                                self.indice_posidonia_actual += 1
                                self.puntos_posidonia.append([])

                ida = not ida
        # Si en la ultima pasada ha detectado posidonia y esta en el modo 1 ha de hacer las pasadas verticales
        if (self.tipo_deteccion == 1 and len(self.puntos_posidonia[self.indice_posidonia_actual]) > 0):
            self.realizar_pasadas_verticales()
            self.indice_posidonia_actual += 1
            self.puntos_posidonia.append([])

    def realizar_pasadas_verticales(self):
        puntos_actuales = self.puntos_posidonia[self.indice_posidonia_actual]

        # Calcular limites poligono
        offset = self.distancia_pasadas
        xs = [p[0] for p in puntos_actuales]
        ys = [p[1] for p in puntos_actuales]
        
        min_x, max_x = min(xs) - offset, max(xs) + offset
        min_y, max_y = min(ys) - offset, max(ys) + offset

        # Generar x cada distancia pasadas
        x_puntos = np.arange(min_x, max_x + 1, self.distancia_pasadas)
        # Empezar en la segunda X y terminar en la penúltima
        x_puntos = x_puntos[1:-1]
        ida_vertical = True

        for x in x_puntos:
            punto_inicio = [x, min_y] if ida_vertical else [x, max_y]
            punto_fin = [x, max_y] if ida_vertical else [x, min_y]

            # Ir al inicio de la pasada vertical
            self.hacer_seccion(self.pos_actual[0], self.pos_actual[1], punto_inicio[0], punto_inicio[1])

            # Hacer la pasda vertical
            self.hubo_deteccion_en_pasada = False
            self.hacer_seccion(punto_inicio[0], punto_inicio[1], punto_fin[0], punto_fin[1])

            # Comprobar si hubo algun punto, en caso negativo, dividir los puntos en dos posidonias
            if not self.hubo_deteccion_en_pasada:
                x_corte = self.pos_actual[0]
                
                puntos_izq = []
                puntos_der = []
                
                # Clasificar puntos a izquierda o derecha del robot
                for p in self.puntos_posidonia[self.indice_posidonia_actual]:
                    if p[0] < x_corte:
                        puntos_izq.append(p)
                    else:
                        puntos_der.append(p)
                
                # Si hay puntos a ambos lados de la pasada sin deteccion
                if len(puntos_izq) > 0 and len(puntos_der) > 0:
                    self.puntos_posidonia[self.indice_posidonia_actual] = puntos_izq
                    self.puntos_posidonia.append(puntos_der)
                    # Indice es igual al ultimo añadido
                    self.indice_posidonia_actual = len(self.puntos_posidonia) - 1

            ida_vertical = not ida_vertical
        
        self.hubo_deteccion_en_pasada = False

    def generar_cuadrado(self):
        mitad = self.lado / 2.0 
        punto_inicio_clapa = Point(self.pos_actual)

        while self.modo_seguimiento and not rospy.is_shutdown():
            centro_x = self.pos_actual[0]
            centro_y = -self.pos_actual[1]

            # Puntos de las secciones del cuadrado
            puntos_cuadrado = [
                (centro_x + mitad, centro_y),         # De frente hacia la izquierda
                (mitad + centro_x, mitad + centro_y),    # Esquina abajo Izquierda
                (-mitad + centro_x, mitad + centro_y),   # Esquina abajo Derecha
                (-mitad + centro_x, -mitad + centro_y),  # Esquina arriba Derecha
                (mitad + centro_x, -mitad + centro_y),   # Esquina arriba Izquierda
                (mitad + centro_x, centro_y)          # Cierre
            ]

            self.detener_seccion = False

            for target_x, target_y_mapa in puntos_cuadrado:
                self.hacer_seccion(self.pos_actual[0], self.pos_actual[1], target_x, -target_y_mapa)
                rospy.sleep(0.5)

                if self.detener_seccion:
                    self.detener_seccion = False
                    
                    if len(self.puntos_posidonia[self.indice_posidonia_actual]) > 3:
                        dist_inicio = Point(self.pos_actual).distance(punto_inicio_clapa)
                        # Si la distancia al punto de inicio es menor que el lado del cuadrado se considera el poligono cerrado
                        if dist_inicio < self.lado:
                            rospy.loginfo("Fin contorno posidonia")
                            self.modo_seguimiento = False
                            return
                    break 

    def wall_following(self):
        paso = 2.0
        punto_inicio = Point(self.pos_actual)

        self.modo_seguimiento = True
        self.detener_seccion = False

        deteccion_anterior = not self.esta_sobre_posidonia()
        rumbo_seguimiento = self.yaw_actual

        while self.modo_seguimiento and not rospy.is_shutdown():
            deteccion_actual = self.esta_sobre_posidonia()

            # Acaba de salir de la posidonia
            if deteccion_anterior and not deteccion_actual:
                rumbo_seguimiento += np.deg2rad(90)

            # Acaba de entrar en la posidonia
            elif not deteccion_anterior and deteccion_actual:
                rumbo_seguimiento += np.deg2rad(-45)

            # Sigue dentro
            elif deteccion_anterior and deteccion_actual:
                rumbo_seguimiento += np.deg2rad(-20)

            # Sigue fuera
            elif not deteccion_anterior and not deteccion_actual:
                rumbo_seguimiento += np.deg2rad(45)


            x_fin = self.pos_actual[0] + paso * np.cos(rumbo_seguimiento)
            y_fin = self.pos_actual[1] + paso * np.sin(rumbo_seguimiento)
            self.hacer_seccion(self.pos_actual[0], self.pos_actual[1], x_fin, y_fin)

            deteccion_anterior = deteccion_actual

            if len(self.puntos_posidonia[self.indice_posidonia_actual]) > 10:
                dist_inicio = Point(self.pos_actual).distance(punto_inicio)
                # Si la distancia al punto de inicio es menor que el paso se considera el poligono cerrado
                if dist_inicio < paso:
                    rospy.loginfo("Fin contorno posidonia")
                    self.modo_seguimiento = False
                    nube = MultiPoint(self.puntos_posidonia[self.indice_posidonia_actual])
                    poligono_detectado = nube.convex_hull

                    self.clapas_detectadas.append(poligono_detectado)
                    break

    def calcular_area(self):
        # Areas reales
        for i, poligono in enumerate(self.poligonos_posidonia):
            # Se utiliza el area de shapely
            area_real = poligono.area
            rospy.loginfo(f"Area posidonia real {i}: {area_real}")

        # Area poligonos detectados
        for i, puntos_clapa in enumerate(self.puntos_posidonia):
            # Se necesitan mínimo 3 puntos
            if len(puntos_clapa) >= 3:
                # Lista de puntos a un objeto MultiPoint de Shapely
                nube_puntos = MultiPoint(puntos_clapa)

                # Se genera el poligono exterior de los puntos
                poligono_detectado = nube_puntos.convex_hull

                # Calculo de area
                area_calculada = poligono_detectado.area

                rospy.loginfo(f"Area posidonia calculada {i}: {area_calculada}")


if __name__ == '__main__':
    try:
        Navegacion()
    except rospy.ROSInterruptException:
        pass