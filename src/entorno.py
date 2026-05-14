#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cola2.utils.ned import NED 

def main():
    rospy.init_node('entorno')
    
    # Publicador para los markers
    publicador_marker = rospy.Publisher('visualizacion_entorno', Marker, queue_size=10)

    # Origen en navigatior NED origin
    origin_lat = rospy.get_param('/sparus2/navigator/ned_latitude')
    origin_lon = rospy.get_param('/sparus2/navigator/ned_longitude')
    ned = NED(origin_lat, origin_lon, 0)

    # Coordenadas del rectangulo
    coordenadas_entorno = [
        [3.230217044521737, 39.360785383636426],
        [3.230217044521737, 39.36056497880358],
        [3.2305964558431697, 39.36056497880358],
        [3.2305964558431697, 39.360785383636426],
        [3.230217044521737, 39.360785383636426]
    ]

    # Coordenadas de la posidonia
    coordenadas_posidonia = [
        [3.23038999186997, 39.36074932820392],
        [3.230363067900157, 39.360745634907744],
        [3.230304443128972, 39.360678148287974],
        [3.2303587253248622, 39.36061469066013],
        [3.230423863959544, 39.36059723140863],
        [3.2304790146702658, 39.36059857442851],
        [3.2305167950788416, 39.360650616414404],
        [3.230476843383201, 39.3607325404943],
        [3.23038999186997, 39.36074932820392]
    ]

    # Configuración del marcador del entorno
    marker_entorno = Marker()
    marker_entorno.header.frame_id = "world"
    marker_entorno.ns = "entorno_navegacion"
    marker_entorno.id = 1
    marker_entorno.type = Marker.LINE_STRIP # Une los puntos con una línea
    marker_entorno.color.b = 1  # Azul
    marker_entorno.color.a = 1  # Opaco
    marker_entorno.scale.x = 0.2 # Ancho de línea

    coordenadas_entorno_ned = [] # Variable para poder compartir las coordenadas ya convertidas al python de exploration_strategy

    for lon, lat in coordenadas_entorno:
        n, e, d = ned.geodetic2ned([lat, lon, 2.0])
        p = Point()
        p.x, p.y, p.z = n, e, d 
        marker_entorno.points.append(p)
        coordenadas_entorno_ned.append([float(n), float(e)])

    
    rospy.set_param('/entorno/entorno_ned', coordenadas_entorno_ned)

    # Configuración del marcador de la posidonia
    marker_posidonia = Marker()
    marker_posidonia.header.frame_id = "world"
    marker_posidonia.ns = "clapa_posidonia"
    marker_posidonia.id = 2
    marker_posidonia.type = Marker.LINE_STRIP
    marker_posidonia.color.g = 1  # Verde
    marker_posidonia.color.a = 1  # Opaco
    marker_posidonia.scale.x = 0.2 # Ancho de línea


    for lon, lat in coordenadas_posidonia:
        n, e, d = ned.geodetic2ned([lat, lon, 2.0])
        p = Point()
        p.x, p.y, p.z = n, e, d
        marker_posidonia.points.append(p)

    rate = rospy.Rate(1) # Publicar una vez por segundo


    while not rospy.is_shutdown():
        publicador_marker.publish(marker_entorno)
        publicador_marker.publish(marker_posidonia)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass