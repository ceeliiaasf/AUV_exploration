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
        [3.2200361553293533, 39.35651169177703],
        [3.2200361553293533, 39.35621403437702],
        [3.220549435111792, 39.35621403437702],
        [3.220549435111792, 39.35651169177703],
        [3.2200361553293533, 39.35651169177703]
    ]

    # Coordenadas de la posidonia
    coordenadas_posidonia = [
        [
            [3.2200720343402907, 39.356448730930765],
            [3.220106646957049, 39.35637823317583],
            [3.220208796387567, 39.356382802476986],
            [3.2202501626855167, 39.356430453741694],
            [3.220223147960894, 39.35648202151151],
            [3.220115933268886, 39.35648789631887],
            [3.2200720343402907, 39.356448730930765]
        ],
        [
            [3.2203936784140126, 39.356488549074925],
            [3.220413939458581, 39.35642196790292],
            [3.220511867837587, 39.356398468650156],
            [3.220484853112964, 39.356477452217774],
            [3.2203936784140126, 39.356488549074925]
        ],
        [
            [3.2201079131936012, 39.356293018152286],
            [3.220139148969821, 39.35623231162185],
            [3.2202185047261764, 39.35623165886341],
            [3.2202573383933952, 39.35628714332944],
            [3.2201762942173104, 39.35631978123041],
            [3.2201079131936012, 39.356293018152286]
        ]
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
    markers_posidonia = []

    coordenadas_posidonia_ned = [] # Variable para poder compartir las coordenadas ya convertidas al python de exploration_strategy
    for i, poligono in enumerate(coordenadas_posidonia):
        marker_posidonia = Marker()
        marker_posidonia.header.frame_id = "world"
        marker_posidonia.ns = "clapa_posidonia"
        marker_posidonia.id = i
        marker_posidonia.type = Marker.LINE_STRIP
        marker_posidonia.action = Marker.ADD
        marker_posidonia.color.g = 1  # Verde
        marker_posidonia.color.a = 1  # Opaco
        marker_posidonia.scale.x = 0.2 # Ancho de línea
        
        poligono_ned = []

        for lon, lat in poligono:
            n, e, d = ned.geodetic2ned([lat, lon, 2.0])

            p = Point()
            p.x, p.y, p.z = n, e, d
            marker_posidonia.points.append(p)

            poligono_ned.append([float(n), float(e)])
            
        markers_posidonia.append(marker_posidonia)
        coordenadas_posidonia_ned.append(poligono_ned)


    rospy.set_param('/entorno/posidonia_ned', coordenadas_posidonia_ned)

    rate = rospy.Rate(1) # Publicar una vez por segundo

    while not rospy.is_shutdown():
        publicador_marker.publish(marker_entorno)
        for marker in markers_posidonia:
            publicador_marker.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass