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
        [3.2558267, 39.3953824],
        [3.2567483, 39.3953824],
        [3.2567483, 39.3948353],
        [3.2558267, 39.3948353],
        [3.2558267, 39.3953824]
    ]

    # Coordenadas de la posidonia
    coordenadas_posidonia = [
    [
        [3.2565615, 39.3949944],
        [3.2566029, 39.3949955],
        [3.2566399, 39.3949962],
        [3.2566758, 39.3949967],
        [3.2567254, 39.3949717],
        [3.2567322, 39.3949208],
        [3.2567234, 39.3949199],
        [3.2567022, 39.3949192],
        [3.2566699, 39.394919],
        [3.2566449, 39.394917],
        [3.2565944, 39.3949256],
        [3.2565385, 39.3949481],
        [3.2565615, 39.3949944]
    ],
    [
        [3.2562312, 39.3948505],
        [3.2562571, 39.3948513],
        [3.2562938, 39.3948416],
        [3.2563594, 39.394855],
        [3.2564016, 39.3948928],
        [3.2564016, 39.3948963],
        [3.2563775, 39.3949321],
        [3.2563327, 39.3949704],
        [3.2562649, 39.3949753],
        [3.2562116, 39.3949558],
        [3.2561608, 39.39492],
        [3.2561804, 39.3948757],
        [3.2562312, 39.3948505]
    ],
    [
        [3.2559327, 39.3950851],
        [3.256, 39.3950865],
        [3.2560525, 39.39505],
        [3.2560708, 39.395003],
        [3.2560468, 39.394956],
        [3.2560254, 39.3949418],
        [3.2560157, 39.3949331],
        [3.2559707, 39.3949006],
        [3.2559117, 39.3949114],
        [3.2558741, 39.3949516],
        [3.2558561, 39.3950057],
        [3.2558911, 39.3950476],
        [3.2559327, 39.3950851]
    ],
    [
        [3.2563711, 39.3953663],
        [3.2564208, 39.3953385],
        [3.2564601, 39.3952923],
        [3.2564345, 39.3952496],
        [3.2563734, 39.3952338],
        [3.2563299, 39.3952453],
        [3.2563146, 39.3952418],
        [3.2562896, 39.3952411],
        [3.2562539, 39.3952408],
        [3.2562197, 39.3952817],
        [3.2562603, 39.3953141],
        [3.2562694, 39.3953211],
        [3.2562834, 39.3953294],
        [3.2562847, 39.3953297],
        [3.2563175, 39.3953485],
        [3.2563711, 39.3953663]
    ]
]
    # Configuración del marcador del entorno
    marker_entorno = Marker()
    marker_entorno.header.frame_id = "world"
    marker_entorno.ns = "entorno_navegacion"
    marker_entorno.id = 1
    marker_entorno.type = Marker.LINE_STRIP # Une los puntos con una línea
    marker_entorno.color.r = 0.0
    marker_entorno.color.g = 0.2
    marker_entorno.color.b = 0.75
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
        marker_posidonia.color.r = 0.18
        marker_posidonia.color.g = 0.45
        marker_posidonia.color.b = 0.20
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