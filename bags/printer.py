import rosbag
import numpy as np
import rospy
import math
import matplotlib.pyplot as plt
from tscf_exploration.msg import takeobjetive
from nav_msgs.msg import OccupancyGrid
import pdb
from copy import deepcopy
from scipy.optimize import curve_fit

contador_0_path_info = 0
contador_1_path_info = 0
contador_0_bid = 0
contador_1_bid = 0
contador_take_obj = 0

def getProMap():
    ret = None
    bag = rosbag.Bag("Exp1/2-Robots/infoGanada/2Robots_IG8.bag")
    for topic, msg, t in bag.read_messages():
        if ((topic == "take_obj") or (topic == "/take_obj")):
            ret = msg.mapa
    return ret

def getFileCosto(i,j, b):
    if b:
        return "Exp2/" + i + "-Robots/info/" + j + "corrida_99.bag"
    else:
        return "Exp2/" + i + "-Robots/costo/" + j + "corrida_99.bag"

def print_coverage(bagse):
    # rospy.init_node('custom_talker', anonymous=True)
    # pub = rospy.Publisher('hola', OccupancyGrid,  queue_size=100)
    # rate = rospy.Rate(10)
    bag = rosbag.Bag(bagse)
    inicio = False
    tiempo_ini = 0
    ultimoMapa = None
    cont = 0
    for topic, msg, t in bag.read_messages():
        if ((topic == "take_obj") or (topic == "/take_obj")):
            cont += 1
            # pub.publish(msg.mapa)
            # rate.sleep()
    tiempo = np.zeros(cont)
    cubrimiento = np.zeros(cont)
    i = -1
    for topic, msg, t in bag.read_messages():
        if ((topic == "take_obj") or (topic == "/take_obj")):
            i += 1
            if (not inicio):
                tiempo_ini = t.to_sec()
                inicio = True
            tiempo[i] = t.to_sec() - tiempo_ini
            cantidad = 0
            for j in msg.mapa.data :
                if not ( j == -1):
                    cantidad += 1
            cubrimiento[i] = cantidad
    bag.close()
    maximo = cubrimiento[cubrimiento.size - 1]
    return tiempo, (cubrimiento * 1/maximo )

def get_path_info_numpy(bag, topico):
    ret1 = ""
    ret2 = ""
    ret3 = ""
    for topic, msg, t in bag.read_messages():
        if (topic == topico):
            info = msg.data.split(" ")
            ret1 = ret1 + " " + info[1]
            ret2 = ret2 + " " + info[2]
            ret3 = ret3 + " " + info[3]
    return np.fromstring(ret1, sep=' '), np.fromstring(ret2, sep=' '), np.fromstring(ret3, sep=' ')

# def print_eficiencia(bag):

def distancia_2_puntos(a, b):
    dx = float(a[0] - b[0])
    dy = float(a[1] - b[1])
    return math.sqrt(math.pow(dx,2) + math.pow(dy,2))

def initialize_map_points(mapa):
    map_points = {}
    width = mapa.info.width
    height = mapa.info.height
    y_origin = mapa.info.origin.position.x
    x_origin = mapa.info.origin.position.y
    for i in range(width*height):
        fila = i % width
        columna = i // width;
        a = float(((x_origin + fila) + 0.5))
        b = float(((y_origin + columna)+ 0.5))
    	map_points[i] = [a,b]
    return map_points

def contCeldas(mapa):
    ret = 0
    for i in mapa.data:
        if (i != -1):
            ret += 1
    return ret

def create_info_experiment(bagse):
    bag = rosbag.Bag(bagse)
    resumen = {}
    # info_take_obj = {}
    # info_objetive = {}
    # info_atrv_move = {0:{}, 1:{}, 2:{}, 3:{}, 4:{}}
    # info_atrv_asgn = {0:{}, 1:{}, 2:{}, 3:{}, 4:{}}
    metrosRec = {}
    metrosRec["atrv0"] = 0.0
    metrosRec["atrv1"] = 0.0
    metrosRec["atrv2"] = 0.0
    metrosRec["atrv3"] = 0.0
    metrosRec["atrv4"] = 0.0
    for topic, msg, t in bag.read_messages():
        if (topic == "/take_obj") or (topic == "take_obj"):
            if not (resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            # print "mapa -> " + str(msg.indice)
            resumen[msg.indice]["mapa"] =  msg.mapa
            resumen[msg.indice]["tiempo_demorado"] = t.to_sec()
        if (topic == "/objetive") or (topic == "objetive"):
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            resumen[msg.indice]["asignaciones"] = 0
            for asign in msg.asignaciones:
                if (not resumen[msg.indice].has_key(asign.idRobot)):
                    resumen[msg.indice][asign.idRobot] = {}
                resumen[msg.indice][asign.idRobot]["objetivo"] = asign.objetivo
                resumen[msg.indice][asign.idRobot]["costo"] = asign.costo
                resumen[msg.indice][asign.idRobot]["info_efectiva"] = asign.info_gain
                resumen[msg.indice]["asignaciones"] += 1
        if (topic == "/atrv0/path_info"):
            inf = msg.data.split(" ")
            metrosRec["atrv0"] = metrosRec["atrv0"] + float(inf[1])
            if not (resumen.has_key(int(inf[3]))):
                print "X1"
                resumen[int(inf[3])] = {}
            if not (resumen[int(inf[3])].has_key("atrv0")):
                print "X2"
                resumen[int(inf[3])]["atrv0"] = {}
            resumen[int(inf[3])]["atrv0"]["metros_recorridos"] = float(inf[1])
        if (topic == "/atrv1/path_info"):
            inf = msg.data.split(" ")
            metrosRec["atrv1"] = metrosRec["atrv1"] + float(inf[1])
            if not (resumen.has_key(int(inf[3]))):
                resumen[int(inf[3])] = {}
            if not (resumen[int(inf[3])].has_key("atrv1")):
                resumen[int(inf[3])]["atrv1"] = {}
            resumen[int(inf[3])]["atrv1"]["metros_recorridos"] = float(inf[1])
        if (topic == "/atrv2/path_info"):
            inf = msg.data.split(" ")
            metrosRec["atrv2"] = metrosRec["atrv2"] + float(inf[1])
            if not (resumen.has_key(int(inf[3]))):
                resumen[int(inf[3])] = {}
            if not (resumen[int(inf[3])].has_key("atrv2")):
                resumen[int(inf[3])]["atrv2"] = {}
            resumen[int(inf[3])]["atrv2"]["metros_recorridos"] = float(inf[1])
        if (topic == "/atrv3/path_info"):
            inf = msg.data.split(" ")
            metrosRec["atrv3"] = metrosRec["atrv3"] + float(inf[1])
            if not (resumen.has_key(int(inf[3]))):
                resumen[int(inf[3])] = {}
            if not (resumen[int(inf[3])].has_key("atrv3")):
                resumen[int(inf[3])]["atrv3"] = {}
            resumen[int(inf[3])]["atrv3"]["metros_recorridos"] = float(inf[1])
        if (topic == "/atrv4/path_info"):
            inf = msg.data.split(" ")
            metrosRec["atrv4"] = metrosRec["atrv4"] + float(inf[1])
            if not (resumen.has_key(int(inf[3]))):
                resumen[int(inf[3])] = {}
            if not (resumen[int(inf[3])].has_key("atrv4")):
                resumen[int(inf[3])]["atrv4"] = {}
            resumen[int(inf[3])]["atrv4"]["metros_recorridos"] = float(inf[1])
        if (topic == "/atrv0/bid"):
            aux_costo = 1000
            aux_centro = -1
            for elem in msg.infoCentros:
                if (elem.cost < aux_costo):
                    aux_costo = elem.cost
                    aux_centro = elem.centro
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            if not(resumen[msg.indice].has_key("atrv0")):
                resumen[msg.indice]["atrv0"] = {}
            resumen[msg.indice]["atrv0"]["asign_costo"] = aux_costo
        if (topic == "/atrv1/bid"):
            aux_costo = 1000
            aux_centro = -1
            for elem in msg.infoCentros:
                if (elem.cost < aux_costo):
                    aux_costo = elem.cost
                    aux_centro = elem.centro
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            if not(resumen[msg.indice].has_key("atrv1")):
                resumen[msg.indice]["atrv1"] = {}
            resumen[msg.indice]["atrv1"]["asign_costo"] = aux_costo
        if (topic == "/atrv2/bid"):
            aux_costo = 1000
            aux_centro = -1
            for elem in msg.infoCentros:
                if (elem.cost < aux_costo):
                    aux_costo = elem.cost
                    aux_centro = elem.centro
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            if not(resumen[msg.indice].has_key("atrv2")):
                resumen[msg.indice]["atrv2"] = {}
            resumen[msg.indice]["atrv2"]["asign_costo"] = aux_costo
        if (topic == "/atrv3/bid"):
            aux_costo = 1000
            aux_centro = -1
            for elem in msg.infoCentros:
                if (elem.cost < aux_costo):
                    aux_costo = elem.cost
                    aux_centro = elem.centro
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            if not(resumen[msg.indice].has_key("atrv3")):
                resumen[msg.indice]["atrv3"] = {}
            resumen[msg.indice]["atrv3"]["asign_costo"] = aux_costo
        if (topic == "/atrv4/bid"):
            aux_costo = 1000
            aux_centro = -1
            for elem in msg.infoCentros:
                if (elem.cost < aux_costo):
                    aux_costo = elem.cost
                    aux_centro = elem.centro
            if not(resumen.has_key(msg.indice)):
                resumen[msg.indice] = {}
            if not(resumen[msg.indice].has_key("atrv4")):
                resumen[msg.indice]["atrv4"] = {}
            resumen[msg.indice]["atrv4"]["asign_costo"] = aux_costo
    bag.close()
    return resumen

def get_range_cells(celda, width):
    range_cells = []
    for j in  range(-8*width, 9*width, width):
        for i in range(-8, 9, 1):
            range_cells.insert(0, celda+i+j)
    return range_cells

def get_vecinos(celda, width):
    range_cells = []
    for j in  range(-1*width, 2*width, width):
        for i in range(-1, 2, 1):
            range_cells.insert(0, celda+i+j)
    return range_cells

def es_vecino_de_set(celda, vecinos, width):
    es = False
    for v in vecinos:
        lis = get_vecinos(v, width)
        if (celda in lis):
            es = True
            break
    return es

def get_info_estimated(mapa, objetivo, map_points):
    obstaculos = {}
    info_ganada = []
    nivel = []
    sig_nivel = []
    visitadas = []
    sig_nivel.insert(0,objetivo)
    while(sig_nivel != []):
        nivel = []
        nivel = sig_nivel
        sig_nivel = []
        for actual in nivel:
            for posVecino in get_vecinos(actual, mapa.info.width):
                if (not posVecino in visitadas):
                    visitadas.insert(0,posVecino)
                    # aux = mapa2.data[posVecino]
                    # mapa2.data[posVecino]=100
                    # pub2.publish(mapa2)
                    # mapa2.data[posVecino]=aux
                    if (mapa.data[posVecino] == 100):
                        llave = posVecino
                        for key, value in obstaculos.items():
                            if es_vecino_de_set(posVecino, value[0], mapa.info.width):
                                llave = key
                                break
                        if (llave == posVecino):
                            obstaculos[llave] = [[posVecino], []]
                        else:
                            obstaculos[llave][0].insert(0, posVecino)
                    else:
                        llave = -1
                        for key, value in obstaculos.items():
                            if es_vecino_de_set(posVecino, value[1], mapa.info.width):
                                llave = key
                                break
                        if (llave != -1):
                            puedo = True
                            for elem in obstaculos[llave][0]:
                                if not((mapa.data[posVecino] == -1) and  (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0) and (distancia_pto_recta(objetivo, posVecino, elem, map_points) > (1/math.sqrt(2)) )):
                                    puedo = False
                                    break
                            if puedo:
                                info_ganada.insert(0, posVecino)
                                sig_nivel.insert(0,posVecino)
                            else:
                                obstaculos[llave][1].insert(0, posVecino)
                        else:
                            llave = -1
                            for key, value in obstaculos.items():
                                if es_vecino_de_set(posVecino, value[0], mapa.info.width):
                                    llave = key
                                    break
                            if (llave != -1):
                                puedo = True
                                for elem in obstaculos[llave][0]:
                                    if not((mapa.data[posVecino] == -1) and  (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0) and (distancia_pto_recta(objetivo, posVecino, elem, map_points) >(1/math.sqrt(2)))):
                                        puedo = False
                                        break
                                if puedo:
                                    info_ganada.insert(0, posVecino)
                                    sig_nivel.insert(0,posVecino)
                                else:
                                    obstaculos[llave][1].insert(0, posVecino)
                            else:
                                if (mapa.data[posVecino] == -1) and (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0):
                                    info_ganada.insert(0, posVecino)
                                    sig_nivel.insert(0,posVecino)
    filt = []
    for h in info_ganada:
        if not (h in filt):
            # mapa2.data[h]=100
            filt.insert(0,h)
    # pub2.publish(mapa2)
    return filt

def get_obstaculos(mapa, listaCeldas):
    ret = []
    for i in listaCeldas:
        if (mapa.data[i] == 100):
            ret.insert(0, i)
    return ret

def distancia_pto_recta(objetivo,celda,o, map_points):
    dist = abs(map_points[objetivo][0] - map_points[o][0])
    if (map_points[objetivo][0] != map_points[celda][0]):
        r = float(map_points[objetivo][1] - map_points[celda][1])
        k = float(map_points[objetivo][0] - map_points[celda][0])
        t = float(map_points[o][0] - map_points[o][1])
        z = float(map_points[objetivo][1] - map_points[celda][1])
        h = float(map_points[objetivo][0] - map_points[celda][0])
        dist = abs( ( r / k ) * map_points[o][0] -  map_points[o][1] + (map_points[celda][1] - ( z / h ) * map_points[celda][0] ) )
        dist = dist / math.sqrt(math.pow(( z / h ), 2) + 1)
    return dist

def veo_celda(mapa, listaObst, celda, objetivo, map_points):
    ret = True
    dist = float(0.708)
    for o in listaObst:
        if (celda != o) and (celda != objetivo):
            if (distancia_pto_recta(objetivo,celda,o, map_points) > dist):
                ret = ret and True
            else:
                ret = False
                break
    return ret

def get_info_gain(mapa, objetivo, map_points, listaEstimada):
    obstaculos = {}
    info_ganada = []
    nivel = []
    sig_nivel = []
    visitadas = []
    mapaEfector = OccupancyGrid()
    mapaEfector.header = mapa.header
    mapaEfector.info = mapa.info
    mapaEfector.data = list(mapa.data)
    mapaEfector.data[objetivo] = 100

    sig_nivel.insert(0,objetivo)
    while(sig_nivel != []):
        nivel = []
        nivel = sig_nivel
        sig_nivel = []
        for actual in nivel:
            for posVecino in get_vecinos(actual, mapa.info.width):
                if (not posVecino in visitadas):
                    visitadas.insert(0,posVecino)
                    if (mapa.data[posVecino] == 100):
                        llave = posVecino
                        for key, value in obstaculos.items():
                            if es_vecino_de_set(posVecino, value[0], mapa.info.width):
                                llave = key
                                break
                        if (llave == posVecino):
                            obstaculos[llave] = [[posVecino], []]
                        else:
                            obstaculos[llave][0].insert(0, posVecino)
                    else:
                        llave = -1
                        for key, value in obstaculos.items():
                            if es_vecino_de_set(posVecino, value[1], mapa.info.width):
                                llave = key
                                break
                        if (llave != -1):
                            puedo = True
                            for elem in obstaculos[llave][0]:
                                if not((mapa.data[posVecino] != -1) and  (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0) and (distancia_pto_recta(objetivo, posVecino, elem, map_points) > (1/math.sqrt(2)) )):
                                    puedo = False
                                    break
                            if puedo:
                                info_ganada.insert(0, posVecino)
                                sig_nivel.insert(0,posVecino)
                            else:
                                obstaculos[llave][1].insert(0, posVecino)
                        else:
                            llave = -1
                            for key, value in obstaculos.items():
                                if es_vecino_de_set(posVecino, value[0], mapa.info.width):
                                    llave = key
                                    break
                            if (llave != -1):
                                puedo = True
                                for elem in obstaculos[llave][0]:
                                    if not((mapa.data[posVecino] != -1) and  (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0) and (distancia_pto_recta(objetivo, posVecino, elem, map_points) >(1/math.sqrt(2)))):
                                        puedo = False
                                        break
                                if puedo:
                                    info_ganada.insert(0, posVecino)
                                    sig_nivel.insert(0,posVecino)
                                else:
                                    obstaculos[llave][1].insert(0, posVecino)
                            else:
                                if (mapa.data[posVecino] != -1) and (distancia_2_puntos(map_points[objetivo], map_points[posVecino]) <= 6.0):
                                    info_ganada.insert(0, posVecino)
                                    sig_nivel.insert(0,posVecino)
    filt = 0
    for h in listaEstimada:
        if (h in info_ganada):
            mapaEfector.data[h] = 100
            filt += 1
    # pub.publish(mapaEfector)
    return filt

def buscarTiempoTotal(dicc):
    tiempo_inicial = dicc[0]["tiempo_demorado"]
    tiempo_total = 0
    for k in dicc:
        tiempo_total = dicc[k]["tiempo_demorado"] - tiempo_inicial
    return tiempo_total

def buscarMetrajeTotal(dicc, j):
    metraje0 = 0
    metraje1 = 0
    metraje2 = 0
    metraje3 = 0
    metraje4 = 0
    for k in dicc:
        if (dicc[k].has_key("atrv0") and (dicc[k]["atrv0"].has_key("metros_recorridos")) and (dicc[k]["atrv0"]["metros_recorridos"] != 0)):
            metraje0 = dicc[k]["atrv0"]["metros_recorridos"]
        if (dicc[k].has_key("atrv1") and (dicc[k]["atrv1"].has_key("metros_recorridos"))and (dicc[k]["atrv1"]["metros_recorridos"] != 0)):
            metraje1 = dicc[k]["atrv1"]["metros_recorridos"]
        if (dicc[k].has_key("atrv2") and (dicc[k]["atrv2"].has_key("metros_recorridos"))and (dicc[k]["atrv2"]["metros_recorridos"] != 0)):
            metraje2 = dicc[k]["atrv2"]["metros_recorridos"]
        if (dicc[k].has_key("atrv3") and (dicc[k]["atrv3"].has_key("metros_recorridos"))and (dicc[k]["atrv3"]["metros_recorridos"] != 0)):
            metraje3 = dicc[k]["atrv3"]["metros_recorridos"]
        if (dicc[k].has_key("atrv4") and (dicc[k]["atrv4"].has_key("metros_recorridos"))and (dicc[k]["atrv4"]["metros_recorridos"] != 0)):
            metraje4 = dicc[k]["atrv4"]["metros_recorridos"]
    return metraje0 + metraje1 + metraje2 + metraje3 + metraje4

def tiempoMinimo(dicc, j):
    minimo = 100000
    indice = -1
    for key in dicc:
        aux = buscarMetrajeTotal(dicc[key], j)
        if (aux < minimo):
            indice = key
            minimo = aux
    print minimo
    return indice

def tiempoMaximo(dicc, j):
    maximo = 0
    indice = -1
    for key in dicc:
        aux = buscarMetrajeTotal(dicc[key], j)
        if (aux > maximo):
            indice = key
            maximo = aux
    print maximo
    return indice

def getTiempoExp(dicc):
    vector = np.zeros(len(dicc))
    cont = 0
    for i in dicc:
        vector[cont] = buscarTiempoTotal(dicc[i])
        cont += 1
    print vector
    return vector.mean(), vector.std()

def getMetrajeExp(dicc, j):
    vector = np.zeros(len(dicc))
    cont = 0
    for i in dicc:
        vector[cont] = buscarMetrajeTotal(dicc[i], j)
        cont += 1
    print vector
    return vector.mean(), vector.std()

def getEficienciaExp(dicc, j):
    vector = np.zeros(len(dicc))
    cont = 0
    for i in dicc:

        vector[cont] = buscarMetrajeTotal(dicc[i], j) / buscarTiempoTotal(dicc[i])
        cont += 1
    return vector.mean(), vector.std()

# rospy.init_node('custom_talker', anonymous=True)
# pub1 = rospy.Publisher('estimada', OccupancyGrid, queue_size=10)
# pub2 = rospy.Publisher('ganada', OccupancyGrid, queue_size=10)
info_bags = {2:{"costo":{}, "infoG":{}}, 3:{"costo":{}, "infoG":{}} ,4:{"costo":{}, "infoG":{}}, 5:{"costo":{}, "infoG":{}}}
mapa = getProMap()
maximoCubrimiento = contCeldas(mapa)
print "Maximo Cubrimiento " + str(maximoCubrimiento)
mapoin = initialize_map_points(mapa)
# rate = rospy.Rate(1) # 10hz
# for j in range(3):
j = 0
for i in range(10):
    filex = getFileCosto(str(j+2),str(i+1), True)
    print filex
    resumen = create_info_experiment(filex)
    for key in resumen:
        resumen[key]["cubrimiento"] = contCeldas(resumen[key]["mapa"])/float(maximoCubrimiento)
        if (resumen[key].has_key("asignaciones")) and (resumen[key]["asignaciones"] != 0):
            for k in range(j+2):
                if (resumen[key].has_key("atrv" + str(k))) and (resumen[key]["atrv" + str(k)].has_key("objetivo")):
                    mapaEstimador = OccupancyGrid()
                    mapaEstimador.header = resumen[key]["mapa"].header
                    mapaEstimador.info = resumen[key]["mapa"].info
                    mapaEstimador.data = list(resumen[key]["mapa"].data)
                    mapaEstimador.data[resumen[key]["atrv" + str(k)]["objetivo"]] = 100
                    estimada = get_info_estimated(resumen[key]["mapa"], resumen[key]["atrv" + str(k)]["objetivo"], mapoin)
                    resumen[key]["atrv" + str(k)]["info_estimada"] = len(estimada)
                    for q in estimada:
                        mapaEstimador.data[q] = 100
                    # pub1.publish(mapaEstimador)
                    resumen[key]["atrv" + str(k)]["info_efectiva"] = get_info_gain(mapa, resumen[key]["atrv" + str(k)]["objetivo"], mapoin, estimada)
                    # rate.sleep()

    borrar = False
    a_sacar = []
    for key in resumen:
        if borrar:
            a_sacar.insert(0,key)
        else:
            if not resumen[key].has_key("asignaciones"):
                a_sacar.insert(0,key)
        if resumen[key]["cubrimiento"] >= 0.99:
            borrar = True
    print "saco -> " + str(len(a_sacar))
    for h in a_sacar:
        del resumen[h]

    info_bags[j+2]["infoG"][i+1] = resumen
    resumen2 = create_info_experiment(getFileCosto(str(j+2),str(i+1), False))
    for key in resumen2:
        resumen2[key]["cubrimiento"] = contCeldas(resumen2[key]["mapa"])/float(maximoCubrimiento)
    borrar = False
    a_sacar = []
    for key in resumen2:
        if borrar:
            a_sacar.insert(0,key)
        if resumen2[key]["cubrimiento"] >= 0.99:
            borrar = True
    info_bags[j+2]["costo"][i+1] = resumen2
#
# time_means = {"costo": np.zeros(4), "infoG": np.zeros(4)}
# time_means_err = {"costo": np.zeros(4), "infoG": np.zeros(4)}
# metraje_means = {"costo": np.zeros(4), "infoG": np.zeros(4)}
# metraje_means_err = {"costo": np.zeros(4), "infoG": np.zeros(4)}
# eficiencia_means = {"costo": np.zeros(4), "infoG": np.zeros(4)}
# eficiencia_means_err = {"costo": np.zeros(4), "infoG": np.zeros(4)}
#
# for j in range(4):
#     for h in time_means:
#         # print " "
#         # print " J -> " + str(j) + " " + str(h) + " tiempo"
#         time_means[h][j] , time_means_err[h][j] = getTiempoExp(info_bags[j+2][h])
#         # print time_means[h][j]
#         # print " "
#         # print " J -> " + str(j) + " " + str(h) + " metraje_means"
#         metraje_means[h][j] , metraje_means_err[h][j] = getMetrajeExp(info_bags[j+2][h], j)
#         # print metraje_means[h][j]
#         # print " "
#         # print " J -> " + str(j) + " " + str(h) + " eficiencia_means"
#         eficiencia_means[h][j] , eficiencia_means_err[h][j] = getEficienciaExp(info_bags[j+2][h], j)
#         # print eficiencia_means[h][j]

def getcoefs(dictio):
    coef = {}
    efect = {}
    for k in range(j+2):
        coef["atrv" + str(k)] = {}
        coef["total"] = {}
        efect["atrv" + str(k)] = {}
    for key in dictio:
        info_efectiva = 0
        info_estimada = 0
        for k in range(j+2):
            coef["atrv" + str(k)][key] = 0.0
            efect["atrv" + str(k)][key] = 0.0
            if (dictio[key].has_key("atrv" + str(k))) and (dictio[key]["atrv" + str(k)].has_key("info_estimada")) and (dictio[key]["atrv" + str(k)].has_key("info_efectiva")):
                if ((dictio[key]["atrv" + str(k)]["info_estimada"]) != 0):
                    coef["atrv" + str(k)][key] = dictio[key]["atrv" + str(k)]["info_estimada"] - dictio[key]["atrv" + str(k)]["info_efectiva"]
                    efect["atrv" + str(k)][key] = dictio[key]["atrv" + str(k)]["info_estimada"]
                    info_efectiva += dictio[key]["atrv" + str(k)]["info_efectiva"]
                    info_estimada += dictio[key]["atrv" + str(k)]["info_estimada"]
        coef["total"][key] = 0.0
        if (info_estimada != 0):
            coef["total"][key] = float(float(info_estimada) - info_efectiva)

    print "largo de coef -> " + str(len(coef["total"]))
    return coef, efect

def getLongNorm(dictio):
    minimo = 100000
    for i in range(10):
        aux = len(dictio[i]["total"])
        if (aux < minimo):
            minimo = aux
    return minimo

def normalizar(coefs, largoMin):
    ret = {}
    for i in range(10):
        ret[i] = {}
        for key in coefs[i]:
            ret[i][key] = np.zeros(largoMin)
        largo = len(coefs[i]["total"])
        sacar = largo - largoMin
        if (sacar == 0) :
            distancia = 0
        else:
            distancia = largo/sacar
        cont = 0
        cont_lar = 0
        for p in coefs[i]["total"]:
            cont_lar += 1
            if sacar != 0 and ((cont_lar) % distancia) == 0:
                sacar -= 1
            else:
                for key in coefs[i]:
                    ret[i][key][cont] = coefs[i][key][p]
                cont += 1

        print str(i) + " -> largo = " + str(largo) + "entonces saco -> " + str(sacar) + " cada -> " + str(distancia)
    return ret


def getNormalized(dictio):
    coefs = range(10)
    for i in range(10):
        print "largo esperado -> " + str(len(info_bags[j+2]["infoG"][i+1]))
        coefs[i] = getcoefs(dictio[i+1])
    largoMin = getLongNorm(coefs)
    print largoMin
    ret = normalizar(coefs, largoMin)
    return ret


def func(x, A, B, C):
    """Modelo para nuestros datos."""
    return A * np.exp(-B * x ** 2) + C

coefs = range(10)
efect = range(10)
j = 0
suubs = {"total": 313, "atrv0": 211,"atrv1": 212,"atrv2": 613,"atrv3": 614, "atrv4": 615}
for e in range(1):
    plt.figure(e+1)
    plt.title("Experimento " + str(e) + " con " + str(j+2) + " robots")
    coefs[e], efect[e] = getcoefs(info_bags[j+2]["infoG"][e+1])
    for r in coefs[e]:
        if r != "total":
            division = np.zeros(len(coefs[e]['total']))
            estima = np.zeros(len(coefs[e]['total']))
            asig = np.arange(len(coefs[e]['total']))
            cincos = np.ones(len(coefs[e]['total'])) * 5
            cs = np.ones(len(coefs[e]['total'])) * 25
            cont = 0
            cantidad_err_med = 0
            cantidad_div_med = 0
            # pdb.set_trace()
            for h in coefs[e][r]:
                division[cont] = coefs[e][r][h]
                if r != "total":
                    estima[cont] = efect[e][r][h]
                cont += 1
            z = np.polyfit(asig, division, 2)
            p = np.poly1d(z)
            plt.subplot(suubs[r])

            # errMax = division.max()
            # errMin = division.min()
            # vect = np.ones(len(coefs[e]['total']))
            # abajo = errMax - errMin
            # vect = vect * errMin
            # division = division - vect
            # division = division / abajo
            plt.title(r)
            lab = "Error con Media: " + str(division.mean()) + " Max: " + str(division.max()) + " Min: " +str(division.min())
            #lab2 = "Info. estimada con Media: " + str(estima.mean()) + " Max: " + str(estima.max()) + " Min: " +str(estima.min())
            plt.plot(asig,division,'.', label=lab)
            plt.plot(asig,p(asig), label="Aprox. pol gr.2")

            plt.xlabel('Nro. asigancion')
            # plt.ylabel('Error normalizado (Informacion efectivamente ganada - Informacion estimada a ganar).')
            plt.legend()
    plt.show()
# pdb.set_trace()
#
#
# normalizados = {}
# for j in range(4):
#     normalizados[j+2] = getNormalized(info_bags[j+2]["infoG"])
#     i = normalizados[j+2][0]["atrv0"].size
#     totales = np.zeros(i)
#     atrv = [[],[],[],[],[]]
#     atrv_err = [[],[],[],[],[]]
#     for cant in range(j+2):
#         atrv[cant] = np.zeros(i)
#         atrv_err[cant] = np.zeros(i)
#     err = np.zeros(i)
#
#     ind = 0
#     for key in range(i):
#         aux = np.zeros(10)
#         atrv_aux = [1,2,3,4,5,6,7,8,9,0]
#         for cant in range(j+2):
#             atrv_aux[cant] = np.zeros(10)
#         for h in range(10):
#             aux[h] = normalizados[j+2][h]["total"][key]
#             for cant in range(j+2):
#                 atrv_aux[cant][h] = normalizados[j+2][h]["atrv" + str(cant)][key]
#
#         totales[ind] = aux.mean()
#         for cant in range(j+2):
#             atrv[cant][ind] = atrv_aux[cant].mean()
#             atrv_err[cant][ind] = atrv_aux[cant].std()
#         err[ind] = aux.std()
#         ind += 1
#     fig, ax = plt.subplots()
#     asign = np.arange(i)
#     for cant in range(j+2):
#         #ax.errorbar(asign,atrv[cant],yerr=atrv_err[cant], label="atrv" + str(cant))
#         ax.errorbar(asign,atrv[cant], label="atrv" + str(cant))
#     #ax.errorbar(asign,totales,yerr=err, label='total')
#     ax.errorbar(asign,totales, label='total')
#     ax.set_title("Experimento con " + str(j+2) + " robots")
#     ax.set_xlabel('Nro. asigancion')
#     ax.set_ylabel('Informacion efectivamente ganada / Informacion estimada a ganar')
#     ax.legend()
# plt.show()











#
# N = 4
#
# ind = np.arange(N)
# ind2 = ind + 0.175*np.ones(N)    # the x locations for the groups
# width = 0.35       # the width of the bars: can also be len(x) sequence
#
# plt.subplot(311)
# p1 = plt.bar(ind, time_means["costo"], width, yerr=time_means_err["costo"])
# p2 = plt.bar(ind2, time_means["infoG"], width, yerr=time_means_err["infoG"], align='edge')
#
# plt.ylabel('Tiempo (s)')
# plt.xlabel('Cantidad de robots')
# plt.xticks(ind2, ('2', '3', '4'))
# plt.legend((p1[0], p2[0]), ('Utilizando estimador de Costo', 'Utilizando estimador de Informacion Ganada'))
#
# # log
# plt.subplot(312)
# p1 = plt.bar(ind, metraje_means["costo"], width, yerr=metraje_means_err["costo"])
# p2 = plt.bar(ind2, metraje_means["infoG"], width, yerr=metraje_means_err["infoG"], align='edge')
#
# plt.ylabel('Metros del sistema (m) ')
# plt.xlabel('Cantidad de robots')
# plt.xticks(ind2, ('2', '3', '4'))
# plt.legend((p1[0], p2[0]), ('Utilizando estimador de Costo', 'Utilizando estimador de Informacion Ganada'))
#
# # symmetric log
# plt.subplot(313)
# p1 = plt.bar(ind, eficiencia_means["costo"], width, yerr=eficiencia_means_err["costo"])
# p2 = plt.bar(ind2, eficiencia_means["infoG"], width, yerr=eficiencia_means_err["infoG"], align='edge')
#
# plt.ylabel('Eficiencia (m/s)')
# plt.xlabel('Cantidad de robots')
# plt.xticks(ind2, ('2', '3', '4', '5'))
# plt.legend((p1[0], p2[0]), ('Utilizando estimador de Costo', 'Utilizando estimador de Informacion Ganada'))
#
# plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.5,wspace=0.5)
#
# plt.show()


# tiempo = {}
# for j in range(4):
#     for i in range(10):metros_recorridos

#         tiempo[j+2][costo][i]=info_bags[j+2][costo][i][]


# rospy.init_node('custom_talker', anonymous=True)
# pub = rospy.Publisher('/hola', OccupancyGrid,  queue_size=100)
# pub2 = rospy.Publisher('/hola2', OccupancyGrid,  queue_size=100)
# rate = rospy.Rate(3)
# rate.sleep()
# for item in info_bag[0].items():
#     pub.publish(item[1].mapa)
#     rate.sleep()


# item = info_bag[1].items()[0]
# mapa = info_bag[0][item[0]].mapa
# mapa2 = OccupancyGrid()
# mapa2.header = mapa.header
# mapa2.info = mapa.info
# mapa2.data = list(mapa.data)
# pub.publish(mapa)tama[i]
# map_poin = initialize_map_points(mapa)
# rate.sleep()
# print len(item[1].asignaciones)
# for elem in item[1].asignaciones:
#     print " Robot -> " + str(elem.idRobot) + " Celda -> " + str(elem.objetivo) + " Costo -> " + str(elem.costo) + " InfoGain -> " + str(elem.info_gain)
#     lis = get_info_estimated(mapa, mapa2, elem.objetivo, map_poin, pub2, rate)
#     # for j in lis:
#     #     mapa2.data[j] = 100
#     # pub2.publish(mapa2)
#     # rate.sleep()
#     print "Corroboro -> " + str(len(lis))

# infoGanada = []tama[i]
# costoG = []
# infoGanada.insert(0, print_coverage("2-Robots/infoGanada/2Robots_IG1.bag"))
# costoG.insert(0, print_coverage("2-Robots/costo/2Robots_Costo1.bag"))
# for i in range(1):
#     plt.figure(i)
#     plt.title("Experimento " + str(i+1))
#     plt.plot(infoGanada[i][0],infoGanada[i][1], label="Exp. IG " + str(i))
#     plt.plot(costoG[i][0],costoG[i][1], label="Exp. C " + str(i))
#     plt.legend()
#     plt.xlabel('Tiempo')
#     plt.ylabel('Cubrimiento')
#
# plt.show()


get_range_cells(8,10)


#print str(info_path_0[0])
# for topic, msg, t in bag.read_messages():
#     if (not topic in topicos):
#         topicos.insert(0,topic)
#         print topic
#     if (topic == "/atrv0/path_info"):
#         contador_0_path_info += 1
#     if (topic == "/atrv1/path_info"):
#         contador_1_path_info += 1
#     if (topic == "/atrv0/bid"):
#         contador_0_bid += 1
#     if (topic == "/atrv1/bid"):
#         contador_1_bid += 1
#     if (topic == "/take_obj"):
#         contador_take_obj += 1
#
# print " contador_0_path_info " + str(contador_0_path_info)
# print " contador_1_path_info " + str(contador_1_path_info)
# print " contador_0_bid " + str(contador_0_bid)
# print " contador_1_bid " + str(contador_1_bid)
# print " contador_take_obj " + str(contador_take_obj)
