#!/usr/bin/python

# bb_vfh.py

# Desc: Minimal config for path planning
# Author: Alexandre Amory
# Date: 10/19/2014
# to run player server - "player bb_goto.cfg" in one window
# to run controller - "python bb_goto.py" in another window

import sys,os,math
import time
import cv2
import cv as cv
import numpy as np
import implementation as a_star
# this should be whereever "playerc.py" is.  
# On linux, you can find this out with "locate playerc.py"
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
from DStarLiteJava import DStarLite
from copy import deepcopy
from playercpp import *


##### GRAUS PARA RADIANDOS #####
def dtor (deg):
    return deg*math.pi/180.0;
def Pos_mapToImg (posMap, mapa, horizontal) :
    if horizontal :
        return (mapa.GetHeight() / 2) + (posMap * mapa.GetHeight() / 15)
    return mapa.GetWidth() - ((mapa.GetWidth() / 2) + (posMap * mapa.GetWidth() / 15))

def Pos_imgToMap (posImg, mapa, horizontal) :
    if horizontal :
        return ((posImg - mapa.GetHeight() / 2.0) * 15.0 / mapa.GetHeight())
    return ((posImg - mapa.GetHeight() / 2.0) * 15.0 / mapa.GetWidth())

def ConvertMtsToPxls (mts, mapa, horizontal) :
    if horizontal :
        return (mapa.GetHeight() / 2) + ((-7.5 + mts) * mapa.GetHeight() / 15)
    return mapa.GetWidth() - ((mapa.GetWidth() / 2) + ((+7.5 - mts) * mapa.GetWidth() / 15))


def imprimeP(x, y, img)  :
    img[ y + 0, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 3] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 1, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 1, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 2, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 2, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 3, x + 3] = 64 # set the colour  , 0, 0


def imprimeR(x, y, img)  :
    img[ y + 0, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 3] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 1, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 1, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 2, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 2, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 2, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 3, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 3, x + 3] = 64 # set the colour  , 0, 0

def imprimeO(x, y, img)  :
    img[ y + 0, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 3] = 64 # set the colour  , 0, 0

    img[ y + 1, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 1, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 2, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 2, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 3, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 3, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 3, x + 3] = 64 # set the colour  , 0, 0

def imprimeC(x, y, img)  :
    img[ y + 0, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 2] = 64 # set the colour  , 0, 0
    img[ y + 0, x + 3] = 64 # set the colour  , 0, 0

    img[ y + 1, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 1, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 2, x + 0] = 64 # set the colour  , 0, 0
    img[ y + 2, x + 4] = 64 # set the colour  , 0, 0

    img[ y + 3, x + 1] = 64 # set the colour  , 0, 0
    img[ y + 3, x + 3] = 64 # set the colour  , 0, 0


##### EROSAO #####
def erode(erosion_size, img):
    #erosion_size = 2*erosion_size+1
    arredondamento = int(math.ceil(erosion_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(arredondamento,arredondamento))
    eroded = cv2.erode(img,kernel)
    return eroded;

##### IMAGEM #####
def geraImagem(mapa, position, goal):

    img = np.zeros((mapa.GetHeight(),mapa.GetWidth(),1), np.uint8)


    x = 0
    y = 0
    fator = 0
    celula = 0
    obstaculos = []
    while ( x < mapa.GetHeight() ):
        while ( y < mapa.GetWidth() ):
            celula = mapa.GetCell ((int)(y)-fator, mapa.GetHeight() - (int)( x))
            if (celula == -1):
                img[ x, y] = 255 # set the blank colour onde nao tem nada, 255, 255
            else :
                if (celula == 1):
                    img[x, y] = 0 # set the black colour onde tem borda, 0, 0
                    
                else :
                    img[x, y] = 128# set the red colour onde nao se sabe o que tem, 0, 0
                    
            '''if y > 5 * fator:
                fator = fator + 1
                y = y + 1
                if (celula == -1):
                    img[ x, y] = 255 # set the blank colour onde nao tem nada, 255, 255
                else :
                    if (celula == 1):
                        img[x, y] = 0 # set the black colour onde tem borda, 0, 0
                    else :
                        img[x, y] = 64# set the red colour onde nao se sabe o que tem, 0, 0'''
            y = y + 1
        x = x + 1
        fator = 0
        y = 0

    cv2.imwrite("map.png", img);

    
    #calculo de erosao
    hip = math.sqrt(math.pow(ConvertMtsToPxls(position.GetSize().sw, mapa, False), 2) + math.pow(ConvertMtsToPxls(position.GetSize().sl, mapa, True), 2))
    print "sw: (%.3f)" % ConvertMtsToPxls(position.GetSize().sw, mapa, False)
    #print "hipotenusa: (%.3f)" % hip
    erosion_size  = hip
    #print "erosao: (%.3f)" % erosion_size

    imgEroded = erode(erosion_size, img)
    cv2.imwrite("erosao.png", imgEroded);

    return imgEroded




def callA_Star(mapa, position, goal, imgEroded):
    posicaoRoboX = Pos_mapToImg((position.GetXPos()), mapa, True) 
    posicaoRoboY = Pos_mapToImg(position.GetYPos(), mapa, False)

    ptRobot =  (int(posicaoRoboX), int(posicaoRoboY))
    ptGoal = (Pos_mapToImg(goal[0], mapa, True), Pos_mapToImg(goal[1], mapa, False))
    #ptGoal = (int(posicaoRoboX) + 60, int(posicaoRoboY))

    print'no mapa: ptRobot: (%.1f,%.1f)' % (position.GetXPos(), position.GetYPos())
    print'na Iamgem: ptRobot: (%.1f,%.1f), ptGoal: (%.1f,%.1f)' % (ptRobot[0], ptRobot[1], ptGoal[0], ptGoal[1])

    grafo = a_star.Graph(mapa.GetWidth(), mapa.GetHeight())
    grafo.setObstacles(imgEroded,  mapa.GetHeight(), mapa.GetWidth())
    
    imprimeR(posicaoRoboX, posicaoRoboY, imgEroded)
    imprimeP( 150, 450, imgEroded)
    imprimeC( mapa.GetWidth() / 2, mapa.GetHeight() / 2, imgEroded)
    imprimeO( ptGoal[0], ptGoal[1], imgEroded)

    cv2.imwrite("debug.png", imgEroded);

    #grafo = a_star.Graph(10, 10)
    #grafo.setObstacles(img,  10, 10)
    #ptGoal = (5,5)
    #ptRobot = (1,1)
    (came_from, cost_so_far) = a_star.a_star_search(grafo, ptRobot, ptGoal)
    
    if ptGoal in came_from :
        path =  a_star.reconstruct_path(came_from, ptRobot, ptGoal)
        path.reverse()
        return path
    else :
        print 'sem rota para o destino: ', goal
        return None

def findObstD_Star(mapa, imageEroded, ptRobot, ptGoal, pathFind):
    
    pathFind.init(ptRobot[0],ptRobot[1],ptGoal[0],ptGoal[1]);
    imgLine = np.zeros((mapa.GetHeight(),mapa.GetWidth(),1), np.uint8)


    x = 0
    y = 0
    celula = 0
    obstaculos = []
    pixelView = 20
    contador = 1
    #Detector de bordas
    while ( x < mapa.GetHeight() ):
        while ( y < mapa.GetWidth() ):
            if y != 0 and x != 0:
                if imageEroded[y-1][x] == 255 and imageEroded[y][x] == 0 or imageEroded[y][x-1] == 255 and imageEroded[y][x] == 0 or imageEroded[y-1][x] == 0 and imageEroded[y][x] == 255 or imageEroded[y][x-1] == 0 and imageEroded[y][x] == 255:
                    imgLine[y][x] = 0
                    if x >  ptRobot[0] - pixelView  and  ptRobot[0] + pixelView  > x and y > ptRobot[1] - pixelView  and  ptRobot[1] + pixelView  > y:
                        pathFind.updateCell(x, y, -1);
                        #print "ok", contador
                        contador = contador + 1;
                else :
                    imgLine[y][x] = 255
            else : 
                imgLine[y][x] = 0
                if x >  ptRobot[0] - pixelView  and  ptRobot[0] + pixelView  > x and y > ptRobot[1] - pixelView  and  ptRobot[1] + pixelView  > y:
                    pathFind.updateCell(x, y, -1);
                    #print "ok", contador
                    contador = contador + 1;
            y = y + 1
        x = x + 1
        y = 0

    img = np.zeros((mapa.GetHeight(),mapa.GetWidth(),1), np.uint8)


    x = ptRobot[0] - pixelView  
    y = ptRobot[1] - pixelView
    #imprimeR(ptRobot[0] , ptRobot[1] , img)
    cv2.imwrite("D-star.png", imgLine);

def callD_Star(mapa, position, goal, pathFind):
    posicaoRoboX = Pos_mapToImg((position.GetXPos()), mapa, True) 
    posicaoRoboY = Pos_mapToImg(position.GetYPos(), mapa, False)

    ptRobot =  (int(posicaoRoboX), int(posicaoRoboY))
    ptGoal = (Pos_mapToImg(goal[0], mapa, True), Pos_mapToImg(goal[1], mapa, False))
    #ptGoal = (int(posicaoRoboX) + 60, int(posicaoRoboY))

    print'no mapa: ptRobot: (%.1f,%.1f)' % (position.GetXPos(), position.GetYPos())
    print'na Iamgem: ptRobot: (%.1f,%.1f), ptGoal: (%.1f,%.1f)' % (ptRobot[0], ptRobot[1], ptGoal[0], ptGoal[1])

    #findObstD_Star(mapa, imgEroded, ptRobot, ptGoal, pathFind)
    ini = time.time()
    pf.init(ptRobot[0],ptRobot[1],ptGoal[0],ptGoal[1]);
    fim = time.time()
    print "INIT D*:", ptGoal, fim-ini
     
    #for valor in range(0,m.GetWidth()-m.GetWidth()/2) :
    #    pf.updateCell( m.GetWidth()/2,valor, -1)


def UpdateDstar(mapa, position, pathFind):
    posicaoRoboX = Pos_mapToImg((position.GetXPos()), mapa, True) 
    posicaoRoboY = Pos_mapToImg(position.GetYPos(), mapa, False)
    
    ptRobot =  (int(posicaoRoboX), int(posicaoRoboY))
    if  not (pf.s_start.x == ptRobot[0] and pf.s_start.y == ptRobot[1]):
        '''
        A B AandB AorB
        V V   V     V 
        V F   F     V
        F V   F     V
        F F   F     F
        '''
        print "updateStart:(", pf.s_start.x, ",", pf.s_start.y, ") to ", ptRobot
        pf.updateStart(ptRobot[0],ptRobot[1])
    print "replan"
    ini = time.time()
    if pf.replan() == False :

        return []
    fim = time.time()
    print "REPLAN D*:", fim-ini

    path = pf.getPath();

    print "QUANTIDADE DE PASSOS", len(path) 
    #raw_input('Position Goal in X?')
    for i in path:
        print"x: ", i.x, " y: ", i.y;
    
    return path

def leSensorDstar(read, position, mapa):
    varBreak = 0
    for sensor in range(0,read.GetRangeCount()-1) :
                
        if read.GetRange(sensor) < 5 :
            #O valor 0.36 eh retirado do bigbob.inc pegando o fov e dividindo por samples
            AngSensor = (sensor - read.GetRangeCount()/2) * math.radians(0.36)

            AngObjeto = AngSensor + position.GetYaw()

            if AngObjeto > math.pi :
                AngObjeto = (AngObjeto - math.pi) - math.pi

            PosObjX = math.cos(AngObjeto) * (read.GetRange(sensor)) + position.GetXPos()
            PosObjY = math.sin(AngObjeto) * (read.GetRange(sensor)) + position.GetYPos()
            PosObjImgX = int(Pos_mapToImg(PosObjX,mapa, True))
            PosObjImgY = int(Pos_mapToImg(PosObjY,mapa, False))
            #print "robo", position.GetXPos(), position.GetYPos(), "obstaculos", PosObjImgX, PosObjImgY

            if PosObjImgX > m.GetWidth() - 1:
                PosObjImgX = m.GetWidth() - 1
            if PosObjImgY > m.GetHeight() - 1:
                PosObjImgY = m.GetHeight() - 1
            #print "m.GetHeight()", m.GetHeight(), m.GetWidth()
            #print "LOL_X",PosObjImgY ,"LOL_Y",PosObjImgX
            if auxiliar[PosObjImgY][PosObjImgX] != 255 :
                auxiliar[PosObjImgY][PosObjImgX] = 255
                pf.updateCell( PosObjImgX, PosObjImgY, -1)

                varBreak = 1
    return varBreak




# Create a client object
c = PlayerClient('localhost')

# Create a proxy for position2d:0
# nao muda nada usar goto ou position2d driver, entao ficamos c o position2d
p = Position2dProxy(c,1)
p.RequestGeom()
size = p.GetSize()
print 'Robot size: (%.3f,%.3f,%.3f)' % (size.sw, size.sl, size.sh)

# para localizar os obstaculos estaticos
m = MapProxy(c,0)
m.RequestMap()
print 'Map size: (%d,%d), origem: (%d,%d), resolution (m/cell) %.3f)' % (m.GetWidth(), m.GetHeight(), m.GetOriginX(), m.GetOriginY(), m.GetResolution())

pf = DStarLite.DStarLite();


# Create a proxy for laser ranger:0
# para localizar obstaculos dinamicos
s = RangerProxy(c,0)
s.RequestGeom()

# read once to get things going
c.Read()

# set the goal position
ga = 0.0
p.SetMotorEnable(True)
p.ResetOdometry()
#p.GoTo(gx,gy,ga)

# tolerancia em relacao ao objetivo
distance_epsilon = 0.5
angle_epsilon = dtor(5)

'''path = None
while path == None :
    Goal = (int(raw_input('Position Goal in X?')),int(raw_input('Position Goal in Y?')))
    imgEroded = geraImagem(m, p, Goal)
    path = callA_Star(m, p, Goal, imgEroded)
'''

path = None
#while path == None :
Goal = (int(raw_input('Position Goal in X?')),int(raw_input('Position Goal in Y?')))
    #imgEroded = geraImagem(m, p, Goal)
    #path = callD_Star(m, p, Goal, pf)


#print len(path)
#raw_input('Position Goal in X?')

callD_Star(m, p, Goal, pf)
#UpdateDstar(m, p, pf)

auxiliar = np.zeros((m.GetHeight(),m.GetWidth(),1), np.uint8)
'''for valor in range(400,500) :
    pf.updateCell( 250,valor, -1)'''
contador = 0
contadorGOTO = 0
while 1 :
    print 'laco 0'
    #imprime = "auxiliar"
    #imprime += 'contador'
    posicaoRoboX = Pos_mapToImg((p.GetXPos()), m, True) 
    posicaoRoboY = Pos_mapToImg(p.GetYPos(), m, False)    
    
    
    #varBreak = leSensorDstar(s, p, m)
    '''for waypoint in path :
        imprimeO(waypoint.x,waypoint.y,auxiliar)'''
    varBreak = 0
    debug = deepcopy(auxiliar)
    imprimeR(posicaoRoboX,posicaoRoboY,debug)
    cv2.imwrite("auxiliar"+str(Goal)+str(contador)+".png", debug);
    cv2.imwrite("auxiliar"+str(Goal)+str(contador)+"-1.png", auxiliar);
    contador = contador +1

    path = None
    path = UpdateDstar(m, p, pf)
    #raw_input('Position Goal in X?')

    for waypoint in path :
        print 'laco waypoint'

        #if varBreak :
        #    continue

        gx = (Pos_imgToMap(waypoint.x, m, True))
        #gx = (Pos_imgToMap(waypoint[0], m, True))
        #gx = math.ceil(Pos_imgToMap(waypoint[0], m, True))
        gy = (-Pos_imgToMap(waypoint.y, m, False))
        #gy = (-Pos_imgToMap(waypoint[1], m, False))
        #gy = math.ceil(-Pos_imgToMap(waypoint[1], m, False))

        print'Pos_imgToMap( %.3f, %.3f). gx&gy ( %.3f, %.3f)' % ((Pos_imgToMap(waypoint.x, m, True)),(Pos_imgToMap(waypoint.y, m, False)), gx, gy )
        print "posAtual", p.GetXPos(), p.GetYPos()
        #print'Pos_imgToMap( %.3f, %.3f). gx&gy ( %.3f, %.3f)' % ((Pos_imgToMap(waypoint[0], m, True)),(Pos_imgToMap(waypoint[1], m, False)), gx, gy )

        p.GoTo(gx,gy,p.GetYaw())
        contadorGOTO = contadorGOTO + 1
        #varBreak = 0
        while ( math.sqrt( (p.GetXPos()-gx)**2+ (p.GetYPos()-gy)**2) > distance_epsilon):
            print 'laco 1'
            # read the sensors, etc.
            varBreak = leSensorDstar(s, p, m)
            c.Read()

            #while True :
            print "Alessandro"
            print "contador goto", contadorGOTO, "posAtual", p.GetXPos(), p.GetYPos(), "objetivo", gx, gy
            #p.SetSpeed(0.0, 0.0)



            
            if varBreak :
                #p.SetSpeed(0.0, 0.0)
                #c.Read()
                break


            # p.GetStall() eh interessante para detectar colisao do robo
            print 'Current pose (%.3f %.3f %.3f), waypoint (%.3f %.3f %.3f), Goal %.3f, %.3f' %\
                    (p.GetXPos(),p.GetYPos(),p.GetYaw(),gx,gy,ga, Goal[0], Goal[1])
        c.Read()
        if varBreak :
            break
        print 'done waypoint' 

    if varBreak :
        continue
    break
p.GoTo(gx,gy,ga)        
while ( abs(  (p.GetYaw()-ga) > angle_epsilon ) ):
    c.Read()    

print 'NUMEROS DE ESTADOS EXPANDIDOS:', pf.openListCount

'''-6,-6 >>> -5,-5
INIT D*: (83, 417) 8.48770141602e-05
REPLAN D*: 0.019161939621
QUANTIDADE DE PASSOS 24
-------
INIT D*: (83, 417) 0.000204086303711
REPLAN D*: 0.0203518867493
QUANTIDADE DE PASSOS 24
-------
INIT D*: (83, 417) 6.89029693604e-05
REPLAN D*: 0.0193190574646
QUANTIDADE DE PASSOS 24
'''
#################
'''-5,-5 >>> -6,-6  
INIT D*: (50, 450) 0.000102043151855
REPLAN D*: 0.025542974472
QUANTIDADE DE PASSOS 34
-------
INIT D*: (50, 450) 9.20295715332e-05
REPLAN D*: 0.0258178710938
QUANTIDADE DE PASSOS 34
-------
INIT D*: (50, 450) 7.60555267334e-05
REPLAN D*: 0.0258409976959
QUANTIDADE DE PASSOS 34
'''
#1836
#QUANTIDADE DE PASSOS PARA ATINGIR O OBJETIVO_1(-6,-6) - 134 | 134 | 134
#QUANTIDADE DE PASSOS PARA ATINGIR O OBJETIVO_1(-5,-5) - 34  | 33  | 29
#QUANTIDADE DE PASSOS PARA ATINGIR O OBJETIVO_N(-6,-6) - 949 | 774 | 812
#QUANTIDADE DE PASSOS PARA ATINGIR O OBJETIVO_N(-5,-5) - 200 | 219 | 268
#VIZINHOS VISITADOS_1(-6, -6) - 264 | 264 | 264
#VIZINHOS VISITADOS_1(-5, -5) - 256 | 264 | 216
#VIZINHOS VISITADOS_N(-6, -6) - 1552 | 1504 | 1536
#VIZINHOS VISITADOS_N(-5, -5) - 1400 | 2016 | 1848
#QUANTIDADE DE REPLANEJAMENTO(-5,-5) - 8
#QUANTIDADE DE REPLANEJAMENTO(-5,-5) - 7
#init(-5,-5) - 7.79628753662e-05
#replan(-5,-5) - 0.0240190029144
#init(-6,-6) - 7.70092010498e-05
#replan(-6,-6) - 0.02676820755




            
# Now stop
p.SetSpeed(0.0, 0.0)

del p
del s
del m
# o robo deve ser o ultimo a ser deletado
del c


