#!/usr/bin/python

# bb_vfh.py

# Desc: Minimal config for path planning
# Author: Alexandre Amory
# Date: 10/19/2014
# to run player server - "player bb_goto.cfg" in one window
# to run controller - "python bb_goto.py" in another window

import sys,os,math
import cv2
import cv as cv
import numpy as np
import implementation as a_star
# this should be whereever "playerc.py" is.  
# On linux, you can find this out with "locate playerc.py"
sys.path.append('/usr/local/lib64/python2.7/site-packages/')
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
        return ((posImg - mapa.GetHeight() / 2) * 15 / mapa.GetHeight())
    return ((posImg - mapa.GetHeight() / 2) * 15 / mapa.GetWidth())

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
    erosion_size = 2*erosion_size+1
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
    imgNoEroded = img

    
    #calculo de erosao
    hip = math.sqrt(math.pow(ConvertMtsToPxls(position.GetSize().sw, mapa, False), 2) + math.pow(ConvertMtsToPxls(position.GetSize().sl, mapa, True), 2))
    #print "sw: (%.3f)" % ConvertMtsToPxls(position.GetSize().sw, mapa, False)
    #print "hipotenusa: (%.3f)" % hip
    erosion_size  = hip / 2 #????
    #print "erosao: (%.3f)" % erosion_size

    imgEroded = erode(erosion_size, img)
    cv2.imwrite("erosao.png", imgEroded);





    #print 'DEBUG: posReal:(%.3f, %.3f) withMathCeil:(%.3f, %.3f)' % (position.GetXPos(), position.GetYPos(),round(position.GetXPos()), round(position.GetYPos()))

    posicaoRoboX = Pos_mapToImg(math.ceil(position.GetXPos()), mapa, True) 
    posicaoRoboY = Pos_mapToImg(math.ceil(position.GetYPos()), mapa, False)

    ptRobot =  (int(posicaoRoboX), int(posicaoRoboY))
    ptGoal = (Pos_mapToImg(goal[0], mapa, True), Pos_mapToImg(goal[1], mapa, True))
    #ptGoal = (int(posicaoRoboX) + 60, int(posicaoRoboY))

    print'no mapa: ptRobot: (%.3f,%.3f)' % (position.GetXPos(), position.GetYPos())
    print'na Iamgem: ptRobot: (%.3f,%.3f), ptGoal: (%.3f,%.3f)' % (ptRobot[0], ptRobot[1], ptGoal[0], ptRobot[1])

    imprimeR(posicaoRoboX, posicaoRoboY, imgEroded)
    imprimeP( 245, 454, imgNoEroded)
    imprimeC( mapa.GetWidth() / 2, mapa.GetHeight() / 2, imgEroded)
    imprimeO( ptGoal[0], ptGoal[1], imgEroded)

    cv2.imwrite("debug.png", imgEroded);

    grafo = a_star.Graph(mapa.GetWidth(), mapa.GetHeight())
    grafo.setObstacles(imgEroded,  mapa.GetHeight(), mapa.GetWidth())
    #grafo = a_star.Graph(10, 10)
    #grafo.setObstacles(img,  10, 10)
    #ptGoal = (5,5)
    #ptRobot = (1,1)
    (came_from, cost_so_far) = a_star.a_star_search(grafo, ptRobot, ptGoal)
    return a_star.reconstruct_path(came_from, ptRobot, ptGoal)


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

 


# Create a proxy for sonar ranger:0
# para localizar obstaculos dinamicos
s = RangerProxy(c,1)
s.RequestGeom()

# read once to get things going
c.Read()

# set the goal position
ga = 0.0
p.SetMotorEnable(True)
p.ResetOdometry()
#p.GoTo(gx,gy,ga)

# tolerancia em relacao ao objetivo
distance_epsilon = 0.1
angle_epsilon = dtor(180)

Goal = (int(raw_input('Position Goal in X?')),int(raw_input('Position Goal in Y?')))
#Goal = (2,2)
#goalAux = (Goal[0]+1, Goal[1]+1)
goalAux = Goal

path = geraImagem(m, p, goalAux)
path.reverse()
for waypoint in path :
    gx = math.ceil(Pos_imgToMap(waypoint[0], m, True))
    gy = math.ceil(Pos_imgToMap(waypoint[1], m, False))
    print'mathceil( %.3f, %.3f). gx&gy ( %.3f, %.3f)' % ((Pos_imgToMap(waypoint[0], m, True)),(Pos_imgToMap(waypoint[1], m, False)), gx, gy )

    p.GoTo(gx,gy,ga)
    while ( math.sqrt( (p.GetXPos()-gx)**2+ (p.GetYPos()-gy)**2) > distance_epsilon or\
            abs(  (p.GetYaw()-ga) > angle_epsilon )):
        # read the sensors, etc.
        c.Read()

        # p.GetStall() eh interessante para detectar colisao do robo
        print 'Current pose (%.1f %.1f %.1f), waypoint (%.1f %.1f %.1f), Goal %.1f, %.1f' %\
                (p.GetXPos(),p.GetYPos(),p.GetYaw(),gx,gy,ga, Goal[0], Goal[1])

    print 'done ', waypoint

# Now stop
p.SetSpeed(0.0, 0.0)

del p
del s
del m
# o robo deve ser o ultimo a ser deletado
del c



