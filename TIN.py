import random
import matplotlib.pyplot as plt
import numpy as np
plt.figure()
plt.xlim(0, 10)
plt.ylim(0, 10)
path = r'D:\大三下事务\闯世界\ReUMICH\TIN Generation\\'
NumofPoints = 10

class TrianglePoint():  # Point of Triangle
    def __init__(self, x=0, y=0, nPointID=0):
        (self.x, self.y, self.nPointID) = (x, y, nPointID)

class TriangleEdge(): # Edge of Triangle
    edge = {}
    def __init__(self, EdgeIndex=0, nUsedTime=0):
        self.ID = 0
        self.edge[EdgeIndex] = nUsedTime
    # ID: the id of the Point
    # EdgeIndex: the index of another point of the Edge
    # nUsedTime: the time the Edge is used

    def GetUsedTime(self, EdgeIndex):
        return self.edge[EdgeIndex]

class Triangle():
    def __init__(self, p1=0, p2=0, p3=0, nTriangleID=0):
        (self.p1, self.p2, self.p3, self.nTriangleID) = (p1, p2, p3, nTriangleID)

def GRP():  # Generate Random Points
    x = [random.uniform(0, 10) for i in range(NumofPoints)]
    y = [random.uniform(0, 10) for i in range(NumofPoints)]
    data = open(path + 'Points.txt', 'w')
    for i in range(NumofPoints):
        data.write(str(x[i]) + ' ' + str(y[i]) + '\n')
    data.close()

def ReadPoints():
    x = [float(i.split()[0]) for i in open(path + 'Points.txt', 'r')]
    y = [float(i.split()[1]) for i in open(path + 'Points.txt', 'r')]
    return (x, y)

(x, y) = ReadPoints()  # read the coord of points and save to x,y
plt.plot(x, y, '.')
for i in range(NumofPoints):
    plt.text(x[i], y[i], str(i))
m_TrianglePoints = {}
m_TriangleEdges = {}
m_TIN = {}
for i in range(len(x)):
    m_TrianglePoints[i] = TrianglePoint(x[i], y[i], i)
    m_TriangleEdges = TriangleEdge()
    m_TIN[i] = Triangle()
nCurTri = 0
TriCount = 0

def inCircumcircle(p, p1, p2, p3): # Return TRUE if the Point (xp, yp) lies inside the circumcircle
    # print(p.x, p.y)
    # print(p1.x, p1.y)
    # print(p2.x, p2.y)
    # print(p3.x, p3.y)
    if p2.y == p1.y:
        m2 = -(p3.x - p2.x) / (p3.y - p2.y)
        mx2 = (p2.x + p3.x) * 0.5
        my2 = (p2.y + p3.y) * 0.5
        # CircumCircle center(xc, yc)
        xc = (p2.x + p1.x) * 0.5
        yc = m2 * (xc - mx2) + my2
    elif p3.y==p2.y:
        m1 = -(p2.x - p1.x) / (p2.y - p1.y)
        mx1 = (p1.x + p2.x) * 0.5
        my1 = (p1.y + p2.y) * 0.5
        # CircumCircle center(xc, yc)
        xc = (p3.x + p2.x) * 0.5
        yc = m1 * (xc - mx1) + my1
    else:
        m1 = -(p2.x - p1.x) / (p2.y - p1.y)
        m2 = -(p3.x - p2.y) / (p3.y - p2.y)
        mx1 = (p1.x + p2.x) * 0.5
        mx2 = (p2.x + p3.x) * 0.5
        my1 = (p1.y + p2.y) * 0.5
        my2 = (p2.y + p3.y) * 0.5
        # CircumCircle center(xc, yc)
        xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2)
        yc = m1 * (xc - mx1) + my1

    rs = (xc - mx1)**2 + (yc - my1)**2
    ds = (xc - p.x)**2 + (yc - p.y)**2
    return ds <= rs

def calccos(p, p1, p2): # calculate the cos of angle of p1-> p -> p2
    a = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    b = np.sqrt((p.x - p2.x)**2 + (p.y - p2.y)**2)
    c = np.sqrt((p.x - p1.x) ** 2 + (p.y - p1.y) ** 2)
    return (b**2 + c**2 - a**2) / (2 * b * c)

def inTin(Tri, Tin, m): # if triangle already in TIN, return True
    (p1_id, p2_id, p3_id) = (Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID)
    for i in range(m):
        idlist = [Tin[m].p1.nPointID, Tin[m].p2.nPointID, Tin[m].p3.nPointID]
        if p1_id in idlist and p2_id in idlist and p3_id in idlist:
            return True
    return False

def GetInitTriangle(Points):  # Find an init Triangle
    initID = 0
    while True:
        initPoint = Points[initID]
        # finding the point that is nearest to the initial Point
        ds_less = 2 * NumofPoints**2 # square of distance of any two points should less than this
        nearestID = initID + 1
        for i in range(initID + 1, NumofPoints):
            ds_temp = (Points[i].x - initPoint.x)**2 + (Points[i].y - initPoint.y)**2
            if ds_temp <= ds_less:
                ds_less = ds_temp
                nearestID = i
        nextID = nearestID
        nextPoint = Points[nearestID]

        # finding a point and its sum of squares
        # of distance to the initPoint and the nextPoint
        ds2_less = 4 * NumofPoints**2 # sum of squares of distance should less than this
        lessID = 0
        for i in range(initID+1, NumofPoints):
            if i != initID and i != nextID:
                ds2_temp = (Points[i].x - initPoint.x)**2 + (Points[i].y - initPoint.y)**2 + \
                           (Points[i].x - nextPoint.x) ** 2 + (Points[i].y - nextPoint.y) ** 2
                if ds2_temp <= ds2_less:
                    ds2_less = ds2_temp
                    lessID = i
        thirdID = lessID
        thirdPoint = Points[lessID]

        # check if there exists any point out of the circumcircle
        # made up by initPoint, nextPoint, thirdPoint
        # if exist, change an initPoint(initID+=1) and do this progress again
        flag = False
        for i in range(initID+1, NumofPoints):
            if i not in [initID, nextID, thirdID]:
                if inCircumcircle(Points[i], initPoint, nextPoint, thirdPoint) == True:
                    flag = True
                    break

        if flag == True:
            print(flag)
            initID += 1
        else:
            return Triangle(Points[initID], Points[nextID], Points[thirdID], 0)

def EdgeExband(Tri, P, Tin, Edge, TriCount, nCurTri):
    (p1, p2, p3) = (Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID)
    if Edge[p1].edge[p2] >=2:
        return 0
    p = 0
    cos = 1
    p_find_flag = False
    for i in range(NumofPoints):
        if i != p1 and i != p2:
            A = (P[p2].y - P[p1].y) / (P[p2].x - P[p1].x)
            B = (P[p1].y * P[p2].x - P[p2].y * P[p1].x) / (P[p2].x - P[p1].x)

            # the function of the line (from p1 to p2)
            def Func(p):
                return P[p].y - A * P[p].x - B

            if Func(i) * Func(p3) < 0:
                if p1 in Edge[i].edge.keys() and Edge[i].edge[p1] >= 2:
                    break
                if p2 in Edge[i].edge.keys() and Edge[i].edge[p2] >= 2:
                    break
                cos_temp = calccos(P[p], P[p1], P[p2])
                if cos_temp < cos:
                    cos = cos_temp
                    p = i
                    p_find_flag = True

    if p_find_flag == True:

        TriCount += 1

def Exband(Tri, Points,  l):
    def FindPEP(p1, p2, p3, Points):
        re = []
        A = (p2.y - p1.y) / (p2.x - p1.x)
        B = (p1.y * p2.x - p2.y * p1.x) / (p2.x - p1.x)
        # the function of the line (from p1 to p2)
        def Func(p):
            return p.y - A * p.x - B

        for i in range(NumofPoints):
            if i not in [p1.nPointID, p2.nPointID, p3.nPointID]:
                if Func(Points[i]) * Func(p3) < 0:
                    re.append(i)
        return re

    PEP = []  # Possible Expand Points
    (p1, p2, p3) = (Tri[l].p1, Tri[l].p2, Tri[l].p3)
    PEP = FindPEP(p1, p2, p3, Points)
    return PEP
    # if len(PEP) != 0:
    #     return PEP
    # else:
    #     PEP = FindPEP(p3, p2, p1, Points)
    #     if len(PEP) != 0:
    #         return PEP
    #     else:
    #         PEP = FindPEP(p3, p1, p2, Points)
    #         return PEP

def GetNextTriangle(Tin, Points, m):
    (p1, p2, p3) = (Tin[m].p1, Tin[m].p2, Tin[m].p3)
    PEP = Exband(Tin, Points, m)
    print(PEP)
    for i in PEP:
        flag = False
        # initial flag is false
        # flag is False means no other point exists in the circumcircle
        for j in range(NumofPoints):
            if inCircumcircle(Points[j], p1, p2, Points[i]) == True:
                flag = True
                break

        if flag == False:
            print(i, flag, ': no other point exists in the circumcircle')
            m += 1
            (Tin[m].p1, Tin[m].p2, Tin[m].p3, Tin[m].nTriangleID) = (p1, p2, Points[i], m)
    return (Tin, m)

def allGenerated(TIN):
    pass

def ExpandTriangle(Tin, Points, n, m):

    while n <= m:
        (p1, p2, p3) = (Tin[n].p1, Tin[n].p2, Tin[n].p3)
        print('n=%d, m=%d' % (n, m))
        print('points are ', Tin[n].p1.nPointID, Tin[n].p2.nPointID, Tin[n].p3.nPointID)
        PEP = Exband(Tin, Points, n)
        print('PEP', PEP)
        for i in PEP:
            flag = False
            # initial flag is false
            # flag is False means no other point exists in the circumcircle
            for j in range(NumofPoints):
                if inCircumcircle(Points[j], p1, p2, Points[i]) == True:
                    flag = True
                    break

            if flag == False:
                print('no other point exists in the circumcircle by', i, p1.nPointID, p2.nPointID)
                tri_temp = Triangle(p1, p2, Points[i], m)
                if inTin(tri_temp, Tin, m) == False:
                    m += 1
                    Tin[m] = tri_temp
                    #(Tin[m].p1, Tin[m].p2, Tin[m].p3, Tin[m].nTriangleID) = (p1, p2, Points[i], m)
                # if n != m:
                #     #pass
                #     ExpandTriangle(Tin, Points, n, m)
        n += 1

def DrawTriangle(Tri):
    plt.plot([Tri.p1.x, Tri.p2.x], [Tri.p1.y, Tri.p2.y])
    plt.plot([Tri.p1.x, Tri.p3.x], [Tri.p1.y, Tri.p3.y])
    plt.plot([Tri.p3.x, Tri.p2.x], [Tri.p3.y, Tri.p2.y])

def DrawTin(Tin, m):
    for i in range(m):
        DrawTriangle(Tin[i])

m_TIN[0] = GetInitTriangle(m_TrianglePoints)

(m_TIN, TriCount) = GetNextTriangle(m_TIN, m_TrianglePoints, TriCount)
print(TriCount)
#ExpandTriangle(m_TIN, m_TrianglePoints, nCurTri, TriCount)

DrawTin(m_TIN, TriCount)
plt.show()
