import random
import matplotlib.pyplot as plt
import numpy as np
NumofPoints = 100
plt.figure()
plt.xlim(0, NumofPoints)
plt.ylim(0, NumofPoints)
path = r'E:temp\\'


class TrianglePoint():  # Point of Triangle
    def __init__(self, x=0, y=0, nPointID=0):
        (self.x, self.y, self.nPointID) = (x, y, nPointID)

class TriangleEdge(): # Edge of Triangle
    def __init__(self, edge):
        self.edge = edge
        self.ID = 0
    # ID: the id of the Point
    # EdgeIndex: the index of another point of the Edge
    # nUsedTime: the time the Edge is used

class Triangle():
    def __init__(self, p1=0, p2=0, p3=0, nTriangleID=0):
        (self.p1, self.p2, self.p3, self.nTriangleID) = (p1, p2, p3, nTriangleID)

def GRP():  # Generate Random Points
    x = [random.uniform(0, NumofPoints) for i in range(NumofPoints)]
    y = [random.uniform(0, NumofPoints) for i in range(NumofPoints)]
    data = open(path + 'Points' + str(NumofPoints) + '.txt', 'w')
    for i in range(NumofPoints):
        data.write(str(x[i]) + ' ' + str(y[i]) + '\n')
    data.close()
    return (x, y)

def ReadPoints():
    x = [float(i.split()[0]) for i in open(path + 'Points' + str(NumofPoints) + '.txt', 'r')]
    y = [float(i.split()[1]) for i in open(path + 'Points' + str(NumofPoints) + '.txt', 'r')]
    return (x, y)
(x, y) = GRP()
#(x, y) = ReadPoints()  # read the coord of points and save to x,y
plt.plot(x, y, '.')
plt.show()
P = {}
Pl = []
Edge = {}
TIN = {}
for i in range(len(x)):
    P[i] = TrianglePoint(x[i], y[i], i)
    Pl.append(i)
    Edge[i] = TriangleEdge({})
    TIN[i] = Triangle()

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
    c = np.sqrt((p.x - p1.x)**2 + (p.y - p1.y)**2)
    return (b**2 + c**2 - a**2) / (2 * b * c)

def EdgeExband(TC, nCur, option='p1p2', indent=0):
    # Exband edge
    # option can be 'p1p2', 'p2p3' or 'p1p3'
    # return TriCount

    space = '    '
    print(space*indent + '>>>Enter EdgeExband')

    # step 1
    Tri = TIN[nCur]
    print(space*indent + '>>>step 1')

    if option == 'p1p2':
        (p1, p2, p3) = (Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID)
    elif option == 'p2p3':
        (p1, p2, p3) = (Tri.p2.nPointID, Tri.p3.nPointID, Tri.p1.nPointID)
    elif option == 'p1p3':
        (p1, p2, p3) = (Tri.p1.nPointID, Tri.p3.nPointID, Tri.p2.nPointID)
    print(space*indent + 'exband p1=%d p2=%d p3=%d' % (p1, p2, p3))

    if p2 in Edge[p1].edge.keys() and Edge[p1].edge[p2] >=2:
        print(space + 'used time of p1 p2(%d %d) is bigger than 2' % (p1, p2))
        print(space * indent + 'Exit EdgeExband')
        return None

    # stpe 2
    print(space*indent + '>>>step 2')
    p = 0
    cos = 1
    p_find_flag = False # this flag shows if we can find a p that satisfy the condition
    for i in P.keys():
        print(space * (indent+1) +'i=%d' % i)
        # step 2.1
        print(space * (indent+1) + '>>>step 2.1')
        if i != p1 and i != p2:
            # step 2.2
            print(space * (indent+1) + '>>>step 2.2')
            #print(P[p2].x - P[p1].x)
            A = (P[p2].y - P[p1].y) / (P[p2].x - P[p1].x)
            B = (P[p1].y * P[p2].x - P[p2].y * P[p1].x) / (P[p2].x - P[p1].x)
            # the function of the line (from p1 to p2)
            def Func(t):
                return P[t].y - A * P[t].x - B

            #
            if p3 in P.keys() and Func(i) * Func(p3) < 0:
                print(space * 2 + 'i(%d) and p3(%d) lies in the different part of line made up by p1(%d) and p2(%d)' % (i, p3, p1, p2))

                # step 2.3
                print(space * (indent+1) + '>>>step 2.3')
                if (p1 in Edge[i].edge.keys() and Edge[i].edge[p1] >= 2) or \
                        (p2 in Edge[i].edge.keys() and Edge[i].edge[p2] >= 2):
                    pass
                else:
                    # step 2.4
                    print(space * (indent+1) + '>>>step 2.4')
                    #print(P[i].x, P[i].y, P[p1].x, P[p1].y, P[p2].x, P[p2].y)
                    cos_temp = calccos(P[i], P[p1], P[p2])
                    #print(cos_temp, cos)
                    if cos_temp < cos:
                        cos = cos_temp
                        p = i
                        p_find_flag = True
                        print('find p=%d' % p)

    def UpdateEdge(p1, p2, p): # update the Edge of p1
        if p2 in Edge[p1].edge.keys():
            Edge[p1].edge[p2] += 1
        else:
            Edge[p1].edge[p2] = 1
        #print(space * indent + 'Edge[p1(%d)].edge[p2(%d)] = %d' % (p1, p2, Edge[p1].edge[p2]))

        if p in Edge[p1].edge.keys():
            Edge[p1].edge[p] += 1
        else:
            Edge[p1].edge[p] = 1
        #print(space * indent + 'Edge[p1(%d)].edge[p(%d)] = %d' % (p1, p, Edge[p1].edge[p]))
        pop_flag = True
        # initial pop flag is True
        # pop flag is True means p1 will pop from P
        # if any Usedtime in p1's edge dict don't equal to 2, flag trans to false
        for i in Edge[p1].edge.values():
            if i != 2:
                pop_flag = False
                break

        if pop_flag == True:
            P.pop(p1)
            Pl.remove(p1)

    # step 3
    print(space*indent + '>>>step 3')

    if p_find_flag == True:

        # step 4
        print(space*indent + '>>>step 4')
        TIN[TC] = Triangle(P[p1], P[p2], P[p], TC)
        TC += 1
        print(space*indent + 'TriCount %d' % TC)
        # step 5
        print(space*indent + '>>>step 5')
        UpdateEdge(p1, p2, p)
        UpdateEdge(p2, p1, p)
        UpdateEdge(p, p1, p2)
        PrintEdge(Edge)
        print(space*indent+'Exit ExbandEdge')
        return TC
    else:
        PrintEdge(Edge)
        print(space * indent + 'Exit EdgeExband')
        return None

def GetInitTriangle(TC, nCur):
    print('Enter GetInitTriangle')
    # Find an Triangle (edge: p1 p2)
    # return the triangle found and TriCount
    init = 0
    def inTIN(p1, p2, p3):
        #print('Enter inTIN')
        #print('p1 = %d p2=%d p3=%d', (p1, p2, p3))

        #PrintTin(TIN, TC)
        if TC >= 1:
            for i in range(TC):
                Tri = TIN[i]
                TriPl = [Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID]
                #if p1 == TIN[i].p1.nPointID and p2 == TIN[i].p2.nPointID and p3 == TIN[i].p3.nPointID:
                if p1 in TriPl and p2 in TriPl and p3 in TriPl:
                    #print('in TIN')
                    #print('Exit inTIN')
                    return True
        #print('Exit inTIN')
        return False
    def UpdateEdge(p1, p2, p3):  # update the Edge of p1
        #PrintEdge(Edge)
        if p2 in Edge[p1].edge.keys():

            #print('in')
            #PrintEdge(Edge)
            Edge[p1].edge[p2] += 1
        else:
            Edge[p1].edge[p2] = 1
        #print('Edge[p1(%d)].edge[p2(%d)] = %d' % (p1, p2, Edge[p1].edge[p2]))

        if p3 in Edge[p1].edge.keys():
            #print('in')
            #PrintEdge(Edge)
            Edge[p1].edge[p3] += 1
        else:
            Edge[p1].edge[p3] = 1
        #print('Edge[p1(%d)].edge[p3(%d)] = %d' % (p1, p3, Edge[p1].edge[p3]))

    while init < len(Pl):
        p1 = Pl[init]
        # finding the point that is nearest to the initial Point
        ds_less = 2 * NumofPoints**2 # square of distance of any two points should less than this
        p2 = 0
        for i in P.keys():
            if i != p1:
                ds_temp = (P[i].x - P[p1].x) ** 2 + (P[i].y - P[p1].y) ** 2
                if ds_temp <= ds_less:
                    ds_less = ds_temp
                    p2 = i
        # finding a point and its sum of squares
        # of distance to the initPoint and the nextPoint
        ds2_less = 4 * NumofPoints**2 # sum of squares of distance should less than this

        p3 = 0
        cos = 1
        p3_flag = False
        for i in P.keys():
            # if i != p1 and i != p2:
            #     ds2_temp = (P[i].x - P[p1].x) ** 2 + (P[i].y - P[p1].y) ** 2 + \
            #                (P[i].x - P[p2].x) ** 2 + (P[i].y - P[p2].y) ** 2
            #     if ds2_temp <= ds2_less:
            #         ds2_less = ds2_temp
            #         p3 = i
            cos_temp = calccos(P[i], P[p1], P[p2])
            if cos_temp < cos:
                cos = cos_temp
                p3 = i
                p3_flag = True

        t = inTIN(p1, p2, p3)
        if p3_flag == True and t == True:
            init += 1
        else:
            print('add p1=%d p2=%d p3=%d TC=%d' % (p1, p2, p3, TC))
            TIN[TC] = Triangle(P[p1], P[p2], P[p3], TC)
            #PrintEdge(Edge)
            # print(p3 in Edge[p1].edge.keys())
            UpdateEdge(p1, p2, p3)
            UpdateEdge(p2, p1, p3)
            UpdateEdge(p3, p1, p2)

            TC += 1
            temp = EdgeExband(TC, nCur, option='p1p2', indent=1)
            if temp:
                TC = temp
            print('Exit GetinitTriangle')
            return TC


def Exband(TC, nCur):
    print(TC, nCur)
    TC = GetInitTriangle(TC, nCurTri)
    while nCur <= TC:
        print('TriCount=%d nCur=%d' % (TC, nCur))
        if nCur == TC:
            get_temp = GetInitTriangle(TC, nCurTri)
            if get_temp:
                TC = get_temp
            DrawTin(TIN, TC)
        Tri = TIN[nCur]
        (p1, p2, p3) = (Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID)
        temp1 = EdgeExband(TC, nCur, option='p2p3')
        if temp1:
            TC = temp1
            DrawTin(TIN, TC)

        temp2 = EdgeExband(TC, nCur, option='p1p3')
        if temp2:
            TC = temp2
            DrawTin(TIN, TC)
        nCur += 1

    return (TC, nCur)

def DrawTin(Tin, TC):
    plt.figure()
    plt.xlim(0, NumofPoints)
    plt.ylim(0, NumofPoints)
    plt.plot(x, y, '.')
    for i in range(NumofPoints):
        plt.text(x[i], y[i], str(i))
    for i in range(TC):
        Tri = Tin[i]
        plt.plot([Tri.p1.x, Tri.p2.x], [Tri.p1.y, Tri.p2.y])
        plt.plot([Tri.p1.x, Tri.p3.x], [Tri.p1.y, Tri.p3.y])
        plt.plot([Tri.p3.x, Tri.p2.x], [Tri.p3.y, Tri.p2.y])
    plt.show()

def PrintTin(Tin, m):
    for i in range(m):
        print('i=%d' % i)
        Tri = Tin[i]
        print(Tri.p1.nPointID, Tri.p2.nPointID, Tri.p3.nPointID)

def PrintEdge(Ed):
    for i in Ed.keys():
        for j in Ed[i].edge.keys():
            print('Edge[%d].edge[%d] is %d' % (i, j, Ed[i].edge[j]))
#TriCount = GetInitTriangle(TriCount, nCurTri)
#TriCount = EdgeExband(TriCount, nCurTri)
(TriCount, nCurTri) = Exband(TriCount, nCurTri)
# print('TriCount %d' % TriCount)
# PrintTin(TIN, TriCount)
# DrawTin(TIN, TriCount)
# plt.show()
