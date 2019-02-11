'''
 A*, path planner including elevation heuristic considerations
Brady Ibanez - 100367230
Check functions at lines 42, 96, and modifications at line 160 for elevation considerations. 
'''

from Tkinter import *
import struct
import xml.etree.ElementTree as ET
from Queue import *
import math

# bounds of the window, in lat/long
LEFTLON = -78.9357000
RIGHTLON = -78.8118000
TOPLAT = 43.9921000
BOTLAT = 43.8494000
WIDTH = RIGHTLON-LEFTLON
HEIGHT = TOPLAT-BOTLAT
# ratio of one degree of longitude to one degree of latitude 
LONRATIO = math.cos(TOPLAT*3.1415/180)
WINWID = 500
WINHGT = (int)((WINWID/LONRATIO)*HEIGHT/WIDTH)
TOXPIX = WINWID/WIDTH
TOYPIX = WINHGT/HEIGHT
#width,height of elevation array
EPIX = 1201
# approximate number of meters per degree of latitude
MPERLAT = 111000
MPERLON = MPERLAT*LONRATIO
elevDiffPrior = 0
distPrior = 0

def node_dist(n1, n2):
    ''' Distance between nodes n1 and n2, in meters. '''
    dx = (n2.pos[0]-n1.pos[0])*MPERLON
    dy = (n2.pos[1]-n1.pos[1])*MPERLAT
    return math.sqrt(dx*dx+dy*dy) # in meters

#Used to find difference in elevation between two immediately oriented nodes
#in path
def node_elev_diff(n1, n2):
    return float(n2.elev - n1.elev)
 
class Node():
    ''' Graph (map) node, not a search node! '''
    __slots__ = ('id', 'pos', 'ways', 'elev')
    def __init__(self,id,p,e=0):
        self.id = id
        self.pos = p
        self.ways = []
        self.elev = e
        self.waystr = None
    def __str__(self):
        if self.waystr is None:
            self.waystr = self.get_waystr()
        return str(self.pos) + ": " + self.waystr
    def get_waystr(self):
        if self.waystr is None:
            self.waystr = ""
            self.wayset = set()
            for w in self.ways:
                self.wayset.add(w.way.name)
            for w in self.wayset:
                self.waystr += w.encode("utf-8") + " "
        return self.waystr
        

class Edge():
    ''' Graph (map) edge. Includes cost computation.'''
    __slots__ = ('way','dest')
    def __init__(self, w, src, d):
        self.way = w
        self.dest = d
        #cost modified to float for calculation type
        self.cost = float(node_dist(src,d))

class Way():
    ''' A way is an entire street, for drawing, not searching. '''
    __slots__ = ('name','type','nodes')
    # nodes here for ease of drawing only
    def __init__(self,n,t):
        self.name = n
        self.type = t
        self.nodes = []

class Planner():
    __slots__ = ('nodes', 'ways')
    def __init__(self,n,w):
        self.nodes = n
        self.ways = w

    def heur(self,node,cost,gnode):
        '''
        Modified heuristic to allow for inference to elivation implication.
        Will only accept node routes (ways) when elevation between to nodes
        is equal or less than. Not too much of a difference seen on shorter
        routes.
        '''
        global elevDiffPrior
        global distPrior
        elevDiff = node_elev_diff(node, gnode)

        if elevDiff < elevDiffPrior:
            elevDiffPrior = elevDiff

        elif elevDiff == elevDiffPrior:
            elevDiffPrior = elevDiff 
        
        elif elevDiff > elevDiffPrior:
            #Elevation exceptions printed for ALL ways not equal or lesser
            #elevation. Checks ENTIRE map of Oshawa/Whitby, so lots might print
            #depending on how many nodes are branched to from all nodes in
            #path.
            print("***Elevation exception required***")
            if elevDiffPrior == 0:
            #Used to avoid division by zero for finding percentage increase of elevation
            #between edges
                elevDiffPrior = 0.00001
            percent = abs((elevDiff - elevDiffPrior) / elevDiffPrior)
            #incline difference calculated to find how much extra cost
            #the higher the incline the higher the cost for time to traverse
            if percent <= 0.5:
                pass 
            if percent > 0.5:
                cost = cost*1.1
            if percent > 5:
                cost = cost*1.25
            if percent > 10:
                cost = cost*1.5
            elevDiffPrior = elevDiff
        
        #returns distance after elevation cost attributed
        return cost
    
    def Astar(self,s,g,decoy):

        #s is startnode
        #g is goalnode
        parents = {}
        costs = {}
        dQ = PriorityQueue()
        #appends all path potentialities, while loop references and maintains the
        #appropriate final path options based on heuristic assessment
        #decoy is to avoid misappropriated cost analysis (it's 0, heur used properly later)
        dQ.put((self.heur(s,g,decoy),s))
        parents[s] = None
        costs[s] = 0
        while not dQ.empty():
            cf, cnode = dQ.get()
            if cnode == g:
                print " "
                print "Path found and it will cost approx.",int(costs[g]*60/5000),"minutes walking at 5km/hr."                
                print " "
                print "You will need to take the following streets:"
                print " "
                return self.make_path(parents,g)
            for edge in cnode.ways:
                newcost = costs[cnode] + self.heur(edge.dest, edge.cost, g)
                if edge.dest not in parents or newcost < costs[edge.dest]:
                    parents[edge.dest] = (cnode, edge.way)
                    costs[edge.dest] = newcost
                    #None value indicates higher elevation, don't call .put()
                    #becase will cause Tkinter error. Also not necessary to
                    #reference null ways
                    if node_dist(edge.dest, g) is None:
                        pass
                    else:
                        dQ.put((node_dist(edge.dest, g)+newcost,edge.dest))
        
    def make_path(self,par,g):
        nodes = []
        ways = []
        curr = g
        nodes.append(curr)
        while par[curr] is not None:
            prev, way = par[curr]
            ways.append(way.name)
            nodes.append(prev)
            curr = prev
        nodes.reverse()
        ways.reverse()
        return nodes,ways

class PlanWin(Frame):
    '''
    All the GUI pieces to draw the streets, allow places to be selected,
    and then draw the resulting path.
    '''
    
    __slots__ = ('whatis', 'nodes', 'ways', 'elevs', 'nodelab', 'elab', \
                 'planner', 'lastnode', 'startnode', 'goalnode', 'decoy')
    
    def lat_lon_to_pix(self,latlon):
        x = (latlon[1]-LEFTLON)*(TOXPIX)
        y = (TOPLAT-latlon[0])*(TOYPIX)
        return x,y

    def pix_to_elev(self,x,y):
        return self.lat_lon_to_elev(((TOPLAT-(y/TOYPIX)),((x/TOXPIX)+LEFTLON)))

    def lat_lon_to_elev(self,latlon):
        # row is 0 for 43N, 1201 (EPIX) for 42N
        row = abs((int)((43 - latlon[0]) * EPIX))
        # col is 0 for 18 E, 1201 for 19 E
        col = abs((int)((latlon[1]-80) * EPIX))
        #elev = self.elevs.append(row*EPIX+col)
        elev = row*EPIX+col
        return elev
        
    def maphover(self,event):
        self.elab.configure(text = str(self.pix_to_elev(event.x,event.y)))
        for (dx,dy) in [(0,0),(-1,0),(0,-1),(1,0),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            ckpos = (event.x+dx,event.y+dy)
            if ckpos in self.whatis:
                self.lastnode = self.whatis[ckpos]
                lnpos = self.lat_lon_to_pix(self.nodes[self.lastnode].pos)
                self.canvas.coords('lastdot',(lnpos[0]-2,lnpos[1]-2,lnpos[0]+2,lnpos[1]+2))
                nstr = str(self.lastnode)
                nstr += " "
                nstr += str(self.nodes[self.whatis[ckpos]].get_waystr())
                self.nodelab.configure(text=nstr)
                return

    def mapclick(self,event):
        ''' Canvas click handler:
        First click sets path start, second sets path goal 
        '''
        print("Point selected")
        if self.lastnode is None:
            return
        if self.decoy is None:
            self.decoy = self.nodes[self.lastnode]
        if self.startnode is None:
            self.startnode = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.startnode.pos)
            self.canvas.coords('startdot',(self.snpix[0]-2,self.snpix[1]-2,self.snpix[0]+2,self.snpix[1]+2))
        elif self.goalnode is None:
            self.goalnode = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.goalnode.pos)
            self.canvas.coords('goaldot',(self.snpix[0]-2,self.snpix[1]-2,self.snpix[0]+2,self.snpix[1]+2))

    def clear(self):
        ''' Clear button callback. '''
        global elevDiffPrior
        elevDiffPrior = 0
        self.decoy = None
        self.lastnode = None
        self.goalnode = None
        self.startnode = None
        self.canvas.coords('startdot',(0,0,0,0))
        self.canvas.coords('goaldot',(0,0,0,0))
        self.canvas.coords('path',(0,0,0,0))
            
    def plan_path(self):
        ''' Path button callback, plans and then draws path.'''
        print("Planning!")
        print(" ")
        if self.startnode is None or self.goalnode is None:
            print("Sorry, not enough info.")
            return
        print "From OSM point", self.startnode.id, "to OSM point", self.goalnode.id
        nodes,ways = self.planner.Astar(self.startnode, self.goalnode, self.decoy)
        lastway = ""
        for wayname in ways:
            if wayname != lastway:
                print(wayname)
                lastway = wayname
        coords = []
        for node in nodes:
            npos = self.lat_lon_to_pix(node.pos)
            coords.append(npos[0])
            coords.append(npos[1])
        self.canvas.coords('path',*coords)
        
    def __init__(self,master,nodes,ways,coastnodes,elevs):
        self.whatis = {}
        self.nodes = nodes
        self.ways = ways
        self.elevs = elevs
        self.startnode = None
        self.goalnode = None
        self.decoy = None
        self.planner = Planner(nodes,ways)
        thewin = Frame(master)
        w = Canvas(thewin, width=WINWID, height=WINHGT)#, cursor="crosshair")
        w.bind("<Button-1>", self.mapclick)
        w.bind("<Motion>", self.maphover)
        for waynum in self.ways:
            nlist = self.ways[waynum].nodes
            thispix = self.lat_lon_to_pix(self.nodes[nlist[0]].pos)
            if len(self.nodes[nlist[0]].ways) > 2:
                self.whatis[((int)(thispix[0]),(int)(thispix[1]))] = nlist[0]
            for n in range(len(nlist)-1):
                nextpix = self.lat_lon_to_pix(self.nodes[nlist[n+1]].pos)
                self.whatis[((int)(nextpix[0]),(int)(nextpix[1]))] = nlist[n+1]
                w.create_line(thispix[0],thispix[1],nextpix[0],nextpix[1])
                thispix = nextpix
        thispix = self.lat_lon_to_pix(self.nodes[coastnodes[0]].pos)
        # also draw the coast:
        for n in range(len(coastnodes)-1):
            nextpix = self.lat_lon_to_pix(self.nodes[coastnodes[n+1]].pos)
            w.create_line(thispix[0],thispix[1],nextpix[0],nextpix[1],fill="blue")
            thispix = nextpix

        # other visible things are hiding for now...
        w.create_line(0,0,0,0,fill='orange',width=3,tag='path')

        w.create_oval(0,0,0,0,outline='green',fill='green',tag='startdot')
        w.create_oval(0,0,0,0,outline='red',fill='red',tag='goaldot')
        w.create_oval(0,0,0,0,outline='blue',fill='blue',tag='lastdot')
        w.pack(fill=BOTH)
        self.canvas = w

        cb = Button(thewin, text="Clear", command=self.clear)
        cb.pack(side=RIGHT,pady=5)

        sb = Button(thewin, text="Plan!", command=self.plan_path)
        sb.pack(side=RIGHT,pady=5)

        nodelablab = Label(thewin, text="Node:")
        nodelablab.pack(side=LEFT, padx = 5)
        
        self.nodelab = Label(thewin,text="None")
        self.nodelab.pack(side=LEFT,padx = 5)

        elablab = Label(thewin, text="Elev:")
        elablab.pack(side=LEFT, padx = 5)

        self.elab = Label(thewin, text = "0")
        self.elab.pack(side=LEFT, padx = 5)
        
        thewin.pack()


def build_elevs(efilename):
    ''' read in elevations from a file. '''
    efile = open(efilename)
    estr = efile.read()
    elevs = []
    for spot in range(0,len(estr),2):
        elevs.append(struct.unpack('>h',estr[spot:spot+2])[0])
    return elevs

def build_graph(elevs):
    ''' Build the search graph from the OpenStreetMap XML. '''
    tree = ET.parse('Oshawa.osm')
    root = tree.getroot()

    nodes = dict()
    ways = dict()
    waytypes = set()
    coastnodes = []
    for item in root:
        if item.tag == 'node':
            coords = ((float)(item.get('lat')),(float)(item.get('lon')))
            # row is 0 for 43N, 1201 (EPIX) for 42N
            erow = (int)((43 - coords[0]) * EPIX)
            # col is 0 for 18 E, 1201 for 19 E
            ecol = (int)((coords[1]-18) * EPIX)
            try:
                el = elevs[erow*EPIX+ecol]
            except IndexError:
                el = 0
            nodes[(long)(item.get('id'))] = Node((long)(item.get('id')),coords,el)            
        elif item.tag == 'way':
            if item.get('id') == '4055035': #main coastline way ID
                for thing in item:
                    if thing.tag == 'nd':
                        coastnodes.append((long)(thing.get('ref')))
                continue
            useme = False
            oneway = False
            myname = 'unnamed way'
            for thing in item:
                if thing.tag == 'tag' and thing.get('k') == 'highway':
                    useme = True
                    mytype = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'name':
                    myname = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'oneway':
                    if thing.get('v') == 'yes':
                        oneway = True
            if useme:
                wayid = (long)(item.get('id'))
                ways[wayid] = Way(myname,mytype)
                nlist = []
                for thing in item:
                    if thing.tag == 'nd':
                        nlist.append((long)(thing.get('ref')))
                thisn = nlist[0]
                for n in range(len(nlist)-1):
                    nextn = nlist[n+1]
                    nodes[thisn].ways.append(Edge(ways[wayid],nodes[thisn],nodes[nextn]))
                    thisn = nextn
                if not oneway:
                    thisn = nlist[-1]
                    for n in range(len(nlist)-2,-1,-1):
                        nextn = nlist[n]
                        nodes[thisn].ways.append(Edge(ways[wayid],nodes[thisn],nodes[nextn]))
                        thisn = nextn                
                ways[wayid].nodes = nlist
    return nodes, ways, coastnodes

elevs = build_elevs("N43W080.hgt")
nodes, ways, coastnodes = build_graph(elevs)
#coastnodes draw lake and water bodies
#ways 

master = Tk()
thewin = PlanWin(master,nodes,ways,coastnodes,elevs)
mainloop()
