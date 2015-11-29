#////////////////////////Class constuctor//////////////
#.......Boid class 
class Boid(object):
    """ Custom Boid Class"""
    def __init__(self,Pos,Vel,Range,Vision,Alignment,Separation,Cohesion):
        self.pos = Pos
        self.vel = Vel
        self.ran = Range
        self.vis = Vision
        self.Al = Alignment
        self.Se = Separation
        self.Co = Cohesion
        
        initial_boids = Pos
        #New List to store the trails from the boids
        self.Boid_trail = []
        self.Boid_trail.append(initial_boids)
        
    def update(self,others):
        self.flocking(others)
        self.move()
        if Wrap == False:
            self.Boundary()
        elif Wrap == True:
            self.Boundary2()
 
        
    def move(self):
        t,u1,v1 = Terrain.ClosestPoint(self.pos)
        t1,pln = Terrain.FrameAt(u1,v1)
        xaxis = pln.XAxis
        ang1 = Rhino.Geometry.Vector3d.VectorAngle(self.vel,xaxis,pln)
        xaxis.Rotate(-ang1,pln.ZAxis)
        self.vel = xaxis
        self.pos = self.pos + (self.vel*Velocity)
        t,u2,v2 = Terrain.ClosestPoint(self.pos)
        self.pos = Terrain.PointAt(u2,v2)
        self.Boid_trail.append(self.pos)

    def Boundary(self):
        
        t,u,v = Terrain.ClosestPoint(self.pos)
        
        if u >= 1 or u <= 0 :
            self.vel[0] = -self.vel[0]
            self.vel[2] = -self.vel[2]
        if v >= 1 or v <= 0:
            self.vel[1] = -self.vel[1]
            self.vel[2] = -self.vel[2]

            
    def Boundary2(self):
        
        t,u,v = Terrain.ClosestPoint(self.pos)
        if u >= 1:
            self.pos = Terrain.PointAt(0,v)
        if u <= 0:
            self.pos = Terrain.PointAt(1,v)
        if v >= 1:
            self.pos = Terrain.PointAt(u,0)
        if v <= 0 :
            self.pos = Terrain.PointAt(u,1)


    def flocking(self,others):
        closer = self.Get_neighbors(others,self.ran,self.vis)
        cl_agents = closer[0]
        cl_distances = closer[1]
        if len(cl_agents)>0:
            b_acc = Rhino.Geometry.Vector3d(0.0,0.0,0.0)
            cl_center = self.Center(cl_agents)
            Al_vec = self.Align(cl_agents,cl_distances)
            Se_vec = self.Separate(cl_agents,cl_distances)
            Co_vec = self.Cohere(cl_agents)
            b_acc = b_acc+cl_center
            b_acc = b_acc+Al_vec
            b_acc = b_acc+Se_vec
            b_acc = b_acc+Co_vec
            self.vel = self.vel+b_acc
            self.vel = lim_vec(self.vel)

    def Align(self,others, o_dist):
        Al_steer = Rhino.Geometry.Vector3d(0.0,0.0,0.0)
        for i in range(len(others)):
            b = others[i]
            d = o_dist[i]
            if d >0 or d<0:
                dir = Rhino.Geometry.Vector3d
                dir =  b.vel*(self.ran/d)
                Al_steer =  Al_steer + dir
                Al_steer = lim_vec(Al_steer)
                Al_steer = Al_steer*self.Al
        return Al_steer
        
    def Separate(self,others, o_dist):
        desired_sep = 5.00
        Se_steer =Rhino.Geometry.Vector3d(0.0,0.0,0.0)
        for i in range(len(others)):
            b = others[i]
            d = o_dist[i]
            if d > 0 and d <= desired_sep:
                dir = Rhino.Geometry.Vector3d
                dir = self.pos - b.pos
                if d >0 or d<0:
                    dir =  b.vel*(self.ran/d)
                    if dir:
                        Se_steer =  Se_steer + dir
                        Se_steer = lim_vec(Se_steer)
                        Se_steer = Se_steer*self.Se
        return Se_steer
    
    def Cohere(self,others):
        Co_steer = Rhino.Geometry.Vector3d(0.0,0.0,0.0)
        for i in range(len(others)):
            b = others[i]
            Co_steer =Co_steer + b.pos
            Co_steer = Co_steer*(1/len(others))
            d = self.pos.DistanceTo(Co_steer)
            Co_steer = Co_steer-self.pos
            if d >0 or d<0:
                Co_steer = Co_steer*(self.ran/d)
                Co_steer = lim_vec(Co_steer)
                Co_steer = Co_steer*self.Co
        return Co_steer
    
    def Center(self,others):
        sumx,sumy,sumz = 0.0,0.0,0.0
        for other in others:
            n_pos = other.pos
            sumx += n_pos[0]
            sumy += n_pos[1]
            sumz += n_pos[2]
        for other in others:
            n_pos = other.pos
            if n_pos:
                center = Rhino.Geometry.Point3d(sumx/len(others),sumy/len(others),sumz/len(others))
                toward = Rhino.Geometry.Vector3d
                toward = center - n_pos
                toward = lim_vec(toward)
                return toward
            else:
                return Rhino.Geometry.Vector3d(0,0,0)

    def Get_neighbors(self,others,radius,angle):
        Nboidslist = []
        Nboids_dist = []
        for other in others :
            if other is self: 
                continue
            dist = Rhino.Geometry.Vector3d
            dist =  other.pos - self.pos
            orient = Rhino.Geometry.Vector3d.VectorAngle(((self.pos+self.vel)-self.pos),dist)
            distance= dist.Length
            if distance <radius and orient < angle :
                Nboidslist.append(other)
                Nboids_dist.append(distance)
        return Nboidslist,Nboids_dist
