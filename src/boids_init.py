def Agents_Init(n):
    Boids_population=[]#agent population list

    for i in range(len(n)):
        #instantiate the boid class
        #float(u0)
        #float(v0)
        pos_init= n[i]
        t,u0,v0 = Terrain.ClosestPoint(pos_init)
        pos_init = Terrain.PointAt(u0,v0)
        #print pos_init
        vec_init = random_vel()
        #print vec_init
        Boids_population.append(Boid(pos_init,vec_init,N_distance,N_angle,Al_value,Se_value,Co_value))
    return Boids_population
