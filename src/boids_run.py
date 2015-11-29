def Agents_run(Boids_population):
    for boid in Boids_population:
        
        #Make global variable local and dynamic
        boid.Al = Al_value
        boid.Se = Se_value
        boid.Co = Co_value
        boid.vis = N_angle 
        boid.ran = N_distance 
        
        boid.update(Boids_population)
        agent_pts.append(boid.pos) 
        agents_v.append(boid.vel)
        if len(boid.Boid_trail)>3 and Display == True:
            crv = Rhino.Geometry.Curve.CreateInterpolatedCurve(boid.Boid_trail,3)
            agent_crvs.append(crv)
