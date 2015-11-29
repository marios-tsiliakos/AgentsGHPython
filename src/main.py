# main routine

if Reset == True:
    agent_pts = []
    agent_crvs = []
    agents_v = []

    if Initial_Boids:
        Agents_run(Initial_Boids)
        counter += 1
        print counter
    else:
        Initial_Boids = Agents_Init(Agents_N)
        print len(agent_pts),"flocking agents"
    Agents_pos = agent_pts
    Trails = agent_crvs
    Agents_View = agents_v

elif Reset == False:
    print("Idle")
    Initial_Boids= []
    counter = 0
