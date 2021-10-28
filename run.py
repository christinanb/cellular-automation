from control import *


def runsim(filename):
    """
   runsim() Parses data from a .dat input file and runs controller.PedestrianController to begin 
    a simulation with parameters provided from the input file. 

    """
    
    infile = open(filename, 'r')
    for line in infile:
        # Typical line: variable = value
        variable, value = line.split('=')
        variable = variable.strip() 
        
        if variable == 'width':
            width = int(value)
        elif variable == 'height':
            height = int(value)
        elif variable == 'pedestrians_location':
            pedestrian_loc=list(eval(value)) 
            if len(pedestrian_loc)==1:
               pedestrian_loc=[eval(value)]
        elif variable == 'targets_location':
            targets_loc=list(eval(value))
            if len(targets_loc)==1:
               targets_loc=[eval(value)]
        elif variable == 'obstacles_location':
            obstacles_loc=list(eval(value))
            if len(obstacles_loc)==1:
               obstacles_loc=[eval(value)]
        elif variable == 'speed':
            speed = float(value)
        elif variable == 'max_timesteps':
            max_timesteps = int(value)
        elif variable == 'devour':
            devour = bool(value)
        elif variable == 'dijkstra':
            dijkstra = bool(value)
        elif variable == 'verbose_visualization':
            verbose_visualization = bool(value)
    infile.close()

    
    controller = PedestrianController(width, height, pedestrian_loc,  
                                    targets_loc, obstacles_loc, speed, max_timesteps,devour, dijkstra, verbose_visualization)
    controller.init_costs()
    controller.run()

