from control import *
import sys


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
        value = value.strip()
        
       # Throws an exception if all of the required parameters are not available in dat file 
        if len(value)==0:
            value = None
     
        #sets all variables required to run the controller function
        if variable == 'width':
            if value is not None:
                width = int(value)
            else:
                 sys.exit('The parameter File must contain a widthfor the simulation size')
                    
        elif variable == 'height':
            if value is not None:
                height = int(value)
            else:
                sys.exit('The parameter File must contain a height for the simulation size')
                
        elif variable == 'pedestrians_location':
            if value is not None:
                pedestrian_loc=eval(value)
            else:
                pedestrian_loc=None
            
        elif variable == 'targets_location':
            if value is not None:
                targets_loc=eval(value)
            else:
                targets_loc=None
                
        elif variable == 'obstacles_location':
            if value is not None:
                obstacles_loc=eval(value)
            else:
                obstacles_loc=None
            
        elif variable == 'speed':
            if value is not None:
                speed = float(value)
            else:
                sys.exit('The parameter File must set a speed for the pedestrians')
                
        elif variable == 'max_timesteps':
            if value is not None:
                max_timesteps = int(value)
            else:
                sys.exit('The parameter File must contain the max timesteps')
        
        elif variable == 'devour':
            if value is not None:
                devour = bool(value)
            else:
                sys.exit('The parameter File must set disapearing taget to True or False')
        
        elif variable == 'dijkstra':
            if value is not None:
                dijkstra = bool(value)
            else:
                sys.exit('The parameter File must set the dijkstra algorithm to True or False')
            
            
        elif variable == 'verbose_visualization':
            if value is not None:
                verbose_visualization = bool(value)
            else:
                sys.exit('The parameter File must set the visualisation to True or False')
            
    
        
        
    infile.close()
   
    

    controller = PedestrianController(width, height, pedestrian_loc,  
                                    targets_loc, obstacles_loc, speed, max_timesteps,devour, dijkstra, verbose_visualization)
    controller.init_costs()
    controller.run()

