import os
import platform
import onsite
from fot_planner import FOT

if __name__ == "__main__":
    
    
    if platform.system() in {'Darwin'}:
        input_dir = os.path.join(os.getcwd(), "../inputs")
        output_dir = os.path.join(os.getcwd(), "../outputs")
    else:
        input_dir = "./inputs"
        output_dir = "./outputs"

    for scenario in os.listdir(input_dir):
        print("Start testing "+scenario)
        env,observation,opendrive,goal,dt = onsite.env.make(input_dir + '/' + scenario, output_dir)
        planner = FOT(observation, goal, dt)  # generate a fot planner 
        step = 0
        planner.prepare_data(observation)
        success = planner.planning()
        while True:
            step += 1
            planner.prepare_data(observation)
            if step > 0:
                success = planner.planning()
                step = 0
                if success:
                    action = planner.generate_control()
                    planner.show_frame()
                else:
                    action = planner.generate_control(flag=True)
                    planner.show_frame()
            else:
                action = planner.generate_control(flag=True)
                planner.show_frame()

            observation,reward,done,info = env.step(action)
            if done:
                print(reward)
                break
