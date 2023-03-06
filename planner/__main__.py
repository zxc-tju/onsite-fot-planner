from onsite import scenarioOrganizer, env
import time
from fot_planner import FOT
# from lattice.lattice_planner import lattice_planning, prepare_inputs

if __name__ == "__main__":
    
    # 指定输入输出文件夹位置
    input_dir = "./inputs"
    output_dir = "./outputs"
    # 记录测试时间，用于评估效率，没有特殊用途
    tic = time.time()
    # 实例化场景管理模块（ScenairoOrganizer）和场景测试模块（Env）
    so = scenarioOrganizer.ScenarioOrganizer()
    envi = env.Env()
    # 初始化IDM规控器
    # planner = IDM()
    # 根据配置文件config.py装载场景，指定输入文件夹即可，会自动检索配置文件
    so.load(input_dir, output_dir)

    while True:
        # 使用场景管理模块给出下一个待测场景
        scenario_to_test = so.next()
        if scenario_to_test is None:
            break  # 如果场景管理模块给出None，意味着所有场景已测试完毕。
        print("测试:", scenario_to_test)
        # 如果场景管理模块不是None，则意味着还有场景需要测试，进行测试流程。
        # 使用env.make方法初始化当前测试场景，可通过visilize选择是否可视化，默认关闭
        scenario_to_test['test_settings']['visualize'] = True
        observation, traj = envi.make(scenario=scenario_to_test)

        # record initial position
        init_pos = [observation['vehicle_info']['ego']['x'], observation['vehicle_info']['ego']['y']]

        # try:
        "fot"
        # generate a fot planner
        planner = FOT(observation)   
        step = 0
        planner.prepare_data(observation)
        success = planner.planning()
        "end of fot"
        # 当测试还未进行完毕，即观察值中test_setting['end']还是-1的时候
        while observation['test_setting']['end'] == -1:
            "fot"
            step += 1
            planner.prepare_data(observation)
            if step > 0:
                success = planner.planning()
                step = 0
                if success:
                    action = planner.generate_control()
                    # planner.show_frame()
                else:
                    action = planner.generate_control(flag=True)
                    # planner.show_frame()
            else:
                action = planner.generate_control(flag=True)
                # planner.show_frame()
            "end of fot"

            # "lattice"
            # path_point, vehicle_list = prepare_inputs(observation, init_pos)
            # action = lattice_planning(path_point, vehicle_list,show_results=False)
            # "end of lattice"
            
            observation = envi.step(action)  # 根据车辆的action，更新场景，并返回新的观测值。
        # 如果测试完毕，将测试结果传回场景管理模块（ScenarioOrganizer)
        so.add_result(scenario_to_test, observation['test_setting']['end'])
        # except:
        #     continue    
    toc = time.time()
    print(toc - tic)
