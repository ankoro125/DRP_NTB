import gym
import networkx as nx
import random

### submission information ####
TEAM_NAME = "DRP_NTB_3" 
#TEAM_NAME must be the same as the name registered on the DRP website 
#(or the team name if participating as a team).
##############################

paths = {}
paths_length = {}
H = nx.Graph()
remove_node = []
assigned_agent = []
copy_agent = []

def policy(obs, env):
    global assigned_agent, keys_with_single_element_list
    global copy_agent
    actions = []
    avail_actions = []
    pos_list = env.get_pos_list()  # エージェントの現在情報を取得
    print('pos_list:', pos_list)
    pos_values = [item['pos'] for item in pos_list] # エージェントの現在位置を保存

    #######################################
    # 各エージェント最短ルート計算

    if pos_values == env.start_ori_array:  # エージェントが初期位置にいる場合
        remove_node.clear()
        for agi in range(env.agent_num):
            remove_node.append(env.start_ori_array[agi])
        for agi in range(env.agent_num):
            # エージェントごとに最短経路とその長さを計算
            remove_node.remove(env.start_ori_array[agi])
            try:
                H = env.G.copy() # 仮想環境
                for node in remove_node: # 自身以外のすべてのエージェントの初期位置を除外
                    H.remove_node(node)
                paths[agi] = nx.dijkstra_path(H, env.start_ori_array[agi], env.goal_array[agi], weight='weight')
                paths_length[agi] = nx.dijkstra_path_length(H, env.start_ori_array[agi], env.goal_array[agi], weight='weight')
            except nx.NetworkXNoPath:
                # 経路が見つからなかった場合、初期位置にとどまる
                paths[agi] = [env.start_ori_array[agi]]
                paths_length[agi] = float('inf')
            remove_node.append(env.start_ori_array[agi])
        

        assigned_agent = []
        copy_agent = []

        #######################################
        # 最短距離のエージェント摘出 & ルート確定

        while len(assigned_agent) < env.agent_num:
            # パスの長さを元にエージェントをソート
            paths_length_list = list(paths_length.items())
            sorted_paths_length_list = sorted(paths_length_list,key=lambda x: x[1])
            sorted_keys = [k for k, v in sorted_paths_length_list]
            
            # パスの長さが一番短いエージェントを選択
            for key in sorted_keys:
                if key not in assigned_agent:
                    selected_agent = key
                    break
            
            # 選択したエージェントの初期位置のノードを仮想マップから削除、ゴールノードを追加
            if env.start_ori_array[selected_agent] in remove_node:
                remove_node.remove(env.start_ori_array[selected_agent])
            remove_node.append(paths[selected_agent][-1])
            #remove_node.append(env.goal_array[selected_agent])
            
            # 選択したエージェントの経路を確定
            assigned_agent.append(selected_agent)
            copy_agent.append(selected_agent)
            

            #######################################
            # 他エージェントのルート確定

            # 他のエージェントの経路を再計算
            for agi in range(env.agent_num):
                if agi in assigned_agent:
                    continue
                try:
                    if env.start_ori_array[agi] in remove_node:
                        remove_node.remove(env.start_ori_array[agi])
                    H = env.G.copy()  # 仮想環境            
                    for node in remove_node:
                        H.remove_node(node)
                    paths[agi] = nx.dijkstra_path(H, env.start_ori_array[agi], env.goal_array[agi], weight='weight')
                    paths_length[agi] = nx.dijkstra_path_length(H, env.start_ori_array[agi], env.goal_array[agi], weight='weight')
                    remove_node.append(env.start_ori_array[agi])
                except nx.NetworkXNoPath:        
                    # 経路が見つからなかった場合、初期位置にとどまる           
                    paths[agi] = [env.start_ori_array[agi]]
                    paths_length[agi] = float('inf')
                    remove_node.append(env.start_ori_array[agi])
                    

    #######################################
    #実行
        
    # アクションを決定
    for agi in range(env.agent_num):
        # 現在位置を経路から削除
        if pos_list[agi]['pos'] in paths[agi]:
            paths[agi].remove(pos_list[agi]['pos'])
        
        # 優先順位１位のアクションを決定
        if len(assigned_agent) != 0 and agi == assigned_agent[0]:
            if len(paths[agi]) == 0:
                actions.append(pos_list[agi]['pos'])
                
                assigned_agent.pop(0)
            else:
                actions.append(paths[agi][0])

        elif env.agent_num == 8:
            # 優先順位２～４位のアクションを決定
            if (len(assigned_agent) > 1 and agi == assigned_agent[1]) or (len(assigned_agent) > 2 and agi == assigned_agent[2]) or (len(assigned_agent) > 3 and agi == assigned_agent[3]):
                count = 0
                for i in copy_agent:
                    if agi == i or len(paths[agi]) == 0:
                        break
                    elif 'current_goal' in pos_list[i]:
                        # 現在位置が優先順位１～位のエージェントのゴール地点の場合、道を譲る
                        if pos_list[agi]['pos'] == pos_list[i]['current_goal']:
                            _, avail_actions = env.get_avail_agent_actions(agi,env.n_actions)
                            if avail_actions:
                                action = random.choice(avail_actions)
                                paths[agi].insert(0, action)
                                paths[agi].insert(1, pos_list[agi]['pos'])
                        # 次の経路が優先順位１～位のエージェントの現在位置もしくはゴール地点の場合、現在の位置にとどまる
                        elif paths[agi][0] == pos_list[i]['current_goal'] or paths[agi][0] == pos_list[i]['pos']:
                            pos_value = str(pos_list[agi]['pos'])
                            if len(pos_value) <= 2:
                                actions.append(pos_list[agi]['pos'])
                                count += 1
                            else:
                                actions.append(pos_list[agi]['current_start'])
                                count += 1
                if count == 0:  
                    if len(paths[agi]) != 0:
                        actions.append(paths[agi][0])
                    else:
                        actions.append(pos_list[agi]['pos'])

            else:
                # 優先順位５位以降は現在位置にとどまる
                actions.append(pos_list[agi]['pos'])
            
        elif env.agent_num != 8:
            # 優先順位２～５位のアクションを決定
            if (len(assigned_agent) > 1 and agi == assigned_agent[1]) or (len(assigned_agent) > 2 and agi == assigned_agent[2]) or (len(assigned_agent) > 3 and agi == assigned_agent[3]) or (len(assigned_agent) > 4 and agi == assigned_agent[4]):
                count = 0
                for i in copy_agent:
                    if agi == i or len(paths[agi]) == 0:
                        break
                    elif 'current_goal' in pos_list[i]:
                        # 現在位置が優先順位１～位のエージェントのゴール地点の場合、道を譲る
                        if pos_list[agi]['pos'] == pos_list[i]['current_goal']:
                            _, avail_actions = env.get_avail_agent_actions(agi,env.n_actions)
                            if avail_actions:
                                action = random.choice(avail_actions)
                                paths[agi].insert(0, action)
                                paths[agi].insert(1, pos_list[agi]['pos'])
                        # 次の経路が優先順位１～位のエージェントの現在位置もしくはゴール地点の場合、現在の位置にとどまる
                        elif paths[agi][0] == pos_list[i]['current_goal'] or paths[agi][0] == pos_list[i]['pos']:
                            pos_value = str(pos_list[agi]['pos'])
                            if len(pos_value) <= 2:
                                actions.append(pos_list[agi]['pos'])
                                count += 1
                            else:
                                actions.append(pos_list[agi]['current_start'])
                                count += 1
                if count == 0:  
                    if len(paths[agi]) != 0:
                        actions.append(paths[agi][0])
                    else:
                        actions.append(pos_list[agi]['pos'])

            else:
                # 優先順位５位以降は現在位置にとどまる
                actions.append(pos_list[agi]['pos'])
       
    return actions