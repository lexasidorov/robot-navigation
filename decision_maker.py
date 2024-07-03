class DecisionMaker():

    def __init__(self):
        pass

    def __extract_distance(self, state):
        dst = {
            'lft': 0,
            'rgt': 0, 
        }
        
        for k in dst.keys():
            ik = 400 if state[f'frw_{k}_ik'] else 10000
            us = state[f'frw_{k}_dst']
            us = 10000 if us == 0 or us == None else us
            dst[k] = (ik + us) / 2

        return dst

    def check_obstackle(self, sens_service_state):
        return random() > 0.9
        # return obstacle_distance['rgt'] < 400 or obstacle_distance['lft'] < 400

    def get_bypass_commands(self, state, to_go, passed):
        return [('lights_on', 1),]
        obstacle_distance = self.__extract_distance(state)
        print(1111, state)
        rgt, lft = obstacle_distance['rgt'], obstacle_distance['lft']
        X = max(to_go - passed - 400 - 500, 0)
        cmds = []
        if rgt < 400 and lft < 400:
            cmds = [('stop', None)]
        elif lft < 400:
            cmds = [('right', 90),('left', 90),('go', 500),('left', 90),('right', 90),('go', X)]
        elif rgt < 400:
            cmds = [('left', 90),('right', 90),('go', 500),('right', 90),('left', 90),('go', X)]

        return cmds
class DescisionMaker():

    def __init__(self):
        pass

    def __extract_distance(self, state):
        dst = {
            'lft': 0,
            'rgt': 0, 
        }
        
        for k in dst.keys():
            ik = 400 if state[f'frw_{k}_ik'] else 10000
            us = state[f'frw_{k}_dst']
            us = 10000 if us == 0 or us == None else us
            dst[k] = (ik + us) / 2

        return dst

    def check_obstackle(self, sens_service_state):
        return random() > 0.9
        # return obstacle_distance['rgt'] < 400 or obstacle_distance['lft'] < 400

    def get_bypass_commands(self, state, to_go, passed):
        return [('lights_on', 1),]
        obstacle_distance = self.__extract_distance(state)
        print(1111, state)
        rgt, lft = obstacle_distance['rgt'], obstacle_distance['lft']
        X = max(to_go - passed - 400 - 500, 0)
        cmds = []
        if rgt < 400 and lft < 400:
            cmds = [('stop', None)]
        elif lft < 400:
            cmds = [('right', 90),('left', 90),('go', 500),('left', 90),('right', 90),('go', X)]
        elif rgt < 400:
            cmds = [('left', 90),('right', 90),('go', 500),('right', 90),('left', 90),('go', X)]

        return cmds


if __name__ == '__main__':
    dm = DescisionMaker()
    print(dm.get_bypass_commands(None, 0, 0))