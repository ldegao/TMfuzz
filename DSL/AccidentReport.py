import DSL as sd

class AccidentReport(object):
    def __init__(self):
        self.RoadNetWork = sd.RoadNetWork()
        self.Environment = sd.Environment()
        # 改为dict，key为vehicle name，value为VehicleReport
        self.accidents = {}
        self.Obstacles = sd.Obstacles()


    def set_report(self):
        self.set_roadNetwork()
        self.set_env()
        self.set_vehicles()
        self.set_Obstacles()

    
    def set_roadNetwork(self):
        self.RoadNetWork.set_all()

    def set_env(self):
        self.Environment.set_weather()
    
    def set_vehicles(self):
        # 只允许同一个vehicle出现一次
        v1_name = sd._vehicleDict.name(sd._cid_list[0])
        v2_name = sd._vehicleDict.name(sd._cid_list[1])
        if v1_name not in self.accidents:
            vehicle1 = sd.VehicleReport()
            vehicle1.set_all(sd._cid_list[0])
            self.accidents[v1_name] = vehicle1
        if v2_name not in self.accidents:
            vehicle2 = sd.VehicleReport()
            vehicle2.set_all(sd._cid_list[1])
            self.accidents[v2_name] = vehicle2

    def set_Obstacles(self):
        pass
