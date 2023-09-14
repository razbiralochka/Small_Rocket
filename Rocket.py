import numpy as np

class stage_class:

    # Engines Features
    def engines_set(self, thrust=0, imp=0):
        self._thrust = thrust
        self._gas_flow_speed = imp
        self._consumption = thrust /imp

    # Fuel_Features
    def fuel_set(self,fuel_mass=0, LOX_density=0, LNG_density=0, LNG_to_LOX_ratio=0):
        self._fuel_mass = fuel_mass
        self._LOX_density = LOX_density
        self._LNG_density = LNG_density
        self._LNG_to_LOX_ratio = LNG_to_LOX_ratio
        self._LOX_mass = fuel_mass*(LNG_to_LOX_ratio)/(1+LNG_to_LOX_ratio)
        self._LNG_mass = fuel_mass / (1 + LNG_to_LOX_ratio)
        self._LOX_volume = self._LOX_mass/LOX_density
        self._LNG_volume = self._LNG_mass/LNG_density

    #Mass_Features
    def mass_set(self, start_mass=0):
        self._const_mass = start_mass - self._fuel_mass
        self._wk = self._const_mass/2700

    #Geometry_Features
    def geometry_set(self,dimeter=0, stage_lenght=0, separator_lenght=0):
        self._dimeter = dimeter
        self._stage_lenght = stage_lenght
        self._stage_lenght = stage_lenght
        self._separator_lenght = separator_lenght
        self._LNG_lenght = 4*self._LNG_volume/(np.pi*pow(dimeter,2))
        self._LOX_lenght = 4 * self._LOX_volume / (np.pi * pow(dimeter, 2))
        self._LNG_tank_lengh = self._LNG_lenght/0.9
        self._LOX_tank_lengh = self._LOX_lenght / 0.9

    def LNG_lenght(self):
        return self._LNG_lenght
    def LOX_lenght(self):
        return self._LOX_lenght
    def LNG_tank_lengh(self):
        return self._LNG_tank_lengh
    def LOX_tank_lengh(self):
        return self._LOX_tank_lengh
    def separator_lenght(self):
        return self._separator_lenght
    def const_mass(self):
        return self._const_mass
    def diameter(self):
        return self._dimeter
    def lenght(self):
        return self._stage_lenght
class fairing_class():
    def __init__(self, diameter = 0, lenght = 0):
        self._lenght = lenght
        self._diameter = diameter
    def lenght(self):
        return self._lenght


class Rocket_assembly:
    def __init__(self):
        self._stage = list()
    def add_stage(self, stage):
        self._stage.append(stage)
    def add_fairing(self,obj):
        self._fairing = obj


    #Требует пояснения
    def upd_masses(self):
        K_LOX = np.zeros(2)
        K_LNG = np.zeros(2)
        K = np.zeros(2)

        K21o = self._fairing.lenght() + self._stage[1].LOX_tank_lengh() - self._stage[1].LOX_lenght()
        K22o = self._fairing.lenght() + self._stage[1].LOX_tank_lengh()
        K_LOX[1] = K21o + K22o

        K21g = K22o + self._stage[1].LNG_tank_lengh() - self._stage[1].LNG_lenght()
        K22g = K22o + self._stage[1].LNG_tank_lengh()
        K_LNG[1] = K21g + K22g

        K11o = K22g + self._stage[1].separator_lenght() + self._stage[0].LOX_tank_lengh() - self._stage[0].LOX_lenght()
        K12o = K22g + self._stage[1].separator_lenght() + self._stage[0].LOX_tank_lengh()
        K_LOX[0] = K11o + K12o

        K11g = K12o + self._stage[0].LOX_tank_lengh() - self._stage[0].LOX_lenght()
        K12g = K12o + self._stage[0].LOX_tank_lengh()
        K_LNG[0] = K11g + K12g

        K[1] = K21o + K22g
        K[0] = K11o + K12g

        #Стат момемнт
        const_stat_moment = [0.5 * K[i]*self._stage[i].const_mass() for i,val in enumerate(self._stage)]
        const_stat_moment_sum=sum(const_stat_moment)

        #Момент инецрии
        constr_inertia=[0.25 * (K[i] ** 2) + (self._stage[i].diameter() / 2) ** 2 + 0.333 * (self._stage[i].lenght() ** 2) *
                    self._stage[i].const_mass() for i,val in enumerate(self._stage)]

        constr_inertia_sum = sum(constr_inertia)



class rocket():
    def __init__(self):

        self.body = Rocket_assembly()

        stage = stage_class()

        stage.engines_set(thrust=395900,
                          imp=3300)

        stage.fuel_set(fuel_mass=25120,
                       LOX_density=1140,
                       LNG_density=440,
                       LNG_to_LOX_ratio=3.5)

        stage.mass_set(start_mass=27640)

        stage.geometry_set(dimeter=1.7,
                           stage_lenght=17.4,
                           separator_lenght=2.8)
        self.body.add_stage(stage)


        stage.engines_set(thrust=58800,
                          imp=3300)

        stage.fuel_set(fuel_mass=4440,
                       LOX_density=1140,
                       LNG_density=440,
                       LNG_to_LOX_ratio=3.5)

        stage.mass_set(start_mass=5000)

        stage.geometry_set(dimeter=1.7,
                           stage_lenght=4.7,
                           separator_lenght=2.1)
        self.body.add_stage(stage)

        fairing = fairing_class(diameter=1.7, lenght=3.5)

        self.body.add_fairing(fairing)

        self.body.upd_masses()