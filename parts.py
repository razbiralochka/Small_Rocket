import numpy as np

class Fuel_tank:
    def __init__(self, dia, lenght, constr_mass , liquid_type = "LNG"):
        self._diameter  = dia
        self._lenght = lenght
        self._constr_mass = constr_mass

        if liquid_type == "LNG":
            self._liquid_density = 440
        if liquid_type == "LOX":
            self._liquid_density = 1140
        if liquid_type == "Kerosene":
            self._liquid_density = 840
        if liquid_type == "Hydrogen":
            self._liquid_density = 70

        self._liquid_lenght = 0.9*lenght
        self._liquid_volume = self._liquid_lenght*np.pi*(dia**2)/4
        self._liquid_mass = self._liquid_density*self._liquid_volume
        self._total_mass = self._constr_mass + self._liquid_mass

    def part_inertia(self):
        radius = self._diameter/2
        constr_inertia = self._constr_mass*((radius**2)/2+(self._lenght**2)/3)

        offset = self._lenght-self._liquid_lenght/2

        liquid_inertia = self._liquid_mass*((radius**2)/4+(self._liquid_lenght**2)/12+offset**2)

        total_inetria = constr_inertia+liquid_inertia

        return total_inetria

    def part_mass(self):
        return self._total_mass


    def burnout(self, consumption):
        pass



class Engine:
    def __init__(self, imp, cunsuption):
        self._imp = imp
        self._cunsuption = cunsuption
        self._thrust = imp*cunsuption
        a = 10959.9
        b = 0.000165262
        self._mass = a*np.exp(-b*self._thrust)*(np.exp(b*self._thrust)-1)

    def mass(self):
        return self._mass

class Adapter:
    def __init__(self, upper_dim, lower_dim, lenght, mass):
        self._upper_dim = upper_dim
        self._lower_dim = lower_dim
        self._lenght = lenght
        r1 = upper_dim/2
        r2 = lower_dim/2
        self._inertia = (lenght**2)*mass/(r1+3*r2)/(6*(r1+r2))+0.25*mass*(pow(r2, 2)+pow(r1, 2))

    def part_inertia(self):
        return self._inertia

class Fairing:
    def __init__(self, lenght, dim, mass):
        self._lenght = lenght
        self._dim = dim
        self._radius = dim/2
        self._mass = mass
        self._density = self.find_density()

    def find_density(self):
        arg = 100
        f = self.calculate_mass(arg)-self._mass
        while abs(f)>0.01:
            print(f)
            f =self.calculate_mass(arg)
            df = (self.calculate_mass(arg+1e-7)-self.calculate_mass(arg-1e-7))/1e-7
            arg  = arg  - f/df

        return arg

    def calculate_mass(self,arg):
        cords = np.linspace(0,self._lenght, 1000)
        radiuses = self.radius(cords)
        mass = arg*np.trapz(radiuses, cords)
        return mass

    def radius(self,arg):
        res =0.5*((self._radius**2-self._lenght**2)/self._radius+np.sqrt(pow(self._lenght,4)/pow(self._radius,2)
                                                                    +pow(self._radius,2)-2*pow(self._lenght,2)+
                                                                    8*self._lenght*arg-4*pow(arg,2)))
        return res



fairing = Fairing(10,4,100)








