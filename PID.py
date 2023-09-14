class pid_class():
    def __init__(self, dt_,goal_,kp_,ki_,kd_):
        self._goal = goal_
        self._kp = kp_
        self._ki = ki_
        self._kd = kd_
        self._sumErr = 0
        self._prevErr = 0
        self._dt = dt_
    def gen_signal(self,input):
        err =  input - self._goal
        self._sumErr += err * self._dt
        d_err = (err - self._prevErr)/self._dt
        self._prevErr = err
        signal = self._kp * err + self._ki * self._sumErr + self._kd * d_err



        return signal
    def update_goal(self, goal_):
        self._goal = goal_
