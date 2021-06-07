class PID:

    def __init__(self, p_gain, i_gain, d_gain, uMax=15.0):
        self.last_error = 0.0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.uMax = uMax
        self.i_error = 0.0

    def changegain(self, pgain, igain, dgain):
        self.p_gain = float(pgain)
        self.i_gain = float(igain)
        self.d_gain = float(dgain)

    def Compute(self, input, target, dt, Ui):
        error = -target + input

        p_output = error * self.p_gain
        Ui = Ui + (error) * dt * self.i_gain
        i_output = self.i_error
        d_output = self.d_gain * (error - self.last_error) / dt

        u = p_output + Ui + d_output
        maxU = self.uMax
        if (u > maxU):
            Ui = Ui - self.i_gain * dt * error
            return maxU

        if (u < -self.uMax):
            Ui = Ui - self.i_gain * dt * error
            return -maxU
        self.last_error = error

        return u, Ui
