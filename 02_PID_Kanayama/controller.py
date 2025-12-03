import numpy as np

class KanayamaControl:

    def __init__(self, Kx= 10, Ky=64, Ktheta=16):
        #Values from paper, translated to meters
        # Note: pose and state are used interchangeably here
        self.Kx = Kx
        self.Ky = Ky
        self.Ktheta = Ktheta
    
    def compute_control(self, state, ref_state):
        x, y, theta = state
        xr, yr, thetar, vr, omegar = ref_state


        Te = np.array([[np.cos(theta), np.sin(theta), 0],
                       [-np.sin(theta), np.cos(theta), 0],
                       [0, 0, 1]])

        pe = Te @ (ref_state[0:3] - state)
        xe, ye, thetae = pe
        thetae = (thetae + np.pi) % (2 * np.pi) - np.pi

        q = np.array([vr * np.cos(thetae) + self.Kx*xe,
                      omegar + vr*(self.Ky*ye + self.Ktheta*np.sin(thetae))])
        return q
