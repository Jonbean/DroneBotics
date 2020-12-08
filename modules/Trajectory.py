import numpy as np
from Quadrotor import Quadrotor

class Trajectory:

    def __init__(self, waypoints, T):
        self.waypoints = waypoints
        self.T = T
        self.n = len(waypoints) - 1
        self.z = []
        self.params = [0]
        self.set_param_vals()

    def get_total_dist(self):
        total=0
        for i in range(len(self.waypoints) - 1):
            total += np.linalg.norm(np.array(self.waypoints[i]) - np.array(self.waypoints[i+1]))
        return total


    def set_param_vals(self):

        d_total = self.get_total_dist()

        for i in range(1,self.n):
            d_i = np.linalg.norm(np.array(self.waypoints[i])-np.array(self.waypoints[i-1]))
            self.params.append((d_i/d_total)*self.T + self.params[i-1])
        
        self.params.append(self.T)

    def cubic_spline(self):
   
        self.hs = [self.params[i+1]-self.params[i] for i in range(self.n)]
        self.vs = [2*(self.hs[i-1] + self.hs[i]) for i in range(1, self.n)]

        self.hs = np.array(self.hs)
        self.vs = np.array(self.vs)

        
        for j in range(len(self.waypoints[0])):
 
            bs = [((self.waypoints[i+1][j] - self.waypoints[i][j])/self.hs[i]) for i in range(self.n)]
            us = np.array([6*(bs[i]-bs[i-1]) for i in range(1,self.n)])
            zs = self.solve_equations(us)
            zs = [0.0] + zs.tolist() + [0.0]
            self.z.append(zs)

    def get_current_waypoint(self,t):
        i = 0
        while t >= self.params[i]:
            i += 1
        return i - 1

    def get_pos(self, t):
        i = self.get_current_waypoint(t)
        pos = []
        for j in range(len(self.waypoints[0])):
            elapsed_t = t - self.params[i]
            left_t = self.params[i+1] - t
            a = self.z[j][i+1]/(6*self.hs[i])
            b = self.z[j][i]/(6*self.hs[i])
            c = (self.waypoints[i+1][j] / self.hs[i]) - (self.z[j][i+1] * self.hs[i] / 6)
            d = (self.waypoints[i][j]/self.hs[i]) - (self.z[j][i] * self.hs[i] / 6)
            p = a*(elapsed_t**3) + b*(left_t**3) + c * elapsed_t + d * left_t
            pos.append(p)
        return pos

    def solve_equations(self, us):
        A = np.zeros((self.n-1,self.n-1))
        for i in range(0,self.n-1):
            for j in range(i-1,i+2):
                if j > -1 and j < len(A):
                    if j == i:
                        A[i,j] = self.vs[i]
                    elif j < i:
                        A[i,j] = self.hs[i]
                    else:
                        A[i,j] = self.hs[i+1]
        zs = np.linalg.solve(A,us)
        return zs

            




def main():
    waypoints = [[0,0,5], [5,5,5], [4,-4,5], [4,-4,0]]
    traj = Trajectory(waypoints,3)
    qr = Quadrotor(size=2.0,show_animation=False)
    dt = 0.1
    t = 0
    traj.cubic_spline()
    while t <= traj.T:
        pos = traj.get_pos(t)
        
        print(pos)
        qr.update_pose(pos[0],pos[1],pos[2],0,0,0)
        t += dt
    qr.plot_after(plot_pause=0.05)


if __name__ == "__main__":
    main()