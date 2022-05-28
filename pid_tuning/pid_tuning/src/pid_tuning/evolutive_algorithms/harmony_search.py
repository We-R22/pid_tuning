#!/usr/bin/env python3
from .abstract_evolutive import *

class HarmonySearch(AbstractEvolutive):
    def __init__(self, N, m, Gm, A, r_accept, r_pa, file_path, bw = False, epsilon_1=0.1, stop_error = 0.000001, tm=30):
        super().__init__(N, m, Gm, A, epsilon_1)
        self.r_accept = r_accept
        self.r_pa = r_pa
        self.bw = bw
        self.stop_error = stop_error
        self.tm = tm
        self.read_json(file_path)
        self.get_trajectories()
        self.set_paths()
        self.set_pubssubs()
    
    def ind_eval(self, P: np.ndarray, reset_control: ControlGazebo, rate: Rate):
        """ Arguments:
                @P = population matrix\n
                @reset_control = ControlGazebo object\n
                @rate = rospy.Rate object\n
            Description:
                Sends joint trajectories to roscontrol topic /command and evaluates
                individual error and accumulates it in joint space and 
                cartesian space.\n
            Return:
                None\n
        """
        # pauses simulation
        reset_control.pause()
        for i in range(self.A):
            params = {'p' : P[i*3], 'i' : P[i*3+1], 'd' : P[i*3+2]}
            config = self.dic_cli["client{}".format(i+1)].update_configuration(params)
            self.errors[i] = 0.0

            self.g1_x = 0.0
            self.g1_y = 0.0
            self.g1_z = 0.0

        # unpause simulation
        reset_control.unpause()

        # publishes each element of the trajectories
        length = len(self.trajectories)
        
        for i in range(length):
            svr_c = i
            for j in range(self.A): 
                self.pubs["pub{}".format(j+1)].publish(self.trajectories['q{}'.format(j+1)][i])
            rate.sleep()

            self.g1_x += abs(self.x_d[svr_c] - self.x_o)
            self.g1_y += abs(self.y_d[svr_c] - self.y_o)
            self.g1_z += abs(self.z_d[svr_c] - self.z_o)
            
        for i in range(self.A):
            self.pubs["pub{}".format(i+1)].publish(0.0)
        rate.sleep()

        # restarts and pauses simulation
        reset_control.init_values()
        reset_control.pause()
        # add error to individual (m = 3 (pos3 0,1,2,3))
        P[self.m] = sum(self.errors)

        # scv function
        P[-1] = 0.0
        P[-1] = self.scv()

        self.info.header.stamp = rospy.Time.now()
        self.info.header.frame_id = "base_link"

        self.info.individual = 1
        self.info.genes = P[0:-2]
        self.info.of = P[-2]
        self.info.scv = P[-1]
        
        self.pub.publish(self.info)

    def harmony_search(self, X: np.ndarray, reset_control: ControlGazebo, rate: Rate):
        """ Arguments:
                @X = harmony memory\n
                @reset_control = ControlGazebo object\n
                @rate = rospy.Rate object\n
            Description:
                Harmony Search algorithm implementation in its
                variant: "modified to handle numerical constraints
                using the feasiability rules of Deb"\n                
            Return:
                Best harmony\n
        """
        g = 0
        best_worst = 1000
        start = time.time()
        end = start
        while (g<= self.Gm and best_worst >= self.stop_error and (end - start) < self.tm):
            rospy.loginfo("Generation: {}".format(g))
            self.info.generation = g

            #self.info.header.seq = rospy.Time.
            ##self.info.header = Header()
            self.info.header.stamp = rospy.Time.now()
            self.info.header.frame_id = "base_link"            

            X_new = np.zeros(self.m+2)

            for j in range(self.m):
                if random() < self.r_accept:
                    index = randint(0, self.N-1)
                    if random() < self.r_pa:
                        if not self.bw:
                            self.bw = (self.b[0][j] - self.a[0][j]) / 1000

                        X_new[j] = X[index][j] + self.bw * choice([n for n in range(-1, 2) if n != 0])
                    else:
                        X_new[j] = X[index][j]
                else:
                    X_new[j] = (self.b[0][j]- self.a[0][j])*np.random.random_sample() - self.a[0][j]
    
                if (X_new[j] > self.b[0][j]):
                    X_new[j] = 2*self.b[0][j] - X_new[j]
                if (X_new[j] < self.a[0][j]):
                    X_new[j] = 2*self.a[0][j] - X_new[j]

            self.ind_eval(X_new, reset_control, rate)

            X = self.deb_bubble_sort(X)
            if self.deb(X_new, X[-1, :]) == 1:
                X[-1, :] = X_new

            X_best_new = X[0, :]
            X_worst = X[-1, :]
            best_worst = abs(X_best_new[-2] - X_worst[-2])

            if (g == 0):
                X_best = X_best_new

            elif(g > 0 and self.deb(X_best_new, X_best) == 1):
                X_best = X_best_new

            g += 1
            end = time.time()

            rospy.loginfo("X_best: {}".format(X_best))

        rospy.loginfo("X_best founded: {}".format(X_best))
        return X_best