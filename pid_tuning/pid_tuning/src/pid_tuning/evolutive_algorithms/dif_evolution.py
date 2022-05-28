#!/usr/bin/env python3
from .abstract_evolutive import *

class DifferentialEvolution(AbstractEvolutive):
    def __init__(self, N: int, m: int, Gm: int, F: float, C: float, A: int, file_path: str, epsilon_1=0.10, stop_error=0.000001, tm = 28800):
        super().__init__(N, m, Gm, A, epsilon_1)
        self.F = F
        self.C = C
        self.stop_error = stop_error
        self.tm = tm
        self.read_json(file_path)
        self.get_trajectories()
        self.set_paths()
        self.set_pubssubs()

    def dif_evolution(self, X: np.ndarray, reset_control: ControlGazebo, rate: Rate):
        """ Arguments:
                @X = population matrix\n
                @reset_control = ControlGazebo object\n
                @rate = rospy.Rate object\n
            Definition:
                Differential Evolution algorithm implementation in its
                variant: "DE/rand/1/bin"\n
            Return:
                Best individual\n
        """
        # evolution 
        g = 0  
        best_worst = 1000
        start = time.time()
        end = start
        while(g <= self.Gm and best_worst >= self.stop_error and (end - start) < self.tm):
            rospy.loginfo("Generation: {}".format(g))
            self.info.generation = g

            #self.info.header.seq = rospy.Time.to_sec()
            ##self.info.header = Header()
            self.info.header.stamp = rospy.Time.now()
            self.info.header.frame_id = "base_link" 
            
            # Generate son's matrix
            U = np.zeros((self.N, self.m+1))
            FO_h = np.zeros((self.N, 1)) 
            U = np.concatenate((U, FO_h.reshape(-1,1)),axis=1)
            U = U.astype("float64")

            for i in range(self.N):
                # generate 3 random integers for selecting the 3 individuals for 1st gen
                r1 = choice([n for n in range(self.N) if n != i])
                r2 = choice([n for n in range(self.N) if n not in [i, r1]])
                r3 = choice([n for n in range(self.N) if n not in [i, r1, r2]])
                # mutant vector
                Vi = np.zeros(self.m)
                Vi = X[r1][0:self.m] + self.F*(X[r2][0:self.m] - X[r3][0:self.m])
                
                for w in range(self.m):
                    if (Vi[w] > self.b[0][w]):
                        Vi[w] = 2*self.b[0][w] - Vi[w]
                    if (Vi[w] < self.a[0][w]):
                        Vi[w] = 2*self.a[0][w] - Vi[w]
                # binomial factor
                Fr = randint(0, self.m-1) 
                for j in range(self.m):
                    # rcj decides if it'll mutate or not
                    rcj = random()
                    # crossing factor
                    if (rcj < self.C) or (j == Fr): 
                        # Copy gen from Vi to son
                        U[i][j] = Vi[j]
                    else:
                        U[i][j] = X[i][j]
            # evaluate sons
            self.evaluate(U, reset_control, rate)
            
            # Fathers vs sons
            for i in range(self.N):
                if self.deb(U[i][:], X[i][:]) == 1:
                    X[i][:] = U[i][:]

            X_sorted = self.deb_bubble_sort(X)
            X_best_new = X_sorted[0, :]
            X_worst = X_sorted[-1, :]
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