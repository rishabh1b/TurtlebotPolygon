#!/usr/bin/env python

import numpy as np               # Linear algebra
import matplotlib.pyplot as plt  # Plotting
import math
import operator

#-------------------------------------------------------------------------------
def shortestObstacleDistance(qobs, q):
    return math.sqrt((q[1] - qobs[1]) ** 2 + (q[0] - qobs[0]) ** 2)

def attractiveForce(qgoal, q):
    gain = 1
    temp = tuple(map(operator.sub, qgoal, q))
    return tuple([gain * x for x in temp])

def repulsiveForce(q, qobsarr):
    threshold = 6
    gain = 1
    repulsiveforce = [(0,0)]
    for j in range(len(qobsarr)):
        d = shortestObstacleDistance(qobsarr[j], q)
        if d < threshold and d > 0:
            mult_factor = gain * (((1 / d) - 1 / threshold) / (d ** 3))
            temp = tuple(map(operator.sub, q, qobsarr[j]))
            r_f = tuple([mult_factor * x for x in temp])
            repulsiveforce[0] = tuple(map(operator.add, repulsiveforce[0], r_f))

    return repulsiveforce

def GoToGoal(x_g, y_g):

  #-----------------------------------------------------------------------------
  # Initialization (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Define a map
  nX = 100
  nY = 100
  X, Y = np.meshgrid(np.linspace(0,nX), np.linspace(0,nY))

  # Define start position
  #x_0 = 30
  #y_0 = 40
  #x_0 = 30
  #y_0 = 10
  x_0 = 50
  y_0 = 10
  #-----------------------------------------------------------------------------
  # Calculate potential field for each cell of the map
  #-----------------------------------------------------------------------------
  q_obs_arr = [(50,20), (60, 30)]
  q_goal = (x_g, y_g)
  U = np.zeros((101,101))
  V = np.zeros((101,101))
  x_range = list(reversed(range(101)))
  y_range = range(101)
  for i in x_range:
      for j in y_range:
          att_force = attractiveForce(q_goal, (j, 100 - i))
          repul_force = repulsiveForce((j, 100 - i), q_obs_arr)
          att_force_mag = math.sqrt(att_force[0]**2 + att_force[1]**2)
          repul_force_mag = math.sqrt(repul_force[0][0] ** 2 + repul_force[0][1] ** 2)
          if att_force_mag > 0:
              att_force_x = att_force[0] / att_force_mag
              att_force_y = att_force[1] / att_force_mag
          else:
              att_force_x = 0
              att_force_y = 0
          if repul_force_mag > 0:
              repul_force_x = repul_force[0][0] / repul_force_mag
              repul_force_y = repul_force[0][1] / repul_force_mag
          else:
              repul_force_x = 0
              repul_force_y = 0
          U[i, j] = att_force_x + repul_force_x
          V[i, j] = att_force_y + repul_force_y

  # A dummy potential field
  #U = np.cos(X)
  #V = np.sin(Y)

  #-----------------------------------------------------------------------------
  # Find the robot path using the potential field
  #-----------------------------------------------------------------------------
  # Your final path should be represented as an ordered list of cell coordinates
  # the robot visits

  path = []
  path.append([x_0, y_0])
  max_its = 2000
  curr_pos = [x_0, y_0]
  for i in range(max_its):
      deltax = U[100 - int(round(curr_pos[1])), int(round(curr_pos[0]))]
      deltay = V[100 - int(round(curr_pos[1])), int(round(curr_pos[0]))]
      curr_pos[0] = curr_pos[0] + deltax
      curr_pos[1] = curr_pos[1] + deltay
      path.append([curr_pos[0], curr_pos[1]])
      if math.sqrt((curr_pos[1] - q_goal[1]) ** 2 + (curr_pos[0] - q_goal[0]) ** 2) < 0.1:
          break

  # A dummy path
  """
  for x in range(30):
    path.append([x_0+x+1, y_0])
  for y in range(21):
    path.append([x_0+30, y_0+y+1])
  """

  #-----------------------------------------------------------------------------
  # Plot results (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Plot potential field. Every nth cell will be plotted
  nth = 1
  plt.figure()
  Q = plt.quiver(X[::nth, ::nth], Y[::nth, ::nth], U[::nth, ::nth], V[::nth, ::nth],
        pivot='mid', units='width')
  plt.axis([-5, 105, -5, 105])
  plt.title('Robot path')
  plt.xlabel('X')
  plt.ylabel('Y')

  # Plot Path
  path_x = []
  path_y = []
  for i in range(len(path)-1):
    path_x.append(path[i][0])
    path_y.append(path[i][1])

  plt.plot(path_x, path_y, 'r', linewidth=4)

  # Plot Start and goal positions
  plt.plot([x_0], [y_0], 'bo', markersize=10)
  plt.plot([x_g], [y_g], 'go', markersize=10)
  plt.plot([50], [20], 'mo', markersize=30)
  plt.plot([60], [30], 'mo', markersize=30)

  # Show plot
  plt.show()


#-------------------------------------------------------------------------------
if __name__ == '__main__':
  #x_g = 50
  #y_g = 40
  x_g = 60
  y_g = 40
  #x_g = 70
  #y_g = 30
  GoToGoal(x_g, y_g)
