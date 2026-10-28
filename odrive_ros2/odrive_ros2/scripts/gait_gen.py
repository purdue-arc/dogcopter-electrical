import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

L1, L2, L3 = 0.1, 0.15, 0.35  # Example link lengths


def inverse_kinematics(x, y, z, L1, L2, L3, side):
    D = (x**2 + y**2 - L1**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta1 = math.atan2(-y, x) - math.atan2(math.sqrt(x ** 2 + y ** 2 - L1 ** 2), -1 * L1)
    
    if (side == 0):
        theta3 = math.atan2(-1*math.sqrt(1-D**2),D) # legs 1 and 3
    else:
        theta3 = math.atan2(math.sqrt(1-D**2),D) # legs 2 and 4

    theta2 = math.atan2(z,math.sqrt(x ** 2 + y ** 2 - L1 ** 2)) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))
    
    return (theta1, theta2, theta3)

def traj_gen():
    time = np.linspace(0, 1, 50)
    T = 0.6
    x_start = -0.2
    y_start = -0.3
    x1 = time * T + x_start
    y1 = time * 0 + (y_start)

    x2 = np.linspace(T + x_start, x_start, 50)
    y2 = 0.075 * np.sin((math.pi * (x2-x_start)) / T) + y_start
    final_x = np.concatenate((x1, x2))
    final_y = np.concatenate((y1, y2))

    return (final_x, final_y)


(x_traj, y_traj) = traj_gen()

# Since the IK code calculates in 3D, we want to essentially throw out the first joint angle. We do this by setting the desired position to have amount of "L1" in the 3rd dimension and then only plotting the x and y angles
result = [inverse_kinematics(L1, y, x, L1, L2, L3, 1) for x, y in zip(x_traj, y_traj)]

# Separated unpacking result onto a separate line for those unfamiliar with syntax
# The code below just ignores 1st element of the returned 3-tuple by inverse_kinematics()
_, x_angles, y_angles = zip(*result)
x_angles = list(np.array(x_angles) - math.pi /2)
y_angles = list(np.array(y_angles))

# View Sample Angles
#print(x_angles)
#print(y_angles)


# Joint 1 Pos
x1 = L2 * np.cos(x_angles)
y1 = L2 * np.sin(x_angles)

# End-effector Pos
x2 = x1 + L3 * np.cos(np.array(x_angles) + np.array(y_angles))
y2 = y1 + L3 * np.sin(np.array(x_angles) + np.array(y_angles))


# Plot Stuff (do not ask me how this works).
fig, ax = plt.subplots()
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.4, 0.25)
ax.set_aspect('equal')

line, = ax.plot([], [], 'o-', lw=2)
trace, = ax.plot([], [], 'r-', lw=1)
trace_x, trace_y = [], []

# Animation update function
def update(frame):
    global trace_x, trace_y
    line.set_data([0, x1[frame], x2[frame]], [0, y1[frame], y2[frame]])
    
    # Update trace
    trace_x.append(x2[frame])
    trace_y.append(y2[frame])
    trace.set_data(trace_x, trace_y)
    
    return line, trace

# Create animation
ani = animation.FuncAnimation(fig, update, frames=len(x_angles), interval=20, blit=True)

plt.show()