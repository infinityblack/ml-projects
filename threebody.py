from manim import *
import numpy as np

# Define gravitational constant
G = 6.67430e-11  # m^3⋅kg^−1⋅s^−2 (in standard units)

# Define masses and initial positions of the bodies
masses = [5e24, 5e24, 5e24]  # masses of the bodies (in kg)
positions = [np.array([3, 1, 0], dtype=np.float64), 
             np.array([-3, -1, 0], dtype=np.float64), 
             np.array([0, 3, 0], dtype=np.float64)]  # initial positions (in meters)
velocities = [np.array([0, 1, 0], dtype=np.float64), 
              np.array([0, -1, 0], dtype=np.float64), 
              np.array([1, 0, 0], dtype=np.float64)]  # initial velocities (in m/s)

# Time step for simulation
dt = 0.1  # seconds

# Function to compute the force between two bodies
def compute_gravitational_force(r1, r2, m1, m2):
    r = r2 - r1
    distance = np.linalg.norm(r)
    if distance == 0:
        return np.array([0, 0, 0], dtype=np.float64)  # Avoid division by zero
    force_magnitude = G * m1 * m2 / (distance ** 2)
    return force_magnitude * r / distance  # Return force vector

# Manim scene
class ThreeBodyProblemScene(ThreeDScene):
    def construct(self):
        # Create spheres to represent the bodies
        spheres = []
        for pos in positions:
            sphere = Sphere(radius=0.3, color=BLUE)
            sphere.move_to(pos)
            spheres.append(sphere)
            self.add(sphere)

        # Create trails for visualization
        trails = [Line(start=pos, end=pos, color=WHITE) for pos in positions]
        for trail in trails:
            self.add(trail)

        # Update function for motion simulation
        def update_position(mob, index):
            # Compute the net force on each body
            forces = np.zeros((3, 3), dtype=np.float64)  # Forces on all bodies
            for i in range(3):
                if i != index:
                    forces[index] += compute_gravitational_force(positions[index], positions[i], masses[index], masses[i])

            # Update position and velocity
            acceleration = forces[index] / masses[index]
            velocities[index] += acceleration * dt
            positions[index] += velocities[index] * dt
            mob.move_to(positions[index])

            # Update the trail using np.append
            trails[index].points = np.append(trails[index].points, [positions[index]], axis=0)

        # Animate the movement
        for t in range(200):
            self.wait(0.1)
            for i in range(3):
                update_position(spheres[i], i)

# To run the animation, use the following command in your terminal:
# manim -pql three_body_problem.py ThreeBodyProblemScene
