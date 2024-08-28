# LIDAR Actuator Simulator For Soft Robots

Our goal for this project was to tackle how to implement an actuator simulator for soft robots using SOFA simulation software. In the first few weeks, we focused primarily on understanding what SOFA is, why it is advantageous over other simulation softwares, and taking notes on different components of SOFA features, such as state vectors and solvers. Over the next few weeks, we slowly began installing all the recommended libraries such as SPLIB. 

Next, our main objective was to create a basic cube object and pendulum in SOFA to experiment with the software. With the help of SOFA’s official website tutorials, we created a new scene and using its graph editor, added a cube object and pendulum. We figured out how to combine the cube and pendulum by adding the cube as a child node of the pendulum, and then adjusted the orientation and position of the cube to match the pendulum’s position. Additionally, we explored how to provide input such as torques in the pendulum system. After defining the scene, objects, and components, we learned how to create a Python function which can apply inputs such as torque, even experimenting with constant vs. time-varying torques to visualize their varying effects onto the system.

After we finished this, we attempted to model a collision between two objects by using solvers, which took us a while to complete for a few reasons. The first was learning how to implement a collision detection algorithm to determine when and where objects collide. Another challenge was determining which solver to use such that it is stable and converges to a solution. 

The process used to model this type of collision is summarized here: first, we defined and positioned the objects involved in the collision. Next, we implemented a collision detection algorithm. After, we defined what the material properties (ex. elasticity, friction) of the objects were. We selected the Euler Implicit solver method to integrate equations of motion as it is the default solver in the absence of an explicit specification, as well as more accurate than others.

Lastly, we configured the simulation by defining the time step size and initial conditions. 

# Pendulum Object

This scene consists of three nodes: cubeNode, pendulumNode, and hingeNode.

The cubeNode contains a visual model of a cube, represented by an OglModel component, as well as a MechanicalObject component that defines the degrees of freedom of the cube. The cube is also given a mass, collision detection via the BilateralInteractionConstraint, and a visualization node.

pendulumNode contains a visual model of a pendulum, represented by an OglModel component, as well as a MechanicalObject component that defines the degrees of freedom of the pendulum. The pendulum is also given a mass, collision detection via the BilateralInteractionConstraint, and a visualization node.

hingeNode contains a HingeConstraint that connects the degrees of freedom of the cube and pendulum, creating a hinge joint.

Finally, the root node sets up the simulation parameters, including the solver and constraints. The EulerImplicitSolver is used as the solver, and a CGLinearSolver is used to solve linear systems. The DefaultPipeline is used to handle collisions, and a number of constraints are applied to the dynamics node to simulate the interaction between the cube, pendulum, and ground.

# Two Cubes Collision

In this scene file, there are two cube objects named "Cube1" and "Cube2" that are each defined using a similar set of mechanical components. Each cube has a MechanicalObject to represent its vertices, a UniformMass to give it a mass of 1.0, and a BoxCollisionModel to define its collision shape. Each cube is also fixed to prevent them from moving by adding a FixedConstraint to the first vertex of each MechanicalObject.

The simulation itself is defined in the "root" node, which contains a CollisionPipeline to handle collision detection, a BruteForceDetection to detect collisions between all pairs of objects, a MinProximityIntersection to set the minimum distance for objects to be considered in contact, and a DefaultContactManager to handle the response to collisions. In this case, we use the FrictionContact response with a friction coefficient of 0.1.

Finally, the "root" node contains the two "Cube1" and "Cube2" nodes as child nodes, allowing them to be included in the simulation.

