from Point import Point
import math


# class Agent:
#     # constant
#     k = 12000
#     kappa = 24000
#     v_max = 5
#     time = .01
#     _index = 0

#     def __init__(self, size, mass, pos, goals, desiredSpeed=4):
#         # the constants
#         self.A = 180
#         self.B = 0.08
#         self.tau = 0.5
#         # instance variables
#         self.size = size  # radius
#         self.mass = mass
#         self.pos = pos  # current position: Point object
#         self.velocity = Point(0, 0)  # current velocity: Point object
#         self.desiredSpeed = desiredSpeed  # preferred speed: float
#         self.goals = goals  # exit: Goal object

#         self.index = Agent._index
#         Agent._index += 1

#     @property
#     def desiredDirection(self):
#         """
#         Calculates the vector pointing towards the goal.
#         """
#         num_goals = len(self.goals)
#         if num_goals >= 1:
#             # for goal 1
#             goal1 = self.goals[0]
#             p11 = goal1.parameters['p1']
#             p21 = goal1.parameters['p2']
#             midpt1 = (p11.__add__(p21)).__mul__(1/2)
#             if num_goals >= 2:
#                 # for goal 2
#                 goal2 = self.goals[1]
#                 p12 = goal2.parameters['p1']
#                 p22 = goal2.parameters['p2']
#                 midpt2 = (p12.__add__(p22)).__mul__(1/2)
#                 if num_goals >= 3:
#                     # for goal 3
#                     goal3 = self.goals[2]
#                     p13 = goal3.parameters['p1']
#                     p23 = goal3.parameters['p2']
#                     midpt3 = (p13.__add__(p23)).__mul__(1/2)
#         if num_goals == 1:
#             #go to goal 1
#             # If past the goal move right
#             if self.pos.x >= p11.x:
#                 return Point(1, 0)
#             # If above the goal, move to top point
#             elif self.pos.y - self.size < p11.y:
#                 return self.vectorTo(p11 + Point(0, .5)).norm()
#             # If below the goal, move to bottom point
#             elif self.pos.y + self.size > p21.y:
#                 return self.vectorTo(p21 - Point(0, .5)).norm()
#             # If directly in front of the goal, move right
#             else:
#                 return Point(1, 0)
#         elif num_goals == 2:
#             #go to goal 1
#             if self.pos.__dist__(midpt1) <= self.pos.__dist__(midpt2):
#                 # If past the goal move right
#                 if self.pos.x >= p11.x:
#                     return Point(1, 0)
#                 # If above the goal, move to top point
#                 elif self.pos.y - self.size < p11.y:
#                     return self.vectorTo(p11 + Point(0, .5)).norm()
#                 # If below the goal, move to bottom point
#                 elif self.pos.y + self.size > p21.y:
#                     return self.vectorTo(p21 - Point(0, .5)).norm()
#                 # If directly in front of the goal, move right
#                 else:
#                     return Point(1, 0)
#             #go to goal2
#             else:
#                 # If past the goal move bottom
#                 if self.pos.y >= p12.y:
#                     return Point(0, 1)
#                 # If left of the goal, move to left point
#                 elif self.pos.x - self.size < p12.x:
#                     return self.vectorTo(p12 + Point(.5, 0)).norm()
#                 # If right of the goal, move to right point
#                 elif self.pos.x + self.size > p22.x:
#                     return self.vectorTo(p22 - Point(.5, 0)).norm()
#                 # If directly in front of the goal, move bottom
#                 else:
#                     return Point(0, 1)
#         elif num_goals == 3:
#                 #go to goal 1
#             if self.pos.__dist__(midpt1) <= self.pos.__dist__(midpt2) and self.pos.__dist__(midpt1) <= self.pos.__dist__(midpt3):
#                 # If past the goal move right
#                 if self.pos.x >= p11.x:
#                     return Point(1, 0)
#                 # If above the goal, move to top point
#                 elif self.pos.y - self.size < p11.y:
#                     return self.vectorTo(p11 + Point(0, .5)).norm()
#                 # If below the goal, move to bottom point
#                 elif self.pos.y + self.size > p21.y:
#                     return self.vectorTo(p21 - Point(0, .5)).norm()
#                 # If directly in front of the goal, move right
#                 else:
#                     return Point(1, 0)
#             #go to goal2
#             elif self.pos.__dist__(midpt2) <= self.pos.__dist__(midpt1) and self.pos.__dist__(midpt2) <= self.pos.__dist__(midpt3):
#                 # If past the goal move bottom
#                 if self.pos.y >= p12.y:
#                     return Point(0, 1)
#                 # If left of the goal, move to left point
#                 elif self.pos.x - self.size < p12.x:
#                     return self.vectorTo(p12 + Point(.5, 0)).norm()
#                 # If right of the goal, move to right point
#                 elif self.pos.x + self.size > p22.x:
#                     return self.vectorTo(p22 - Point(.5, 0)).norm()
#                 # If directly in front of the goal, move bottom
#                 else:
#                     return Point(0, 1)
#             #go to goal 3
#             elif self.pos.__dist__(midpt3) <= self.pos.__dist__(midpt1) and self.pos.__dist__(midpt3) <= self.pos.__dist__(midpt2):
#                 # If past the goal move left
#                 if self.pos.x <= p13.x:
#                     return Point(-1, 0)
#                 # If above the goal, move to top point
#                 elif self.pos.y - self.size < p13.y:
#                     return self.vectorTo(p13 - Point(0, -.5)).norm()
#                 # If below the goal, move to bottom point
#                 elif self.pos.y + self.size > p23.y:
#                     return self.vectorTo(p23 + Point(0, -.5)).norm()
#                 # If directly in front of the goal, move left
#                 else:
#                     return Point(-1, 0)

#         # if self.pos.__dist__(midpt1) <= self.pos.__dist__(midpt2):
#         #     # If past the goal move right
#         #     if self.pos.x >= p11.x:
#         #         return Point(1, 0)
#         #     # If above the goal, move to top point
#         #     elif self.pos.y - self.size < p11.y:
#         #         return self.vectorTo(p11 + Point(0, .5)).norm()
#         #     # If below the goal, move to bottom point
#         #     elif self.pos.y + self.size > p21.y:
#         #         return self.vectorTo(p21 - Point(0, .5)).norm()
#         #     # If directly in front of the goal, move right
#         #     else:
#         #         return Point(1, 0)
#         # else:
#         #     # If past the goal move bottom
#         #     if self.pos.y >= p12.y:
#         #         return Point(0, 1)
#         #     # If left of the goal, move to left point
#         #     elif self.pos.x - self.size < p12.x:
#         #         return self.vectorTo(p12 + Point(.5, 0)).norm()
#         #     # If right of the goal, move to right point
#         #     elif self.pos.x + self.size > p22.x:
#         #         return self.vectorTo(p22 - Point(.5, 0)).norm()
#         #     # If directly in front of the goal, move bottom
#         #     else:
#         #         return Point(0, 1)

#     def vectorTo(self, point):
#         return point - self.pos

#     def move(self, force):
#         """ update step - move toward goal during unit time """
#         self.pos = self.pos + self.velocity * Agent.time
#         self.velocity = self.velocity + force / self.mass * Agent.time

#         # apply speed limit
#         speed = self.velocity.mag()
#         if speed > Agent.v_max:
#             self.velocity = self.velocity.norm() * Agent.v_max

#     # the force generated by the agent self
#     def selfDriveForce(self):
#         desiredVelocity = self.desiredDirection * self.desiredSpeed
#         return (desiredVelocity - self.velocity) * (self.mass / self.tau)

class Agent:
    # constants
    k = 12000
    kappa = 24000
    v_max = 5
    time = 0.01
    _index = 0

    def __init__(self, size, mass, pos, goals, fires, desiredSpeed=4):
        # instance variables
        self.A = 180
        self.B = 0.08
        self.tau = 0.5
        self.size = size  # radius
        self.mass = mass
        
        self.pos = pos  # current position: Point object
        self.velocity = Point(0, 0)  # current velocity: Point object
        self.fires = fires
        self.desiredSpeed = desiredSpeed  # preferred speed: float
        self.goals = goals  # exit: Goal object

        self.index = Agent._index
        Agent._index += 1
        
    @property
    def desiredDirection(self):
        """Calculate the desired direction based on current position and goals."""
        midpoints = self._get_midpoints()
        distances = [self.pos.__dist__(mid) for mid in midpoints]
        closest_goal_index = distances.index(min(distances))
        
        goal = self.goals[closest_goal_index]
        p1, p2 = goal.parameters['p1'], goal.parameters['p2']
        goal_direction = self._get_direction_to_goal(p1, p2)

        # Check if there is fire nearby and adjust direction accordingly
        fire_avoidance_vector = self.avoidFire()

        # Combine both directions: moving towards the goal and avoiding fire
        combined_direction = goal_direction + fire_avoidance_vector
        return combined_direction.norm() 

    def avoidFire(self):
        """Calculate the repulsive vector to avoid fire."""
        avoidance_vector = Point(0, 0)
        fire_repulsion_strength = 500  # Arbitrary factor to control avoidance strength
        
        # for fire in self.fires:
        distance_to_fire = self.pos.__dist__(list(self.fires.positions)[0])
        if distance_to_fire < 10:  # Set a threshold distance for fire avoidance
            direction_from_fire = (self.pos - list(self.fires.positions)[0]).norm()
            repulsion_magnitude = fire_repulsion_strength / distance_to_fire**2
            avoidance_vector += direction_from_fire * repulsion_magnitude
        
        return avoidance_vector
    
    def _get_midpoints(self):
        """Calculate the midpoints of all goals."""
        midpoints = []
        for goal in self.goals:
            p1 = goal.parameters['p1']
            p2 = goal.parameters['p2']
            midpoints.append((p1.__add__(p2)).__mul__(0.5))
        return midpoints

    def _get_direction_to_goal(self, p1, p2):
        """Determine the movement direction towards the specified goal."""
        if self.pos.x >= p1.x:
            return Point(1, 0)  # Move right if past the goal
        elif self.pos.y - self.size < p1.y:
            return self.vectorTo(p1 + Point(0, 0.5)).norm()  # Move to top
        elif self.pos.y + self.size > p2.y:
            return self.vectorTo(p2 - Point(0, 0.5)).norm()  # Move to bottom
        return Point(1, 0)  # Default move right

    def vectorTo(self, point):
        return point - self.pos

    def move(self, force):
        """Update the agent's position and velocity based on applied force."""
        self.pos = self.pos + self.velocity * Agent.time
        self.velocity += force / self.mass * Agent.time
        # Apply speed limit
        if self.velocity.mag() > Agent.v_max:
            self.velocity = self.velocity.norm() * Agent.v_max

    def selfDriveForce(self):
        """Calculate the self-driving force of the agent."""
        desiredVelocity = self.desiredDirection * self.desiredSpeed
        return (desiredVelocity - self.velocity) * (self.mass / self.tau)
    # the force between the agent and an part of wall
    def wallForce(self, wall):
        if wall.wallType == 'line' or wall.wallType == 'maze':
            p1 = wall.parameters["p1"]
            p2 = wall.parameters["p2"]
            r = self.vectorTo(p1)
            wallLine = (p2 - p1).norm()
            normalUnitVector = Point(-wallLine.y, wallLine.x)  # rotate 90 degrees counterclockwise
            temp = dotProduct(r, normalUnitVector)
            if temp > 0:  # perpendicular
                normalUnitVector.x *= -1
                normalUnitVector.y *= -1
            elif temp == 0:
                normalUnitVector = -r.norm()

            if dotProduct(self.velocity, wallLine) >= 0:
                tangentUnitVector = wallLine
            else:
                tangentUnitVector = -wallLine
            distance = -dotProduct(r, normalUnitVector)

            # if wall cannot apply force on agent
            #  (in the case of that intersection between wall and normal vector is not on wall.)
            perpendicularPoint = self.pos + normalUnitVector * distance
            if dotProduct(p1 - perpendicularPoint, p2 - perpendicularPoint) > 0:
                nearDistance = min(self.vectorTo(p1).mag(), self.vectorTo(p2).mag())
                if self.size < nearDistance:
                    return Point(0, 0)
            if wall.wallType == 'maze':
                normalUnitVector = normalUnitVector.__mul__(10)
                tangentUnitVector = tangentUnitVector.__mul__(2)
        # elif wall.wallType == 'maze':
            
        #     p1 = wall.parameters["p1"]
        #     p2 = wall.parameters["p2"]
        #     r = self.vectorTo(p1)
        #     wallLine = (p2 - p1).norm()
        #     normalUnitVector = Point(-wallLine.y, wallLine.x)  # rotate 90 degrees counterclockwise
        #     temp = dotProduct(r, normalUnitVector)
        #     if temp > 0:  # perpendicular
        #         normalUnitVector.x *= -1
        #         normalUnitVector.y *= -1
        #     elif temp == 0:
        #         normalUnitVector = -r.norm()

        #     if dotProduct(self.velocity, wallLine) >= 0:
        #         tangentUnitVector = wallLine
        #     else:
        #         tangentUnitVector = -wallLine
        #     distance = -dotProduct(r, normalUnitVector)

        #     # if wall cannot apply force on agent
        #     #  (in the case of that intersection between wall and normal vector is not on wall.)
        #     perpendicularPoint = self.pos + normalUnitVector * distance
        #     if dotProduct(p1 - perpendicularPoint, p2 - perpendicularPoint) > 0:
        #         nearDistance = min(self.vectorTo(p1).mag(), self.vectorTo(p2).mag())
        #         if self.size < nearDistance:
        #             return Point(0, 0)
        #     normalUnitVector = normalUnitVector.__mul__(10)
        #     tangentUnitVector = tangentUnitVector.__mul__(2)

        else:  # wallType : circle
            normalUnitVector = (self.pos - wall.parameters["center"]).norm()
            tangentLine = Point(-normalUnitVector.y, normalUnitVector.x)  # rotate 90 degrees counterclockwise
            if dotProduct(self.velocity, tangentLine) >= 0:
                tangentUnitVector = tangentLine
            else:
                tangentUnitVector = -tangentLine
            distance = (self.pos - wall.parameters["center"]).mag() - wall.parameters["radius"]

        overlap = self.size - distance

        return 3*self.calculateForce(overlap, self.velocity, tangentUnitVector, normalUnitVector)

    # the force between the agent and other agent
    def pairForce(self, other):
        displacement = self.pos - other.pos
        overlap = self.size + other.size - displacement.mag()
        normalUnitVector = (self.pos - other.pos).norm()
        tangentUnitVector = Point(-normalUnitVector.y, normalUnitVector.x)

        return self.calculateForce(overlap, self.velocity - other.velocity, tangentUnitVector, normalUnitVector)

    # calculating each force(Psychological force and contacting force) of wallForce() and pairForce()
    def calculateForce(self, overlap, velocityDifference, tangentUnitVector, normalUnitVector):
        # psychological force
        psyForce = self.psychologicalForce(overlap)
        # touching force(young and tangential force)
        if overlap > 0:  # if distance is shorter than size
            youngForce = self.youngForce(overlap)
            tangentForce = self.tangentialForce(overlap, velocityDifference, tangentUnitVector)
        else:
            youngForce = 0
            tangentForce = 0
        return ((psyForce + youngForce) * normalUnitVector) - (tangentForce * tangentUnitVector)

    # scalar function for the magnitude of force
    def psychologicalForce(self, overlap):
        return self.A * math.exp(overlap / self.B)

    @classmethod
    def youngForce(cls, overlap):  # overlap > 0
        return cls.k * overlap

    @classmethod
    def tangentialForce(cls, overlap, tangVeloDiff, tangentDirection):  # overlap > 0
        return cls.kappa * overlap * dotProduct(tangVeloDiff, tangentDirection)


def dotProduct(vec1, vec2):
    return vec1.x * vec2.x + vec1.y * vec2.y
