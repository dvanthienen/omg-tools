# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from omgtools import *

# create vehicle
vehicle = Holonomic()
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])

# create fleet
N = 4
fleet = Fleet([Holonomic() for l in range(N)])
configuration = RegularPolyhedron(0.2, N, np.pi/4.).vertices.T
init_positions = [-1.5, -1.5] + configuration
terminal_positions = [2., 2.] + configuration
fleet.set_configuration(configuration.tolist())
fleet.set_initial_conditions(init_positions.tolist())
fleet.set_terminal_conditions(terminal_positions.tolist())

# create environment
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)

environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))

problem1 = Point2point(vehicle, environment, freeT=False)
problem1.set_options({'verbose': 1})
problem1.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})

problem2 = Point2point(vehicle, environment, freeT=False)
problem2.set_options({'verbose': 1})
problem2.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem2.set_options({'codegen': {'build': 'jit', 'flags': '-O0'}}) # just-in-time compilation

problem3 = Point2point(vehicle, environment, freeT=False)
problem3.set_options({'verbose': 1})
problem3.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem3.set_options({'codegen': {'build': 'shared', 'flags': '-O0'}}) # compile to shared object

problem4 = Point2point(vehicle, environment, freeT=False)
problem4.set_options({'verbose': 1})
problem4.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem4.set_options({'codegen': {'build': 'existing'}}) # use existing shared object

vehicle.plot('input', knots=True)

print ('No compilation single-vehicle')
problem1.init()
simulator = Simulator(problem1)
simulator.run()

print('\n')
print('Just-in-time compilation single-vehicle')
problem2.init()
vehicle.overrule_state([-1.5, -1.5])
vehicle.overrule_input([0., 0.])
simulator.set_problem(problem2)
simulator.run()

print('\n')
print('Compile to shared object single-vehicle')
problem3.init()
vehicle.overrule_state([-1.5, -1.5])
vehicle.overrule_input([0., 0.])
simulator.set_problem(problem3)
simulator.run()

print('\n')
print('Use previous shared object single-vehicle')
problem4.init()
vehicle.overrule_state(np.array([-1.5, -1.5]))
vehicle.overrule_input(np.zeros(2))
simulator.set_problem(problem4)
simulator.run()
