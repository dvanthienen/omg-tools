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

import sys, os
sys.path.insert(0, os.getcwd()+'/..')
from omgtools import *

# initial guess via ipopt
vehicle = Holonomic(options={'safety_distance': 0.1})
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
problem0 = Point2point(vehicle, environment, freeT=False)
problem0.set_options({'solver': 'ipopt', 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem0.init()
simulator = Simulator(problem0)
simulator.run_once(simulate=False)

# use solution as init guess for blocksqp
vehicle = Holonomic(options={'safety_distance': 0.1})
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
problem1 = Point2point(vehicle, environment, freeT=False)
problem1.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'warmstart': False, 'hess_lim_mem': 0, 'print_header': False}}})
problem1.init()
problem1.father._var_result = problem0.father._var_result
problem1.father._dual_var_result = problem0.father._dual_var_result
simulator = Simulator(problem1)
simulator.run_once(simulate=False, init_reset=False)

# use solution as init guess for blocksqp (same problem)
vehicle = Holonomic(options={'safety_distance': 0.1})
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
problem2 = Point2point(vehicle, environment, freeT=False)
problem2.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'warmstart': True, 'hess_lim_mem': 0, 'print_header': False}}})
problem2.init()
problem2.father._var_result = problem1.father._var_result
problem2.father._dual_var_result = problem1.father._dual_var_result
simulator = Simulator(problem2)
simulator.run_once(simulate=False, init_reset=False)

# use solution as init guess for blocksqp (same problem)
vehicle = Holonomic(options={'safety_distance': 0.1})
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
environment = Environment(room={'shape': Square(5.)})
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [1.7, -0.5]}, shape=rectangle))
problem3 = Point2point(vehicle, environment, freeT=False)
problem3.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'warmstart': True, 'hess_lim_mem': 0, 'print_header': False}}})
problem3.init()
problem3.father._var_result = problem2.father._var_result
problem3.father._dual_var_result = problem2.father._dual_var_result
simulator = Simulator(problem3)
problem3.plot('scene')
vehicle.plot('input')
simulator.run(init_reset=False)
