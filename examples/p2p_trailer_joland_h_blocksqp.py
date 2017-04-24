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
sys.path.insert(0, os.getcwd()+"/..")
from omgtools import *


vehicle = HolonomicOrient(shapes=Square(0.4))  # in deg
vehicle.set_initial_conditions([2., 5., 0.]) 
vehicle.set_terminal_conditions([8., 5., 0.])

trailer = TrailerJolandHolonomic(lead_veh=vehicle,  shapes=Square(0.4), l_hitch = 0.3, l_hitch1 = 0.4,
                  bounds={'tmax': 40., 'tmin': -40.})
trailer.set_initial_conditions([-5.])  

environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
rectangle = Rectangle(width=.2, height=3.8)

environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [6., 7.]}, shape=rectangle))




problem0 = Point2point(trailer, environment, freeT=False) 
problem0.father.add(vehicle)  
problem0.set_options({'solver': 'ipopt', 'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
problem0.init()
simulator = Simulator(problem0)
simulator.run_once(simulate=False)

# create vehicle
vehicle = HolonomicOrient(shapes=Square(0.4))  # in deg
vehicle.set_initial_conditions([2., 5., 0.]) 
vehicle.set_terminal_conditions([8., 5., 0.])

# create trailer
trailer = TrailerJolandHolonomic(lead_veh=vehicle,  shapes=Square(0.4), l_hitch = 0.3, l_hitch1 = 0.4,
                  bounds={'tmax': 40., 'tmin': -40.})
trailer.set_initial_conditions([-5.])  

# create environment
environment = Environment(room={'shape': Square(10.), 'position': [5.,5.]})
rectangle = Rectangle(width=.2, height=3.8)

environment.add_obstacle(Obstacle({'position': [3., 3.]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [6., 7.]}, shape=rectangle))


problem1 = Point2point(trailer, environment, freeT=False)
problem1.father.add(vehicle)
# problem1.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'verbose':True, 'max_iter': 1, 'max_it_qp': 50, 'qp_init': False, 'hess_lim_mem': 0, 'print_header': False}}})
# problem1.set_optcdions({'max_iter': 11, 'solver': 'blocksqp', 'solver_options': {'blocksqp': {'verbose':True, 'opttol': 1e-3, 'nlinfeastol': 1e-3, 'qp_init': False, 'hess_lim_mem': 0, 'print_header': False}}})
problem1.set_options({'solver': 'blocksqp', 'solver_options': {'blocksqp': {'verbose':True, 'qp_init': False, 'hess_lim_mem': 0, 'print_header': False}}})
problem1.init()
problem1.father._var_result = problem0.father._var_result
problem1.father._dual_var_result = problem0.father._dual_var_result
simulator = Simulator(problem1)



trailer.plot('input')
problem1.plot('scene')

simulator.run(init_reset=False)
