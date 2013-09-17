# Copyright 2013, Francois Keith, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-romeo.
# sot-romeo is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-romeo is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-romeo. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

CONFIG_FILES(dynamic_graph/sot/romeo/robot.py)

# Install Python files.
SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/romeo)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/romeo)

SET(PYTHON_MODULE dynamic_graph/sot/romeo)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py" )
PYTHON_INSTALL_BUILD("${PYTHON_MODULE}" "robot.py" "${PYTHON_SITELIB}")


