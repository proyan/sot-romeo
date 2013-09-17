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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-romeo)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Romeo robot")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("romeo >= 1.1.1")

# Those packages are not directly linked, but are used in the 
#  python scripts
ADD_REQUIRED_DEPENDENCY("sot-core >= 2.7")
ADD_REQUIRED_DEPENDENCY("sot-tools")
ADD_REQUIRED_DEPENDENCY("sot-dynamic >= 2.6")

# Add subdirectories.
ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
