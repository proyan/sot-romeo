// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of sot-romeo.
// sot-romeo is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-romeo is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-romeo. If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_ROMEO_DYNAMIC_HH
# define SOT_ROMEO_DYNAMIC_HH
# include <sot-dynamic/dynamic.h>

# if defined (WIN32)
#   if defined (dynamic_romeo_EXPORTS)
#     define DYNAMICROMEO_EXPORT __declspec(dllexport)
#   else
#     define DYNAMICROMEO_EXPORT __declspec(dllimport)
#   endif
# else
#   define DYNAMICROMEO_EXPORT
# endif

namespace dynamicgraph
{
  namespace sot
  {
    namespace romeo
    {
      class DYNAMICROMEO_EXPORT DynamicRomeo : public Dynamic
      {
	DYNAMIC_GRAPH_ENTITY_DECL ();
      public:
	explicit DynamicRomeo (const std::string& name);
	virtual ~DynamicRomeo ();
      };
    } // end of namespace romeo.
  } // end of namespace sot.
} // end of namespace dynamicgraph.


#endif //! SOT_ROMEO_DYNAMIC_HH
