/*
 * Copyright 2012, 2013 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of SocRob Multicast.
 *
 * SocRob Multicast is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SocRob Multicast is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with SocRob Multicast.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SOCROB_MULTICAST_DATAOTHERS_H_
#define _SOCROB_MULTICAST_DATAOTHERS_H_

#include <vector>
#include <set>

#include <socrob_multicast/definitions.h>

#include "ShortData.h"



namespace socrob
{
  namespace multicast
  {
    class ShortDataOthers :
      public ShortData
    {
      public:
        ShortDataOthers (boost::shared_ptr<Protocol> const& protocol,
                         id_type starter_id,
                         std::vector<uint8_t> const& question,
                         std::set<id_type> const& required_sids) :
          ShortData (protocol, starter_id, question, required_sids) {}
          
        bool
        concearns_me() const {
          return (0 != answers().count (protocol_->SID()))
                 || (0 != required().count (protocol_->SID()));
        }
    };
  }
}

#endif
