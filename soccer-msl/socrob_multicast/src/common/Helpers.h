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

#ifndef _SOCROB_MULTICAST_HELPERS_H_
#define _SOCROB_MULTICAST_HELPERS_H_

#include <sstream>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <socrob_multicast/definitions.h>

#include <ros/ros.h>



namespace socrob
{
  namespace multicast
  {
    template<typename T>
    void
    publish_if (std::string const& if_param,
                std::string const& topic,
                ros::Publisher& publisher,
                T const& message)
    {
      bool debug_param;
      bool param_retrieved = ros::param::getCached (if_param, debug_param);
      if (param_retrieved && debug_param) {
        if (!publisher) {
          publisher = ros::NodeHandle().advertise<T> (topic, 10, false);
        }
        publisher.publish (message);
      }
      else {
        if (publisher) {
          publisher = ros::Publisher();
        }
      }
    }
    
    
    
    template <typename A>
    inline std::string toStr (const std::set<A>& theset)
    {
      std::ostringstream s;
      s << "[";
      bool isFirst = true;
      foreach (A t, theset) {
        if (!isFirst) {
          s << ",";
        }
        s << " " << t;
        isFirst = false;
      }
      s << " ]";
      return s.str();
    }
    
    template <>
    inline std::string toStr<uint8_t> (const std::set<uint8_t>& theset)
    {
      std::ostringstream s;
      s << "[";
      bool isFirst = true;
      foreach (uint8_t t, theset) {
        if (!isFirst) {
          s << ",";
        }
        s << " " << static_cast<int> (t);
        isFirst = false;
      }
      s << " ]";
      return s.str();
    }
    
    template <typename A>
    inline std::string toStr (const std::vector<A>& thevector)
    {
      std::ostringstream s;
      s << "[";
      bool isFirst = true;
      foreach (A t, thevector) {
        if (!isFirst) {
          s << ",";
        }
        s << " " << t;
        isFirst = false;
      }
      s << " ]";
      return s.str();
    }
    
    template <>
    inline std::string toStr<uint8_t> (const std::vector<uint8_t>& thevector)
    {
      std::ostringstream s;
      s << "[";
      bool isFirst = true;
      foreach (uint8_t t, thevector) {
        if (!isFirst) {
          s << ",";
        }
        s << " " << static_cast<int> (t);
        isFirst = false;
      }
      s << " ]";
      return s.str();
    }
  }
}

#endif
