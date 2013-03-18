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

#ifndef _SOCROB_MULTICAST_SERIALIZATION_H_
#define _SOCROB_MULTICAST_SERIALIZATION_H_

#include <cstddef>
#include <stdint.h>

#include <vector>

#include <ros/console.h>
#include <ros/serialization.h>



namespace socrob
{
  namespace multicast
  {
    /**
     * Serializes a ROS message, appending the result to the given vector.
     * @param out vector where the result will be appended.
     * @param msg message to serialize.
     */
    template<typename T>
    void
    serialize_append (std::vector<uint8_t> & out,
                      T const& msg)
    {
      std::size_t out_offset = out.size();
      std::size_t msg_size = ros::serialization::serializationLength (msg);
      out.resize (out_offset + msg_size);
      ros::serialization::OStream stream (& (out[out_offset]), msg_size);
      ros::serialization::serialize (stream, msg);
    }
    
    /**
     * Serializes a ROS message, overwritting the whole given vector.
     * @param out vector where the result will be appended.
     * @param msg message to serialize.
     */
    template<typename T>
    void
    serialize_overwrite (std::vector<uint8_t> & out,
                         T const& msg)
    {
      std::size_t msg_size = ros::serialization::serializationLength (msg);
      out.resize (msg_size);
      ros::serialization::OStream stream (& (out[0]), msg_size);
      ros::serialization::serialize (stream, msg);
    }
    
    /**
     * Deserializes a ROS message.
     * @param msg resulting message.
     * @param in vector containing the message.
     * @param offset position in the vector where the serialized message starts, defaults to 0.
     * @return size of the serialized message in the vector.
     */
    template<typename T>
    std::size_t
    deserialize (T& msg,
                 std::vector<uint8_t> & in,
                 std::size_t offset = 0)
    {
      if (offset >= in.size()) {
        ROS_FATAL ("deserialize called with offset after the end of input");
        abort();
      }
      ros::serialization::IStream stream (& (in[offset]), in.size() - offset);
      ros::serialization::deserialize (stream, msg);
      return ros::serialization::serializationLength (msg);
    }
  }
}

#endif
