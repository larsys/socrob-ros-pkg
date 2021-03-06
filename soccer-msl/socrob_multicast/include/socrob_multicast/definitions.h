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

#ifndef _SOCROB_MULTICAST_DEFINITIONS_H_
#define _SOCROB_MULTICAST_DEFINITIONS_H_

#include <cstddef>
#include <stdint.h>

#include <string>
#include <map>
#include <vector>

#include <boost/function.hpp>



// Uses the loopback interface. Use this only when testing offline!
//#define SOCROB_MULTICAST_USE_LOOPBACK_INTERFACE



// Sends long round packages at exact times, the worst case scenario.
//#define SOCROB_MULTICAST_LONG_SYNC_WORST



namespace socrob
{
  namespace multicast
  {
    /// Receive buffer size
    const std::size_t MAX_MESSAGE_SIZE = 65536;
    
    
    /// SID type (8 bits)
    typedef uint8_t id_type;
    
    /// QID type (32 bits)
    typedef uint32_t qid_type;
    
    
    /// Agent states
    typedef enum {NOT_RUNNING, INSERT, RUNNING, REMOVE} agent_state_type;
    
    
    /// Function type that receives a SID
    typedef boost::function<void (id_type) > id_function_type;
    
    /// Empty function of type id_function_type
    inline void empty_id_function (id_type) {}
    
    
    /// Function type that receives a buffer containing data or to be filled
    typedef boost::function<void (std::vector<uint8_t>&) > buffer_function_type;
    
    /// Empty function of type buffer_function_type
    inline void empty_buffer_function (std::vector<uint8_t> &) {}
    
    
    /// Function type that receives a SID and a buffer containing data or to be filled
    typedef boost::function<void (id_type, std::vector<uint8_t>&) > id_buffer_function_type;
    
    /// Empty function of type id_buffer_function_type
    inline void empty_id_buffer_function (id_type, std::vector<uint8_t> const&) {}
    
    
    /// Function type for short round handlers, first argument is the answer, second is the SSID, last is the question.
    typedef boost::function<void (std::vector<uint8_t>&, id_type, std::vector<uint8_t>&) > handler_function_type;
    
    /// Empty function of type handler_function_type
    inline void empty_handler_function (std::vector<uint8_t>&, id_type, std::vector<uint8_t> const&) {}
    
    
    /// Function type for short round callbacks, receiving a map of SIDs to their answers
    typedef boost::function<void (std::map<id_type, std::vector<uint8_t> >&) > short_callback_type;
    
    /// Empty function of type short_callback_type
    inline void empty_short_callback (std::map<id_type, std::vector<uint8_t> >&) {}
  }
}

#endif
