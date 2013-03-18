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

#ifndef _SOCROB_MULTICAST_LONGROUNDMANAGER_H_
#define _SOCROB_MULTICAST_LONGROUNDMANAGER_H_

#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include <socrob_multicast/definitions.h>
#include <socrob_multicast/compressor.h>

#include "../common/MulticastChannel.h"
#include "../common/Protocol.h"

#include "DurationEstimator.h"
#include "TimeControl.h"



namespace socrob
{
  namespace multicast
  {
    class LongRoundManager :
      private boost::noncopyable
    {
      public:
        LongRoundManager (boost::shared_ptr<Protocol> const& protocol,
                          boost::asio::io_service& io_service,
                          std::string const& ADDRESS_STRING,
                          unsigned short PORT,
                          buffer_function_type const& marshall,
                          id_buffer_function_type const& unmarshall,
                          unsigned tup_us,
                          boost::shared_ptr<Compressor> const& compressor);
                          
      private:
        void control (const boost::system::error_code& error);
        void prepare (const boost::system::error_code& error);
        void send (const boost::system::error_code& error);
        void receive_callback (boost::posix_time::ptime const& time, std::vector<uint8_t> & buffer);
        
        // Constants
        const buffer_function_type marshall_;
        const id_buffer_function_type unmarshall_;
        
        // Thread safe variables
        boost::shared_ptr<Compressor> compressor_;
        boost::shared_ptr<Protocol> protocol_;
        TimeControl time_control_;
        
        // Send only variables
        std::vector<uint8_t> send_buffer_;
        boost::asio::deadline_timer timer_;
        DurationEstimator prepare_estimator_;
        bool short_active_last_time_;
        
        // Must be last. Communications only start after everything is initialized.
        MulticastChannel channel_;
    };
  }
}

#endif
