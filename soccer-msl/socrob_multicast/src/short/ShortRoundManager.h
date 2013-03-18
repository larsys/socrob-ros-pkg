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

#ifndef _SOCROB_MULTICAST_SHORTROUNDMANAGER_H_
#define _SOCROB_MULTICAST_SHORTROUNDMANAGER_H_

#include <set>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include <socrob_multicast/definitions.h>
#include <socrob_multicast/compressor.h>

#include "../common/MulticastChannel.h"
#include "../common/Protocol.h"

#include "ShortMine.h"
#include "ShortOthers.h"



namespace socrob
{
  namespace multicast
  {
    class ShortRoundManager :
      private boost::noncopyable
    {
      public:
        ShortRoundManager (boost::shared_ptr<Protocol> const& protocol,
                           boost::asio::io_service& io_service,
                           std::string const& ADDRESS_STRING,
                           unsigned short PORT,
                           handler_function_type const& handler,
                           unsigned short_slot_us,
                           unsigned short_wait_us,
                           std::size_t max_mine_simultaneous,
                           boost::shared_ptr<Compressor> const& compressor);
                           
        bool
        startShortRound (std::vector<uint8_t> const& question,
                         std::set<id_type> const& required_sids,
                         short_callback_type const& callback);
                         
      private:
        void
        receive_callback (boost::posix_time::ptime const& time,
                          std::vector<uint8_t> & buffer);
                          
        void send (const boost::system::error_code& error);
        
        // Thread unsafe methods
        void activate();
        void activate (boost::posix_time::ptime const& new_time,
                       bool shorten_only = false);
        void deactivate();
        
        // Constants
        const boost::posix_time::time_duration short_slot_time_;
        const boost::posix_time::time_duration short_wait_time_;
        
        // Thread safe variables
        boost::shared_ptr<Compressor> compressor_;
        boost::shared_ptr<Protocol> protocol_;
        
        // Variables that must be synchronized
        boost::mutex mutex_;
        bool active_;
        ShortMine active_mine_;
        ShortOthers active_others_;
        boost::asio::deadline_timer timer_;
        
        // Debug, only used in one place
        ros::Publisher send_debug_publisher_;
        ros::Publisher receive_debug_publisher_;
        
        // Must be last. Communications only start after everything is initialized.
        MulticastChannel channel_;
    };
  }
}

#endif
