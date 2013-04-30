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

#ifndef _SOCROB_MULTICAST_MANAGER_H_
#define _SOCROB_MULTICAST_MANAGER_H_

#include <string>
#include <set>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <socrob_multicast/definitions.h>
#include <socrob_multicast/compressor.h>



/// SocRob project namespace
namespace socrob
{
  /// SocRob Multicast library namespace
  namespace multicast
  {
    /**
     * @brief Options to be used when constructing the Manager
     *
     * This class is a simple container of options, to be passed to the Manager
     * constructor to be used there.
     */
    class ManagerOptions
    {
      public:
        /**
         * @brief Constructor
         *
         * The ManagerOptions constructor takes three parameters that must
         * always be specified.
         * @param sid The static ID of the agent. Must be different for all agents participating.
         * @param num_sids The maximum number of agents. Must be the same for all agents.
         * @param multicast_address The address to be used to create the multicast sockets.
         */
        ManagerOptions (uint8_t sid,
                        uint8_t num_sids,
                        std::string const& multicast_address) :
          sid_ (sid),
          num_sids_ (num_sids),
          multicast_address_ (multicast_address),
          bytes_per_second_ (0),
          multicast_long_port_ (2000),
          multicast_short_port_ (2001),
          online_handler_ (empty_id_function),
          offline_handler_ (empty_id_function),
          long_marshall_ (empty_buffer_function),
          long_unmarshall_ (empty_id_buffer_function),
          short_handler_ (empty_handler_function),
          tup_us_ (100000),
          max_lost_ (10),
          short_slot_us_ (5000),
          short_wait_us_ (1000),
          short_max_mine_simultaneous_ (4),
          compressor_ (new Compressor()) {}
          
        /**
         * @brief Sets the hard limit for bandwidth
         *
         * Sets the hard limit for bandwidth. Special value 0 disables the
         * hard limit. By default, the bandwidth is not limited.
         * @param bytes_per_second Maximum bandwidth in bytes per second, 0 means unlimited.
         * @return Reference for method chaining.
         */
        ManagerOptions& bandwidth_bps (std::size_t bytes_per_second) {
          bytes_per_second_ = bytes_per_second;
          return *this;
        }
        
        /**
         * @brief Sets the ports to be used
         *
         * Sets the ports to be used for the multicast sockets. Two ports are
         * necessary, one for short rounds and another for long rounds. The
         * two ports must be different. The ports must be the same for the
         * whole team. By default, ports 2000 and 2001 are used.
         * @param long_port Port to be used by the long round manager.
         * @param short_port Port to be used by the short round manager.
         * @return Reference for method chaining.
         */
        ManagerOptions& ports (unsigned short long_port, unsigned short short_port) {
          multicast_long_port_ = long_port;
          multicast_short_port_ = short_port;
          return *this;
        }
        
        /**
         * @brief Sets the team update period
         *
         * Sets the team update period for the agent, which is the minimum
         * interval between transmissions of each agent (it may be enlarged
         * due to delays). This value must be the same for the whole team.
         * By default, the team update period is 0,1 seconds.
         * @param tup_ms Team update period in milliseconds.
         * @return Reference for method chaining.
         */
        ManagerOptions& tup_ms (unsigned tup_ms) {
          tup_us_ = tup_ms * 1000;
          return *this;
        }
        
        /**
         * @brief Sets the team update period
         *
         * Sets the team update period for the agent, which is the minimum
         * interval between transmissions of each agent (it may be enlarged
         * due to delays). This value must be the same for the whole team.
         * By default, the team update period is 0,1 seconds.
         * @param tup_us Team update period in microseconds.
         * @return Reference for method chaining.
         */
        ManagerOptions& tup_us (unsigned tup_us) {
          tup_us_ = tup_us;
          return *this;
        }
        
        /**
         * @brief Sets the maximum number of lost packets
         *
         * Sets the maximum number of consecutive rounds in which no packet
         * from a given agent is received. After this number of rounds, the
         * agent is removed from the team. By default, this is 10. Using the
         * default team update period of 0,1 seconds, an agent will be removed
         * from the team after 1 second of silence.
         * @param max_lost Maximum number of lost packets.
         * @return Reference for method chaining.
         */
        ManagerOptions& max_lost (unsigned max_lost) {
          max_lost_ = max_lost;
          return *this;
        }
        
        /**
         * @brief Sets the slot duration for the short rounds
         *
         * Sets the slot duration to be used in the short rounds, which is the
         * maximum interval between transmissions of any agent active in a
         * short round. This value must be the same for the whole team. By
         * default, the slot duration is 5 milliseconds.
         * @param short_slot_us Slot duration in microseconds.
         * @return Reference for method chaining.
         */
        ManagerOptions& short_slot_us (unsigned short_slot_us) {
          short_slot_us_ = short_slot_us;
          return *this;
        }
        
        /**
         * @brief Sets the waiting time for the short rounds
         *
         * Sets the waiting time to be used by the short rounds, which is the
         * time that an agent waits before transmitting, after receiving a
         * packet from the agent that was expected to transmit right before.
         * When this happens, the slot duration is ignored. This value must be
         * the same for the whole team. By default, the waiting time is 1
         * millisecond.
         * @param short_wait_us Waiting time in microseconds.
         * @return Reference for method chaining.
         */
        ManagerOptions& short_wait_us (unsigned short_wait_us) {
          short_wait_us_ = short_wait_us;
          return *this;
        }
        
        /**
         * @brief Sets the maximum number of simultaneous short rounds
         *
         * Sets the maximum number of short rounds started by this agent which
         * can be active simultaneously. If this agent keeps starting short
         * rounds before the previous ones are resolved, after this number
         * of rounds new attempts to start a short round will fail, ignoring
         * the attempt. By default, the maximum number of simultaneous short
         * rounds started by one agent is 4.
         * @param short_max_mine_simultaneous Maximum number of simultaneous
         * short rounds.
         * @return Reference for method chaining.
         */
        ManagerOptions& short_max_mine_simultaneous (std::size_t short_max_mine_simultaneous) {
          short_max_mine_simultaneous_ = short_max_mine_simultaneous;
          return *this;
        }
        
        /**
         * @brief Sets the compressor to be used
         *
         * Sets the compressor to be used by both the long and the sort
         * rounds. It must inherit from the Compressor class and be thread
         * safe. It will automatically be used as a filter to encode all
         * transmissions and decode all received packets. The compressor must
         * be the same for the whole team. By default, no compression is used.
         * @param compressor Shared pointer to the compressor.
         * @return Reference for method chaining.
         */
        ManagerOptions& compressor (boost::shared_ptr<Compressor> const& compressor) {
          compressor_ = compressor;
          return *this;
        }
        
        /**
         * @brief Sets the handler for agent activation event
         *
         * Sets the function to be called when the first transmission from an
         * agent is received. Please note that this function might be called
         * simultaneously with other functions from different threads.
         * @param online_handler Agent activation handler.
         * @return Reference for method chaining.
         */
        ManagerOptions& online_handler (id_function_type const& online_handler) {
          online_handler_ = online_handler;
          return *this;
        }
        
        /**
         * @brief Sets the handler for agent deactivation event
         *
         * Sets the function to be called when an agent is determined to be
         * not running. Please note that this function might be called
         * simultaneously with other functions from different threads.
         * @param offline_handler Agent deactivation handler.
         * @return Reference for method chaining.
         */
        ManagerOptions& offline_handler (id_function_type const& offline_handler) {
          offline_handler_ = offline_handler;
          return *this;
        }
        
        /**
         * @brief Sets the function that creates long round packets
         *
         * Sets the function responsible for creation of long round packets.
         * This function will be called at the appropriate moment to generate
         * a packet to be transmitted in the long rounds. Please note that
         * this function might be called simultaneously with other functions
         * from different threads.
         * @param long_marshall Long round packet creator function.
         * @return Reference for method chaining.
         */
        ManagerOptions& long_marshall (buffer_function_type const& long_marshall) {
          long_marshall_ = long_marshall;
          return *this;
        }
        
        /**
         * @brief Sets the function that interprets long round packets
         *
         * Sets the function responsible to interpret long round packets. This
         * function will be called whenever a packet from any agent arrives in
         * the long rounds. Please note that this function might be called
         * simultaneously with other functions from different threads. Please
         * note that this function might be called many times simultaneously.
         * @param long_unmarshall Long round packet interpreter function.
         * @return Reference for method chaining.
         */
        ManagerOptions& long_unmarshall (id_buffer_function_type const& long_unmarshall) {
          long_unmarshall_ = long_unmarshall;
          return *this;
        }
        
        /**
         * @brief Sets the function that interprets short round packets
         *
         * Sets the function responsible to interpret short rounds packets.
         * This function will be called whenever a packet containing a
         * question arrives in the short rounds. As soon as the answer is
         * transmitted, data about the question round is deleted. Therefore,
         * if another packet arrives with the same answer, this function will
         * be called again. This can be a problem, depending on the question
         * semantics. If you need to ignore duplicates, you must implement
         * your own method. One possibility is including an identifier in the
         * question and keep track of already answered questions, ignoring
         * duplicates. Be careful with what happens when one agent is
         * restarted with the system running. Using timestamps instead of
         * identifiers might be simpler. Please note that this function might
         * be called from inside startShortRound. Please note that this
         * function might be called simultaneously with other functions from
         * different threads.
         * @param short_handler Short round packet interpreter function.
         * @return Reference for method chaining.
         */
        ManagerOptions& short_handler (handler_function_type const& short_handler) {
          short_handler_ = short_handler;
          return *this;
        }
        
      private:
        friend class Manager;
        
        uint8_t sid_;
        uint8_t num_sids_;
        std::string const& multicast_address_;
        std::size_t bytes_per_second_;
        unsigned short multicast_long_port_;
        unsigned short multicast_short_port_;
        id_function_type online_handler_;
        id_function_type offline_handler_;
        buffer_function_type long_marshall_;
        id_buffer_function_type long_unmarshall_;
        handler_function_type short_handler_;
        unsigned tup_us_;
        unsigned max_lost_;
        unsigned short_slot_us_;
        unsigned short_wait_us_;
        std::size_t short_max_mine_simultaneous_;
        boost::shared_ptr<Compressor> compressor_;
    };
    
    
    
    /**
     * @brief Main class of SocRob Multicast
     *
     * This class is the Manager, responsible to coordinate everything.
     */
    class Manager :
      private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         *
         * The Manager constructor creates the SocRob Multicast Manager and
         * starts threads and communication. For this reason, please declare
         * the Manager as the last variable of your class or use a shared_ptr
         * to it, so that you initialize it only after everything is ready to
         * start.
         * @param options Options to be used by the Manager.
         */
        Manager (ManagerOptions const& options);
        
        /**
         * @brief Destructor
         *
         * The Manager destructor stops communication and threads.
         */
        ~Manager();
        
        /**
         * @brief Starts a short round for multiple agents
         *
         * Starts a short question round that will transmit the question to
         * all agents specified in the required_sids set. When all the answers
         * are received or agents are considered offline, the callback will be
         * called.
         * @param question Question to be transmitted.
         * @param required_sids Set of agents that need to answer.
         * @param callback Callback function to be called when all answers arrive.
         */
        bool
        startShortRound (std::vector<uint8_t> const& question,
                         std::set<id_type> const& required_sids,
                         short_callback_type const& callback = empty_short_callback);
                         
        /**
         * @brief Starts a short round for a single agent
         *
         * Starts a short question round that will transmit the question to
         * the specified agent. When the answer is received or the agent is
         * considered offline, the callback will be called.
         * @param question Question to be transmitted.
         * @param required_sid Agent that needs to answer.
         * @param callback Callback function to be called when all answers arrive.
         */
        bool
        startShortRound (std::vector<uint8_t> const& question,
                         id_type required_sid,
                         short_callback_type const& callback = empty_short_callback) {
          std::set<id_type> required_sids;
          required_sids.insert (required_sid);
          return startShortRound (question, required_sids, callback);
        }
        
      private:
        struct implementation;
        boost::shared_ptr<implementation> impl_;
    };
  }
}

#endif
