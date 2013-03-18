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

#ifndef _SOCROB_MULTICAST_DURATIONESTIMATOR_H_
#define _SOCROB_MULTICAST_DURATIONESTIMATOR_H_

#include <algorithm>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>



namespace socrob
{
  namespace multicast
  {
    class DurationEstimator
    {
      public:
        DurationEstimator (std::size_t buffer_size = 3,
                           int multiply_percent = 150) :
          estimate_ (boost::posix_time::microseconds (0)),
          buffer_ (buffer_size),
          MULTIPLY_PERCENT_ (multiply_percent) {}
          
        /// Add a new prepare duration to statistics
        void add (boost::posix_time::time_duration const& duration) {
          buffer_.push_back (duration);
        }
        
        /// Updates the current estimate. This can take some time.
        void update() {
          estimate_ = *std::max_element (buffer_.begin(), buffer_.end());
          estimate_ *= MULTIPLY_PERCENT_;
          estimate_ /= 100;
        }
        
        /// Get the current estimate
        boost::posix_time::time_duration get() const {
          return estimate_;
        }
        
      private:
        boost::posix_time::time_duration estimate_;
        boost::circular_buffer<boost::posix_time::time_duration> buffer_;
        const int MULTIPLY_PERCENT_; // Only integer arithmetics in time_duration
    };
  }
}

#endif
