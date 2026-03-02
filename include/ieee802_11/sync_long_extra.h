/* -*- c++ -*- */
/*
 * Copyright (C) 2013, 2016 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDED_IEEE802_11_SYNC_LONG_EXTRA_H
#define INCLUDED_IEEE802_11_SYNC_LONG_EXTRA_H

#include <gnuradio/block.h>
#include <ieee802_11/api.h>
#include <memory>

namespace gr {
    namespace ieee802_11 {

        /*!
         * \brief Like sync_long, but adds an extra tag with absolute STF start.
         * \ingroup ieee802_11
         */
        class IEEE802_11_API sync_long_extra : virtual public gr::block
        {
        public:
            typedef std::shared_ptr<sync_long_extra> sptr;

            static sptr make(unsigned int sync_length, bool log = false, bool debug = false);
        };

    } // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_SYNC_LONG_EXTRA_H */