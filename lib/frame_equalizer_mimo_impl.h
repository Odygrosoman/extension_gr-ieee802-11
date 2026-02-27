#ifndef INCLUDED_IEEE802_11_FRAME_EQUALIZER_MIMO_IMPL_H
#define INCLUDED_IEEE802_11_FRAME_EQUALIZER_MIMO_IMPL_H

#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"

#include <ieee802_11/constellations.h>
#include <ieee802_11/frame_equalizer_mimo.h>
#include <ieee802_11/frame_equalizer_mimo.h>

#include <gnuradio/thread/thread.h>
#include <gnuradio/tags.h>

#include <memory>
#include <vector>

namespace gr {
    namespace ieee802_11 {

        class frame_equalizer_mimo_impl : virtual public frame_equalizer_mimo
        {
        public:
            frame_equalizer_mimo_impl(Equalizer algo, double freq, double bw, bool log, bool debug);
            ~frame_equalizer_mimo_impl();

            void set_algorithm(Equalizer algo);
            void set_bandwidth(double bw);
            void set_frequency(double freq);

            void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;

            int general_work(int noutput_items,
                gr_vector_int& ninput_items,
                gr_vector_const_void_star& input_items,
                gr_vector_void_star& output_items) override;

        private:
            bool parse_signal(uint8_t* signal);
            bool decode_signal_field(uint8_t* rx_bits);
            void deinterleave(uint8_t* rx_bits);

            // Two equalizers (one per RX antenna)
            equalizer::base* d_equalizer_ch1;
            equalizer::base* d_equalizer_ch2;

            gr::thread::mutex d_mutex;

            bool d_debug;
            bool d_log;
            int d_current_symbol;

            viterbi_decoder d_decoder;

            // Frequency / sampling offset tracking (common)
            double d_freq;                      // Hz
            double d_freq_offset_from_synclong; // Hz (from sync_long)
            double d_bw;                        // Hz
            double d_er;
            double d_epsilon0;

            // Previous pilots per antenna
            gr_complex d_prev_pilots_ch1[4];
            gr_complex d_prev_pilots_ch2[4];

            int d_frame_bytes;
            int d_frame_symbols;
            int d_frame_encoding;

            uint8_t d_deinterleaved[48];

            std::shared_ptr<gr::digital::constellation> d_frame_mod;
            constellation_bpsk::sptr d_bpsk;
            constellation_qpsk::sptr d_qpsk;
            constellation_16qam::sptr d_16qam;
            constellation_64qam::sptr d_64qam;

            static const int interleaver_pattern[48];
        };

    } // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_FRAME_EQUALIZER_MIMO_IMPL_H */