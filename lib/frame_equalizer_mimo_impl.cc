/*
 * MIMO (2Rx) extension of SISO frame_equalizer_impl with MRC combining.
 * Keeps SISO logic: wifi_start tags, sampling offset compensation,
 * pilot tracking (beta), residual CFO update (d_er), signal field decode,
 * tags publishing, and "symbols" message port.
 */

#include "equalizer/base.h"
#include "equalizer/comb.h"
#include "equalizer/lms.h"
#include "equalizer/ls.h"
#include "equalizer/sta.h"

#include "frame_equalizer_mimo_impl.h"
#include <ieee802_11/frame_equalizer_mimo.h>

#include "utils.h"
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace gr {
    namespace ieee802_11 {

        frame_equalizer_mimo::sptr
            frame_equalizer_mimo::make(Equalizer algo, double freq, double bw, bool log, bool debug)
        {
            return gnuradio::get_initial_sptr(
                new frame_equalizer_mimo_impl(algo, freq, bw, log, debug));
        }

        frame_equalizer_mimo_impl::frame_equalizer_mimo_impl(Equalizer algo,
            double freq,
            double bw,
            bool log,
            bool debug)
            : gr::block("frame_equalizer_mimo",
                gr::io_signature::make(2, 2, 64 * sizeof(gr_complex)), // 2 Rx antennas
                gr::io_signature::make(1, 1, 48)),
            d_current_symbol(0),
            d_log(log),
            d_debug(debug),
            d_equalizer_ch1(nullptr),
            d_equalizer_ch2(nullptr),
            d_freq(freq),
            d_bw(bw),
            d_frame_bytes(0),
            d_frame_symbols(0),
            d_freq_offset_from_synclong(0.0),
            d_epsilon0(0.0),
            d_er(0.0)
        {
            message_port_register_out(pmt::mp("symbols"));

            d_bpsk = constellation_bpsk::make();
            d_qpsk = constellation_qpsk::make();
            d_16qam = constellation_16qam::make();
            d_64qam = constellation_64qam::make();
            d_frame_mod = d_bpsk;

            // init prev pilots
            for (int k = 0; k < 4; k++) {
                d_prev_pilots_ch1[k] = gr_complex(0, 0);
                d_prev_pilots_ch2[k] = gr_complex(0, 0);
            }

            set_tag_propagation_policy(block::TPP_DONT);
            set_algorithm(algo);
        }

        frame_equalizer_mimo_impl::~frame_equalizer_mimo_impl()
        {
            delete d_equalizer_ch1;
            delete d_equalizer_ch2;
        }

        void frame_equalizer_mimo_impl::set_algorithm(Equalizer algo)
        {
            gr::thread::scoped_lock lock(d_mutex);
            delete d_equalizer_ch1;
            delete d_equalizer_ch2;

            auto create_eq = [&](Equalizer a) -> equalizer::base* {
                switch (a) {
                case COMB: return new equalizer::comb();
                case LS:   return new equalizer::ls();
                case LMS:  return new equalizer::lms();
                case STA:  return new equalizer::sta();
                default:   throw std::runtime_error("Algorithm not implemented");
                }
                };

            d_equalizer_ch1 = create_eq(algo);
            d_equalizer_ch2 = create_eq(algo);
        }

        void frame_equalizer_mimo_impl::set_bandwidth(double bw)
        {
            gr::thread::scoped_lock lock(d_mutex);
            d_bw = bw;
        }

        void frame_equalizer_mimo_impl::set_frequency(double freq)
        {
            gr::thread::scoped_lock lock(d_mutex);
            d_freq = freq;
        }

        void frame_equalizer_mimo_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
        {
            // need aligned symbols from both antennas
            ninput_items_required[0] = noutput_items;
            ninput_items_required[1] = noutput_items;
        }

        /*
         * IMPORTANT:
         * Here symbols1/symbols2 are already equalized per-antenna:
         *   s1 = Y1 / H1 ,  s2 = Y2 / H2   (see ls.cc)
         * Therefore, the correct SIMO combining is the *post-equalization* MRC:
         *
         *   x_hat = (|H1|^2 * s1 + |H2|^2 * s2) / (|H1|^2 + |H2|^2)
         */
        static inline gr_complex combine_posteq_mrc(gr_complex s1, gr_complex h1,
            gr_complex s2, gr_complex h2)
        {
            const float w1 = std::norm(h1);
            const float w2 = std::norm(h2);
            const float norm = w1 + w2;
            if (norm <= 0.0f) return gr_complex(0, 0);
            return (w1 * s1 + w2 * s2) / norm;
        }

        // ✅ GNU Radio constellation does not provide demodulate(symbols[], bits[])
        // We must use decision_maker() per symbol.
        static inline void demodulate_48(const std::shared_ptr<gr::digital::constellation>& mod,
            const gr_complex* symbols,
            uint8_t* bits)
        {
            for (int k = 0; k < 48; k++) {
                bits[k] = (uint8_t)mod->decision_maker(&symbols[k]);
            }
        }

        int frame_equalizer_mimo_impl::general_work(int noutput_items,
            gr_vector_int& ninput_items,
            gr_vector_const_void_star& input_items,
            gr_vector_void_star& output_items)
        {
            gr::thread::scoped_lock lock(d_mutex);

            const gr_complex* in1 = (const gr_complex*)input_items[0];
            const gr_complex* in2 = (const gr_complex*)input_items[1];
            uint8_t* out = (uint8_t*)output_items[0];

            // process only where both streams have items
            const int n_in = std::min(ninput_items[0], ninput_items[1]);

            int i = 0; // input symbol index
            int o = 0; // output OFDM data-symbol index (only for symbols > 2)

            gr_complex symbols1[48];
            gr_complex symbols2[48];
            gr_complex symbols_mrc[48];

            gr_complex cur1[64];
            gr_complex cur2[64];

            // temp buffer for signal field bits (do NOT write to stream output, like SISO)
            uint8_t sig_bits[48];

            std::vector<tag_t> tags;

            while ((i < n_in) && (o < noutput_items)) {

                // wifi_start tag on input 0 (antenna 1)
                get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("wifi_start"));

                if (!tags.empty()) {
                    d_current_symbol = 0;
                    d_frame_symbols = 0;
                    d_frame_mod = d_bpsk;

                    // same as SISO:
                    d_freq_offset_from_synclong =
                        pmt::to_double(tags.front().value) * d_bw / (2 * M_PI);
                    d_epsilon0 =
                        pmt::to_double(tags.front().value) * d_bw / (2 * M_PI * d_freq);
                    d_er = 0.0;

                    // reset prev pilots on new frame
                    for (int k = 0; k < 4; k++) {
                        d_prev_pilots_ch1[k] = gr_complex(0, 0);
                        d_prev_pilots_ch2[k] = gr_complex(0, 0);
                    }

                    dout << "epsilon: " << d_epsilon0 << std::endl;
                }

                // if we're beyond the interesting region -> skip
                if (d_current_symbol > (d_frame_symbols + 2)) {
                    i++;
                    continue;
                }

                std::memcpy(cur1, in1 + i * 64, 64 * sizeof(gr_complex));
                std::memcpy(cur2, in2 + i * 64, 64 * sizeof(gr_complex));

                // --- sampling offset compensation (same correction for both Rx chains) ---
                for (int k = 0; k < 64; k++) {
                    const gr_complex rot = std::exp(gr_complex(
                        0,
                        2 * M_PI * d_current_symbol * 80 * (d_epsilon0 + d_er) * (k - 32) / 64));
                    cur1[k] *= rot;
                    cur2[k] *= rot;
                }

                // --- pilot-based common phase (beta) and residual CFO metric (er) ---
                const gr_complex p = equalizer::base::POLARITY[(d_current_symbol - 2) % 127];

                // beta metric per antenna (copy of SISO logic), then combine metrics
                gr_complex beta_metric1(0, 0), beta_metric2(0, 0);

                if (d_current_symbol < 2) {
                    // SISO uses: arg(x11 - x25 + x39 + x53)
                    beta_metric1 = (cur1[11] - cur1[25] + cur1[39] + cur1[53]);
                    beta_metric2 = (cur2[11] - cur2[25] + cur2[39] + cur2[53]);
                }
                else {
                    // SISO uses: arg((x11*p) + (x39*p) + (x25*p) + (x53*-p))
                    beta_metric1 = (cur1[11] * p) + (cur1[39] * p) + (cur1[25] * p) + (cur1[53] * -p);
                    beta_metric2 = (cur2[11] * p) + (cur2[39] * p) + (cur2[25] * p) + (cur2[53] * -p);
                }

                // combine beta metrics across antennas (phasor sum)
                const double beta = std::arg(beta_metric1 + beta_metric2);

                // er metric uses previous pilots (SISO style), sum across antennas
                gr_complex er_metric(0, 0);
                if (d_current_symbol >= 2) {
                    er_metric += (std::conj(d_prev_pilots_ch1[0]) * cur1[11] * p) +
                        (std::conj(d_prev_pilots_ch1[1]) * cur1[25] * p) +
                        (std::conj(d_prev_pilots_ch1[2]) * cur1[39] * p) +
                        (std::conj(d_prev_pilots_ch1[3]) * cur1[53] * -p);

                    er_metric += (std::conj(d_prev_pilots_ch2[0]) * cur2[11] * p) +
                        (std::conj(d_prev_pilots_ch2[1]) * cur2[25] * p) +
                        (std::conj(d_prev_pilots_ch2[2]) * cur2[39] * p) +
                        (std::conj(d_prev_pilots_ch2[3]) * cur2[53] * -p);
                }

                double er = 0.0;
                if (d_current_symbol >= 2) {
                    er = std::arg(er_metric);
                    er *= d_bw / (2 * M_PI * d_freq * 80);
                }

                // update prev pilots (per antenna), exactly like SISO but duplicated
                if (d_current_symbol < 2) {
                    d_prev_pilots_ch1[0] = cur1[11];
                    d_prev_pilots_ch1[1] = -cur1[25];
                    d_prev_pilots_ch1[2] = cur1[39];
                    d_prev_pilots_ch1[3] = cur1[53];

                    d_prev_pilots_ch2[0] = cur2[11];
                    d_prev_pilots_ch2[1] = -cur2[25];
                    d_prev_pilots_ch2[2] = cur2[39];
                    d_prev_pilots_ch2[3] = cur2[53];
                }
                else {
                    d_prev_pilots_ch1[0] = cur1[11] * p;
                    d_prev_pilots_ch1[1] = cur1[25] * p;
                    d_prev_pilots_ch1[2] = cur1[39] * p;
                    d_prev_pilots_ch1[3] = cur1[53] * -p;

                    d_prev_pilots_ch2[0] = cur2[11] * p;
                    d_prev_pilots_ch2[1] = cur2[25] * p;
                    d_prev_pilots_ch2[2] = cur2[39] * p;
                    d_prev_pilots_ch2[3] = cur2[53] * -p;
                }

                // compensate residual common phase offset beta for both
                const gr_complex beta_rot = std::exp(gr_complex(0, -beta));
                for (int k = 0; k < 64; k++) {
                    cur1[k] *= beta_rot;
                    cur2[k] *= beta_rot;
                }

                // update d_er (same as SISO)
                if (d_current_symbol >= 2) {
                    const double alpha = 0.1;
                    d_er = (1 - alpha) * d_er + alpha * er;
                }

                // --- equalize per antenna ---
                uint8_t dummy_bits[48];
                d_equalizer_ch1->equalize(cur1, d_current_symbol, symbols1, dummy_bits, d_frame_mod);
                d_equalizer_ch2->equalize(cur2, d_current_symbol, symbols2, dummy_bits, d_frame_mod);

                // --- combine using channel estimates H1/H2 with correct mapping (48 data bins, pilots skipped) ---
                const gr_complex* H1 = d_equalizer_ch1->get_H();
                const gr_complex* H2 = d_equalizer_ch2->get_H();

                // Use EXACT same carrier-skipping as ls.cc for data mapping
                int c = 0;
                for (int sc = 0; sc < 64; sc++) {
                    if ((sc == 11) || (sc == 25) || (sc == 32) || (sc == 39) || (sc == 53) ||
                        (sc < 6) || (sc > 58)) {
                        continue;
                    }
                    symbols_mrc[c] = combine_posteq_mrc(symbols1[c], H1[sc], symbols2[c], H2[sc]);
                    c++;
                }

                if (c != 48) {
                    for (int k = 0; k < 48; k++) {
                        symbols_mrc[k] = 0.5f * (symbols1[k] + symbols2[k]);
                    }
                }

                // --- SIGNAL field (symbol index 2) ---
                if (d_current_symbol == 2) {
                    // ✅ fixed
                    demodulate_48(d_frame_mod, symbols_mrc, sig_bits);

                    if (decode_signal_field(sig_bits)) {
                        pmt::pmt_t dict = pmt::make_dict();

                        dict = pmt::dict_add(dict, pmt::mp("frame bytes"), pmt::from_uint64(d_frame_bytes));
                        dict = pmt::dict_add(dict, pmt::mp("encoding"), pmt::from_uint64(d_frame_encoding));

                        const double snr1 = d_equalizer_ch1->get_snr();
                        const double snr2 = d_equalizer_ch2->get_snr();
                        const double snr_comb = std::max(snr1, snr2);

                        dict = pmt::dict_add(dict, pmt::mp("snr"), pmt::from_double(snr_comb));
                        dict = pmt::dict_add(dict, pmt::mp("snr ch1"), pmt::from_double(snr1));
                        dict = pmt::dict_add(dict, pmt::mp("snr ch2"), pmt::from_double(snr2));

                        dict = pmt::dict_add(dict, pmt::mp("nominal frequency"), pmt::from_double(d_freq));
                        dict = pmt::dict_add(dict, pmt::mp("frequency offset"),
                            pmt::from_double(d_freq_offset_from_synclong));
                        dict = pmt::dict_add(dict, pmt::mp("beta"), pmt::from_double(beta));

                        // Optional debug CSI (52 bins)
                        std::vector<gr_complex> csi1 = d_equalizer_ch1->get_csi();
                        std::vector<gr_complex> csi2 = d_equalizer_ch2->get_csi();

                        dict = pmt::dict_add(dict, pmt::mp("csi ch1"), pmt::init_c32vector(csi1.size(), csi1));
                        dict = pmt::dict_add(dict, pmt::mp("csi ch2"), pmt::init_c32vector(csi2.size(), csi2));

                        if (csi1.size() == csi2.size() && !csi1.empty()) {
                            std::vector<gr_complex> csic(csi1.size());
                            for (size_t k = 0; k < csic.size(); k++) csic[k] = csi1[k] + csi2[k];
                            dict = pmt::dict_add(dict, pmt::mp("csi combined"),
                                pmt::init_c32vector(csic.size(), csic));
                        }

                        // add tags to output stream (same as SISO)
                        pmt::pmt_t pairs = pmt::dict_items(dict);
                        for (int k = 0; k < pmt::length(pairs); k++) {
                            pmt::pmt_t pair = pmt::nth(k, pairs);
                            add_item_tag(0,
                                nitems_written(0) + o,
                                pmt::car(pair),
                                pmt::cdr(pair),
                                alias_pmt());
                        }
                    }
                }

                // --- DATA symbols (symbol index > 2) ---
                if (d_current_symbol > 2) {
                    // ✅ fixed
                    demodulate_48(d_frame_mod, symbols_mrc, out + o * 48);

                    message_port_pub(pmt::mp("symbols"),
                        pmt::cons(pmt::make_dict(),
                            pmt::init_c32vector(48, symbols_mrc)));
                    o++;
                }

                i++;
                d_current_symbol++;
            }

            consume(0, i);
            consume(1, i);
            return o;
        }

        // ---- Below: unchanged from SISO ----

        bool frame_equalizer_mimo_impl::decode_signal_field(uint8_t* rx_bits)
        {
            static ofdm_param ofdm(BPSK_1_2);
            static frame_param frame(ofdm, 0);

            deinterleave(rx_bits);
            uint8_t* decoded_bits = d_decoder.decode(&ofdm, &frame, d_deinterleaved);

            return parse_signal(decoded_bits);
        }

        void frame_equalizer_mimo_impl::deinterleave(uint8_t* rx_bits)
        {
            for (int i = 0; i < 48; i++) {
                d_deinterleaved[i] = rx_bits[interleaver_pattern[i]];
            }
        }

        bool frame_equalizer_mimo_impl::parse_signal(uint8_t* decoded_bits)
        {
            int r = 0;
            d_frame_bytes = 0;
            bool parity = false;

            for (int i = 0; i < 17; i++) {
                parity ^= decoded_bits[i];

                if ((i < 4) && decoded_bits[i]) {
                    r = r | (1 << i);
                }

                if (decoded_bits[i] && (i > 4) && (i < 17)) {
                    d_frame_bytes = d_frame_bytes | (1 << (i - 5));
                }
            }

            if (parity != decoded_bits[17]) {
                dout << "SIGNAL: wrong parity" << std::endl;
                return false;
            }

            switch (r) {
            case 11:
                d_frame_encoding = 0;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)24);
                d_frame_mod = d_bpsk;
                dout << "Encoding: 3 Mbit/s   ";
                break;
            case 15:
                d_frame_encoding = 1;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)36);
                d_frame_mod = d_bpsk;
                dout << "Encoding: 4.5 Mbit/s   ";
                break;
            case 10:
                d_frame_encoding = 2;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)48);
                d_frame_mod = d_qpsk;
                dout << "Encoding: 6 Mbit/s   ";
                break;
            case 14:
                d_frame_encoding = 3;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)72);
                d_frame_mod = d_qpsk;
                dout << "Encoding: 9 Mbit/s   ";
                break;
            case 9:
                d_frame_encoding = 4;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)96);
                d_frame_mod = d_16qam;
                dout << "Encoding: 12 Mbit/s   ";
                break;
            case 13:
                d_frame_encoding = 5;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)144);
                d_frame_mod = d_16qam;
                dout << "Encoding: 18 Mbit/s   ";
                break;
            case 8:
                d_frame_encoding = 6;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)192);
                d_frame_mod = d_64qam;
                dout << "Encoding: 24 Mbit/s   ";
                break;
            case 12:
                d_frame_encoding = 7;
                d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)216);
                d_frame_mod = d_64qam;
                dout << "Encoding: 27 Mbit/s   ";
                break;
            default:
                dout << "unknown encoding" << std::endl;
                return false;
            }

            mylog("encoding: {} - length: {} - symbols: {}", d_frame_encoding, d_frame_bytes, d_frame_symbols);
            return true;
        }

        const int frame_equalizer_mimo_impl::interleaver_pattern[48] = {
            0, 3, 6, 9,  12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45,
            1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
            2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47
        };

    } // namespace ieee802_11
} // namespace gr