// Compile this file FIRST (before loop_filter.sv and pll_tb.sv).
//
// IMPORTANT — file name vs package name:
//   File:    00_xreal_pkg.sv   (leading 00 = compile order only)
//   Package: xreal_pkg         (use in import statements)
//
//   Correct:   import xreal_pkg::*;
//   WRONG:     import 00_xreal_pkg::*;   // invalid identifier
//
// Cadence:  irun -f compile.f

package xreal_pkg;

    parameter int MAX_XREAL_TERMS = 32;

    function automatic real real_abs(input real v);
        real_abs = (v < 0.0) ? -v : v;
    endfunction

    typedef struct {
        real b;
        real a;
        int  m;
        real t0;
    } xreal_term_t;

    typedef struct {
        xreal_term_t terms [0:MAX_XREAL_TERMS-1];
        int          count;
    } xreal_seq_t;

    parameter int MAX_SPECTRAL_BINS = 8;

    // Section III-B: s(t) = sum_k (I_k + jQ_k) * exp(j*omega_k*t)
    typedef struct {
        real omega;   // rad/s
        real I_val;
        real Q_val;
    } spectral_bin_t;

    typedef struct {
        spectral_bin_t entry [0:MAX_SPECTRAL_BINS-1];
        int            count;
    } spectral_seq_t;

    function automatic void spectral_clear(output spectral_seq_t spec);
        begin
            spec.count = 0;
        end
    endfunction

    function automatic void spectral_add_bin(
        ref   spectral_seq_t spec,
        input real           omega,
        input real           I,
        input real           Q
    );
        begin
            if (spec.count < MAX_SPECTRAL_BINS) begin
                spec.entry[spec.count].omega = omega;
                spec.entry[spec.count].I_val = I;
                spec.entry[spec.count].Q_val = Q;
                spec.count++;
            end else begin
                $error("spectral_add_bin: MAX_SPECTRAL_BINS exceeded");
            end
        end
    endfunction

    // Scale all XREAL amplitudes by gain (Ki * Vin in eq. 8).
    function automatic xreal_seq_t xreal_scale(input real gain, input xreal_seq_t seq_in);
        xreal_seq_t result;
        begin
            xreal_clear(result);
            for (int i = 0; i < seq_in.count; i++) begin
                xreal_add_term(
                    result,
                    gain * seq_in.terms[i].b,
                    seq_in.terms[i].a,
                    seq_in.terms[i].m,
                    seq_in.terms[i].t0
                );
            end
            xreal_scale = result;
        end
    endfunction

    // Eq. (8): fout = Ki*Vin + f0  as XREAL sequences.
    function automatic xreal_seq_t vco_v_to_f(
        input xreal_seq_t vin,
        input real        Ki,
        input real        f0,
        input real        t0
    );
        xreal_seq_t result;
        begin
            result = xreal_scale(Ki, vin);
            xreal_add_term(result, f0, 0.0, 1, t0);
            xreal_compress(result);
            xreal_sort(result);
            vco_v_to_f = result;
        end
    endfunction

    // Delta-f = fout - f0 (remove DC frequency term at pole a=0, m=1).
    function automatic xreal_seq_t xreal_delta_f(input xreal_seq_t fout, input real f0);
        xreal_seq_t result;
        begin
            xreal_clear(result);
            for (int i = 0; i < fout.count; i++) begin
                if (fout.terms[i].a == 0.0 && fout.terms[i].m == 1) begin
                    if (real_abs(fout.terms[i].b - f0) > 1.0e-15) begin
                        xreal_add_term(
                            result,
                            fout.terms[i].b - f0,
                            0.0,
                            1,
                            fout.terms[i].t0
                        );
                    end
                end else begin
                    xreal_add_term(
                        result,
                        fout.terms[i].b,
                        fout.terms[i].a,
                        fout.terms[i].m,
                        fout.terms[i].t0
                    );
                end
            end
            xreal_compress(result);
            xreal_delta_f = result;
        end
    endfunction

    // Analytical integral of delta-f XREAL (m=1 terms) from 0 to t.
    function automatic real integrate_xreal_m1_at_t(input xreal_seq_t seq, input real t);
        real sum, dt, a, b;
        begin
            sum = 0.0;
            for (int i = 0; i < seq.count; i++) begin
                if (seq.terms[i].m != 1) begin
                    $error("integrate_xreal_m1_at_t: only m=1 supported");
                end else if (t >= seq.terms[i].t0) begin
                    dt = t - seq.terms[i].t0;
                    b  = seq.terms[i].b;
                    a  = seq.terms[i].a;
                    if (real_abs(a) < 1.0e-30)
                        sum += b * dt;
                    else
                        sum += (b / a) * ($exp(a * dt) - 1.0);
                end
            end
            integrate_xreal_m1_at_t = sum;
        end
    endfunction

    // Eq. (9) RF waveform — exact bounded form (always |s| <= v_peak):
    //   phi(t) = 2*pi * integral_0^t delta_f(tau) d tau
    //   s(t)   = v_peak * cos(omega_0*t + phi(t))
    //
    // The linearized cos(w0*t) - sin(w0*t)*phi is only valid for |phi|<<1;
    // it diverges to kV when phi grows from integrating a DC delta-f term.
    function automatic real eval_vco_rf_at_t(
        input real        f0,
        input xreal_seq_t delta_f,
        input real        t,
        input real        v_peak
    );
        real w0, phi, wt, cos_wt;
        begin
            w0     = 2.0 * 3.14159265358979323846 * f0;
            phi    = 2.0 * 3.14159265358979323846 * integrate_xreal_m1_at_t(delta_f, t);
            wt     = w0 * t + phi;
            cos_wt = $cos(wt);
            eval_vco_rf_at_t = v_peak * cos_wt;
        end
    endfunction

    // Legacy name — redirects to bounded evaluator with v_peak = 1.0
    function automatic real eval_vco_rf_approx(
        input real        f0,
        input xreal_seq_t delta_f,
        input real        t
    );
        eval_vco_rf_approx = eval_vco_rf_at_t(f0, delta_f, t, 1.0);
    endfunction

    // Evaluate spectral sum, scaled and clamped to +/- v_peak.
    function automatic real eval_spectral_scaled(
        input spectral_seq_t spec,
        input real           t,
        input real           v_peak
    );
        real sum, w, wt, cos_wt, sin_wt, raw;
        begin
            sum = 0.0;
            for (int k = 0; k < spec.count; k++) begin
                w  = spec.entry[k].omega;
                if (real_abs(w) < 1.0e-30) begin
                    sum += spec.entry[k].I_val;
                end else begin
                    wt     = w * t;
                    cos_wt = $cos(wt);
                    sin_wt = $sin(wt);
                    sum += spec.entry[k].I_val * cos_wt -
                           spec.entry[k].Q_val * sin_wt;
                end
            end
            raw = v_peak * sum;
            if (raw > v_peak)
                eval_spectral_scaled = v_peak;
            else if (raw < -v_peak)
                eval_spectral_scaled = -v_peak;
            else
                eval_spectral_scaled = raw;
        end
    endfunction

    function automatic real eval_spectral_at_t(
        input spectral_seq_t spec,
        input real           t
    );
        eval_spectral_at_t = eval_spectral_scaled(spec, t, 1.0);
    endfunction

    // Build spectral bins from delta-f per eq. (9)-(10) small-deviation model.
    // Carrier at +/- w0 plus first sideband estimate from instantaneous delta-f.
    function automatic spectral_seq_t vco_delta_f_to_spectral(
        input xreal_seq_t delta_f,
        input real        f0,
        input real        t_eval
    );
        spectral_seq_t spec;
        real w0, df_inst, phase_mod, sideband;
        begin
            spectral_clear(spec);
            w0      = 2.0 * 3.14159265358979323846 * f0;
            df_inst = eval_xreal_at_t(delta_f, t_eval);
            phase_mod = 2.0 * 3.14159265358979323846 *
                        integrate_xreal_m1_at_t(delta_f, t_eval);

            // Carriers: delta(w +/- w0)/2 in eq. (9)
            spectral_add_bin(spec,  w0, 0.5, 0.0);
            spectral_add_bin(spec, -w0, 0.5, 0.0);

            // Sideband estimate: +/- 1/(2j*w0) * DeltaF folded around carrier
            if (real_abs(w0) > 1.0e-30) begin
                sideband = -0.5 * df_inst / w0;
                spectral_add_bin(spec,  w0, 0.0, sideband);
                spectral_add_bin(spec, -w0, 0.0, -sideband);
            end

            // Store phase-modulation metadata as low-frequency bin (for debug)
            spectral_add_bin(spec, 0.0, 0.0, phase_mod);

            vco_delta_f_to_spectral = spec;
        end
    endfunction

    function automatic void spectral_print(input string label, input spectral_seq_t spec);
        begin
            $display("%s (%0d bins):", label, spec.count);
            for (int i = 0; i < spec.count; i++) begin
                $display("  [%0d] omega=%0.6e  I=%0.6e  Q=%0.6e",
                         i,
                         spec.entry[i].omega,
                         spec.entry[i].I_val,
                         spec.entry[i].Q_val);
            end
        end
    endfunction

    function automatic void xreal_clear(output xreal_seq_t seq);
        begin
            seq.count = 0;
        end
    endfunction

    function automatic void xreal_add_term(
        ref   xreal_seq_t seq,
        input real        b,
        input real        a,
        input int         m,
        input real        t0
    );
        begin
            if (seq.count < MAX_XREAL_TERMS) begin
                seq.terms[seq.count].b  = b;
                seq.terms[seq.count].a  = a;
                seq.terms[seq.count].m  = m;
                seq.terms[seq.count].t0 = t0;
                seq.count++;
            end else begin
                $error("xreal_add_term: MAX_XREAL_TERMS exceeded");
            end
        end
    endfunction

    function automatic void xreal_compress(ref xreal_seq_t seq);
        int i, j;
        begin
            i = 0;
            while (i < seq.count) begin
                j = i + 1;
                while (j < seq.count) begin
                    if (seq.terms[j].a == seq.terms[i].a &&
                        seq.terms[j].m == seq.terms[i].m &&
                        seq.terms[j].t0 == seq.terms[i].t0) begin
                        seq.terms[i].b += seq.terms[j].b;
                        seq.terms[j] = seq.terms[seq.count - 1];
                        seq.count--;
                    end else begin
                        j++;
                    end
                end
                i++;
            end

            i = 0;
            while (i < seq.count) begin
                if (real_abs(seq.terms[i].b) < 1.0e-15) begin
                    seq.terms[i] = seq.terms[seq.count - 1];
                    seq.count--;
                end else begin
                    i++;
                end
            end
        end
    endfunction

    // Fold all m=1 terms to a common origin t_ref, algebraically summing terms
    // that share the same pole a (same denominator in the Laplace domain).
    // For t >= t_ref:  b*exp(a*(t-t0))  ==  b*exp(a*(t_ref-t0))*exp(a*(t-t_ref))
    // Same-a contributions at different t0 merge into one coefficient at t_ref.
    function automatic void xreal_rebase_m1(ref xreal_seq_t seq, input real t_ref);
        xreal_seq_t folded;
        xreal_seq_t m_gt1;
        int         i, j;
        real        dt, b_fold;
        bit         found;
        begin
            xreal_clear(folded);
            xreal_clear(m_gt1);

            for (i = 0; i < seq.count; i++) begin
                if (seq.terms[i].m != 1) begin
                    xreal_add_term(
                        m_gt1,
                        seq.terms[i].b,
                        seq.terms[i].a,
                        seq.terms[i].m,
                        seq.terms[i].t0
                    );
                end else if (seq.terms[i].t0 <= t_ref) begin
                    dt = t_ref - seq.terms[i].t0;
                    if (real_abs(seq.terms[i].a) < 1.0e-30)
                        b_fold = seq.terms[i].b;
                    else
                        b_fold = seq.terms[i].b * $exp(seq.terms[i].a * dt);

                    found = 1'b0;
                    for (j = 0; j < folded.count; j++) begin
                        if (folded.terms[j].a == seq.terms[i].a &&
                            folded.terms[j].m == 1 &&
                            folded.terms[j].t0 == t_ref) begin
                            folded.terms[j].b += b_fold;
                            found = 1'b1;
                            break;
                        end
                    end
                    if (!found)
                        xreal_add_term(folded, b_fold, seq.terms[i].a, 1, t_ref);
                end else begin
                    xreal_add_term(
                        folded,
                        seq.terms[i].b,
                        seq.terms[i].a,
                        1,
                        seq.terms[i].t0
                    );
                end
            end

            xreal_clear(seq);
            for (i = 0; i < folded.count; i = i + 1)
                xreal_add_term(
                    seq,
                    folded.terms[i].b,
                    folded.terms[i].a,
                    folded.terms[i].m,
                    folded.terms[i].t0
                );
            for (i = 0; i < m_gt1.count; i = i + 1)
                xreal_add_term(
                    seq,
                    m_gt1.terms[i].b,
                    m_gt1.terms[i].a,
                    m_gt1.terms[i].m,
                    m_gt1.terms[i].t0
                );
            xreal_compress(seq);
        end
    endfunction

    function automatic void xreal_sort(ref xreal_seq_t seq);
        xreal_term_t tmp;
        bit swapped;
        begin
            do begin
                swapped = 1'b0;
                for (int i = 0; i < seq.count - 1; i++) begin
                    if ((seq.terms[i].a > seq.terms[i + 1].a) ||
                        (seq.terms[i].a == seq.terms[i + 1].a &&
                         seq.terms[i].m > seq.terms[i + 1].m) ||
                        (seq.terms[i].a == seq.terms[i + 1].a &&
                         seq.terms[i].m == seq.terms[i + 1].m &&
                         seq.terms[i].t0 > seq.terms[i + 1].t0)) begin
                        tmp = seq.terms[i];
                        seq.terms[i] = seq.terms[i + 1];
                        seq.terms[i + 1] = tmp;
                        swapped = 1'b1;
                    end
                end
            end while (swapped);
        end
    endfunction

    function automatic void xreal_add_exponential(
        ref   xreal_seq_t seq,
        input real        amplitude,
        input real        pole,
        input real        t0
    );
        begin
            xreal_add_term(seq, amplitude, pole, 1, t0);
        end
    endfunction

    function automatic void xreal_add_unit_step(
        ref   xreal_seq_t seq,
        input real        amplitude,
        input real        t0
    );
        begin
            xreal_add_term(seq, amplitude, 0.0, 1, t0);
        end
    endfunction

    function automatic void s_domain_multiply_simple_poles(
        input  real        b1, a1, t0_1,
        input  real        b2, a2, t0_out,
        output xreal_seq_t result
    );
        real denom;
        begin
            xreal_clear(result);
            denom = a1 - a2;
            if (denom == 0.0) begin
                $error("s_domain_multiply_simple_poles: coincident poles a1=a2=%0f", a1);
            end else begin
                xreal_add_term(result, (b1 * b2) / denom,       a1, 1, t0_out);
                xreal_add_term(result, (b1 * b2) / (-denom),      a2, 1, t0_out);
            end
        end
    endfunction

    function automatic void partial_fraction_repeated_pole(
        input  real        K,
        input  real        a,
        input  real        p,
        input  int         n,
        input  real        t0_out,
        output xreal_seq_t result
    );
        real ap;
        begin
            xreal_clear(result);
            ap = a - p;

            if (n == 1) begin
                if (ap == 0.0) begin
                    $error("partial_fraction_repeated_pole: input pole equals filter pole");
                end else begin
                    xreal_add_term(result,  K / ap,           a, 1, t0_out);
                    xreal_add_term(result, -K / ap,           p, 1, t0_out);
                end
            end else if (n == 2) begin
                if (ap == 0.0) begin
                    xreal_add_term(result, K, p, 3, t0_out);
                end else begin
                    xreal_add_term(result,  K / (ap * ap),    a, 1, t0_out);
                    xreal_add_term(result, -K / (ap * ap),    p, 1, t0_out);
                    xreal_add_term(result,  K / ap,           p, 2, t0_out);
                end
            end else if (n == 3) begin
                if (ap == 0.0) begin
                    xreal_add_term(result, K, p, 4, t0_out);
                end else begin
                    xreal_add_term(result,  K / (ap * ap * ap), a, 1, t0_out);
                    xreal_add_term(result,  K / ap,               p, 3, t0_out);
                    xreal_add_term(result, -K / (ap * ap),          p, 2, t0_out);
                    xreal_add_term(result,  K / (ap * ap * ap),   p, 1, t0_out);
                end
            end else begin
                $error("partial_fraction_repeated_pole: unsupported filter order n=%0d", n);
            end
        end
    endfunction

    function automatic xreal_seq_t convolve_with_filter(
        input xreal_seq_t seq_in,
        input real        c,
        input real        p,
        input int         n
    );
        xreal_seq_t result;
        xreal_seq_t partial;
        int i, j;
        begin
            xreal_clear(result);

            for (i = 0; i < seq_in.count; i++) begin
                if (seq_in.terms[i].m != 1) begin
                    $error("convolve_with_filter: only m=1 input terms supported in this demo");
                end else begin
                    partial_fraction_repeated_pole(
                        c * seq_in.terms[i].b,
                        seq_in.terms[i].a,
                        p,
                        n,
                        seq_in.terms[i].t0,
                        partial
                    );
                    for (j = 0; j < partial.count; j++) begin
                        xreal_add_term(
                            result,
                            partial.terms[j].b,
                            partial.terms[j].a,
                            partial.terms[j].m,
                            partial.terms[j].t0
                        );
                    end
                end
            end

            if (result.count > 0) begin
                real t_max;
                t_max = result.terms[0].t0;
                for (i = 1; i < result.count; i = i + 1)
                    if (result.terms[i].t0 > t_max)
                        t_max = result.terms[i].t0;
                xreal_rebase_m1(result, t_max);
            end

            xreal_compress(result);
            xreal_sort(result);
            convolve_with_filter = result;
        end
    endfunction

    function automatic real eval_xreal_at_t(input xreal_seq_t seq, input real t);
        real sum, dt, term_val;
        int  k, pow_i;
        begin
            sum = 0.0;
            for (k = 0; k < seq.count; k++) begin
                if (t >= seq.terms[k].t0) begin
                    dt = t - seq.terms[k].t0;
                    term_val = seq.terms[k].b * $exp(seq.terms[k].a * dt);
                    if (seq.terms[k].m > 1) begin
                        pow_i = 1;
                        for (int pwr = 1; pwr < seq.terms[k].m; pwr++) begin
                            pow_i *= pwr;
                            term_val *= dt;
                        end
                        term_val /= pow_i;
                    end
                    sum += term_val;
                end
            end
            eval_xreal_at_t = sum;
        end
    endfunction

    function automatic void xreal_print(input string label, input xreal_seq_t seq);
        begin
            $display("%s (%0d terms):", label, seq.count);
            for (int i = 0; i < seq.count; i++) begin
                $display("  [%0d] b=%0.6e  a=%0.6e  m=%0d  t0=%0.3e",
                         i,
                         seq.terms[i].b,
                         seq.terms[i].a,
                         seq.terms[i].m,
                         seq.terms[i].t0);
            end
        end
    endfunction

    // =========================================================================
    // XBIT — Section III-C logic domain (0, 1, z, x) as event-driven edges
    // =========================================================================
    parameter int MAX_XBIT_EDGES = 512;

    typedef enum logic [1:0] {
        XBIT_VAL_0 = 2'b00,
        XBIT_VAL_1 = 2'b01,
        XBIT_VAL_Z = 2'b10,
        XBIT_VAL_X = 2'b11
    } xbit_val_e;

    typedef struct {
        real      t_edge;
        xbit_val_e level;
    } xbit_edge_t;

    typedef struct {
        xbit_edge_t ev [0:MAX_XBIT_EDGES-1];
        int         count;
        xbit_val_e  level_at_zero;
    } xbit_seq_t;

    // PFD states — Section IV-A
    typedef enum int {
        PFD_ZERO = 0,
        PFD_UP   = 1,
        PFD_DOWN = -1
    } pfd_state_e;

    function automatic void xbit_clear(output xbit_seq_t seq);
        begin
            seq.count         = 0;
            seq.level_at_zero = XBIT_VAL_0;
        end
    endfunction

    function automatic void xbit_add_edge(
        ref   xbit_seq_t seq,
        input real       t_edge,
        input xbit_val_e level
    );
        begin
            if (seq.count < MAX_XBIT_EDGES) begin
                seq.ev[seq.count].t_edge = t_edge;
                seq.ev[seq.count].level  = level;
                seq.count++;
            end else begin
                $error("xbit_add_edge: MAX_XBIT_EDGES exceeded");
            end
        end
    endfunction

    // Standard normal N(0,1) via Box-Muller; seed updated in place.
    function automatic real xreal_randn(ref int seed);
        real u1, u2;
        begin
            u1 = (real'($urandom(seed)) + 1.0) / 4294967296.0;
            u2 = (real'($urandom(seed)) + 1.0) / 4294967296.0;
            xreal_randn = $sqrt(-2.0 * $ln(u1))
                        * $cos(2.0 * 3.14159265358979323846 * u2);
        end
    endfunction

    // White timing jitter on each XBIT edge: t' = t + N(0, sigma^2).
    // Enforce strict monotonicity so edges never reverse order.
    function automatic void xbit_apply_white_jitter(
        ref   xbit_seq_t seq,
        input real       t_start,
        input real       sigma_s,
        ref   int        seed
    );
        real t_prev, t_j;
        real eps;
        begin
            if (sigma_s <= 0.0 || seq.count <= 0)
                return;
            eps    = 1.0e-15;
            t_prev = t_start;
            for (int i = 0; i < seq.count; i++) begin
                t_j = seq.ev[i].t_edge + sigma_s * xreal_randn(seed);
                if (t_j <= t_prev)
                    t_j = t_prev + eps;
                seq.ev[i].t_edge = t_j;
                t_prev = t_j;
            end
        end
    endfunction

    // Plot helper: evaluate xbit level at time t (0.0/1.0/0.5/-1 for 0/1/z/x)
    function automatic real eval_xbit_at_t(input xbit_seq_t seq, input real t);
        xbit_val_e lvl;
        begin
            lvl = seq.level_at_zero;
            for (int i = 0; i < seq.count; i++) begin
                if (t >= seq.ev[i].t_edge)
                    lvl = seq.ev[i].level;
            end
            case (lvl)
                XBIT_VAL_0: eval_xbit_at_t = 0.0;
                XBIT_VAL_1: eval_xbit_at_t = 1.0;
                XBIT_VAL_Z: eval_xbit_at_t = 0.5;
                default:    eval_xbit_at_t = -1.0;
            endcase
        end
    endfunction

    function automatic real pfd_state_to_real(input pfd_state_e st);
        pfd_state_to_real = real'(st);
    endfunction

    // Build square-wave xbit edges from constant frequency [Hz]
    function automatic xbit_seq_t xbit_gen_square_freq(
        input real       f_hz,
        input real       t_start,
        input real       phase_rad,
        input real       t_stop,
        input xbit_val_e level_at_zero
    );
        xbit_seq_t seq;
        real       half_period, t_edge, lvl1;
        xbit_val_e next_lvl;
        begin
            xbit_clear(seq);
            seq.level_at_zero = level_at_zero;
            if (f_hz <= 0.0) begin
                xbit_gen_square_freq = seq;
            end else begin
                half_period = 0.5 / f_hz;
                t_edge      = t_start + (phase_rad / (2.0 * 3.14159265358979323846)) / f_hz;
                if (t_edge < t_start)
                    t_edge = t_start;
                next_lvl = (level_at_zero == XBIT_VAL_0) ? XBIT_VAL_1 : XBIT_VAL_0;
                while (t_edge <= t_stop && seq.count < MAX_XBIT_EDGES) begin
                    xbit_add_edge(seq, t_edge, next_lvl);
                    next_lvl = (next_lvl == XBIT_VAL_0) ? XBIT_VAL_1 : XBIT_VAL_0;
                    t_edge  += half_period;
                end
                xbit_gen_square_freq = seq;
            end
        end
    endfunction

    // Instantaneous VCO phase [rad] in [0, 2*pi) from integral of fout XREAL:
    //   cycles(t) = integral_0^t fout(tau) d tau   [Hz·s = rotations]
    //   phase(t)  = 2*pi * frac(cycles(t))
    function automatic real phase_wrap_0_2pi(input real phi);
        real two_pi, r;
        begin
            two_pi = 2.0 * 3.14159265358979323846;
            r = phi - two_pi * $floor(phi / two_pi);
            if (r < 0.0)
                r += two_pi;
            phase_wrap_0_2pi = r;
        end
    endfunction

    function automatic real xreal_inst_phase_rad_at_t(
        input xreal_seq_t fout,
        input real        t
    );
        real two_pi, cycles, cycle_frac;
        begin
            two_pi     = 2.0 * 3.14159265358979323846;
            cycles     = integrate_xreal_m1_at_t(fout, t);
            cycle_frac = cycles - $floor(cycles);
            if (cycle_frac < 0.0)
                cycle_frac += 1.0;
            xreal_inst_phase_rad_at_t = two_pi * cycle_frac;
        end
    endfunction

    // Bisection: find t in (t_lo, t_stop] where integral fout = target_cycles [Hz·s].
    function automatic real xreal_find_t_for_cycles(
        input xreal_seq_t fout,
        input real        t_lo,
        input real        target_cycles,
        input real        t_stop
    );
        real t_a, t_b, t_m, c_b;
        int  iter;
        begin
            t_a = t_lo;
            t_b = t_lo + 1.0e-10;
            if (t_b > t_stop)
                t_b = t_stop;
            c_b = integrate_xreal_m1_at_t(fout, t_b);
            while (c_b < target_cycles && t_b < t_stop) begin
                t_b += 1.0e-9;
                c_b = integrate_xreal_m1_at_t(fout, t_b);
            end
            if (c_b < target_cycles) begin
                xreal_find_t_for_cycles = t_stop;
            end else begin
                for (iter = 0; iter < 64; iter = iter + 1) begin
                    t_m = 0.5 * (t_a + t_b);
                    if (integrate_xreal_m1_at_t(fout, t_m) < target_cycles)
                        t_a = t_m;
                    else
                        t_b = t_m;
                end
                xreal_find_t_for_cycles = 0.5 * (t_a + t_b);
            end
        end
    endfunction

    // Place XBIT edges when integral fout crosses half-cycle boundaries — matches VCO phase.
    // Module equivalent: xreal_to_xbit #(.EDGE_MODE(1))
    function automatic xbit_seq_t xbit_gen_from_fout_integrated(
        input xreal_seq_t fout,
        input real        t_start,
        input real        t_stop,
        input real        phase_offset_rad
    );
        xbit_seq_t seq;
        real       cycle_pos, c_target, t_last, t_edge;
        xbit_val_e next_lvl, lvl_at_t;
        begin
            xbit_clear(seq);
            cycle_pos = integrate_xreal_m1_at_t(fout, t_start)
                      + phase_offset_rad / (2.0 * 3.14159265358979323846);
            if (cycle_pos - $floor(cycle_pos) < 0.5)
                lvl_at_t = XBIT_VAL_0;
            else
                lvl_at_t = XBIT_VAL_1;
            seq.level_at_zero = lvl_at_t;

            c_target = $floor(cycle_pos) + 0.5;
            if (c_target <= cycle_pos)
                c_target += 0.5;
            next_lvl = (lvl_at_t == XBIT_VAL_0) ? XBIT_VAL_1 : XBIT_VAL_0;
            t_last   = t_start;

            while (seq.count < MAX_XBIT_EDGES && t_last < t_stop) begin
                t_edge = xreal_find_t_for_cycles(fout, t_last, c_target, t_stop);
                if (t_edge >= t_stop || t_edge <= t_last)
                    break;
                xbit_add_edge(seq, t_edge, next_lvl);
                next_lvl  = (next_lvl == XBIT_VAL_0) ? XBIT_VAL_1 : XBIT_VAL_0;
                c_target += 0.5;
                t_last    = t_edge;
            end
            xbit_gen_from_fout_integrated = seq;
        end
    endfunction

    // Build xbit from XREAL frequency sequence (uses f at t_start).
    // Module equivalent: xreal_to_xbit #(.EDGE_MODE(0), .PHASE_SRC(0))
    function automatic xbit_seq_t xbit_from_xreal_freq(
        input xreal_seq_t freq_seq,
        input real        t_start,
        input real        phase_rad,
        input real        t_stop
    );
        real f_hz;
        begin
            f_hz = eval_xreal_at_t(freq_seq, t_start);
            if (f_hz <= 0.0)
                f_hz = 1.0;
            xbit_from_xreal_freq = xbit_gen_square_freq(
                f_hz, t_start, phase_rad, t_stop, XBIT_VAL_0
            );
        end
    endfunction

    // Feedback path: phase from integrated VCO frequency (updates each event).
    // Optional phase_offset_rad trims static divider / routing delay [rad].
    // Module equivalent: xreal_to_xbit #(.EDGE_MODE(0), .PHASE_SRC(1))
    function automatic xbit_seq_t xbit_from_xreal_freq_vco_phase(
        input xreal_seq_t freq_seq,
        input real        t_start,
        input real        t_stop,
        input real        phase_offset_rad
    );
        real f_hz, phase_rad;
        begin
            f_hz = eval_xreal_at_t(freq_seq, t_start);
            if (f_hz <= 0.0)
                f_hz = 1.0;
            phase_rad = xreal_inst_phase_rad_at_t(freq_seq, t_start) + phase_offset_rad;
            phase_rad = phase_wrap_0_2pi(phase_rad);
            xbit_from_xreal_freq_vco_phase = xbit_gen_square_freq(
                f_hz, t_start, phase_rad, t_stop, XBIT_VAL_0
            );
        end
    endfunction

    function automatic void xbit_print(input string label, input xbit_seq_t seq);
        begin
            $display("%s (%0d edges, level@0=%0d):", label, seq.count, seq.level_at_zero);
            for (int i = 0; i < seq.count; i++) begin
                $display("  [%0d] t=%0.3e s  level=%0d",
                         i, seq.ev[i].t_edge, seq.ev[i].level);
            end
        end
    endfunction

    // =========================================================================
    // Charge pump — Section IV-B eq. (5)
    // ICP = < [Iend,0,1,tk], [(I0-Iend), -1/tau, 1, tk] >
    // =========================================================================
    function automatic void cp_append_transition(
        ref   xreal_seq_t seq,
        input real        Iend,
        input real        I0,
        input real        tau,
        input real        tk
    );
        real decay_pole;
        begin
            if (tau <= 0.0)
                decay_pole = -1.0e12;
            else
                decay_pole = -1.0 / tau;
            // Eq. (5) replaces ICP after tk (not an incremental add on prior terms).
            xreal_clear(seq);
            xreal_add_term(seq, Iend, 0.0, 1, tk);
            xreal_add_term(seq, I0 - Iend, decay_pole, 1, tk);
            xreal_compress(seq);
        end
    endfunction

    function automatic real cp_eval_current_at_t(
        input xreal_seq_t icp,
        input real        t
    );
        cp_eval_current_at_t = eval_xreal_at_t(icp, t);
    endfunction

endpackage
