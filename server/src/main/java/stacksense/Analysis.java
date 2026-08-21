package stacksense;

import java.util.ArrayList;
import java.util.List;

/**
 * Derives per-rep metrics from a stored waveform.
 *
 * The upload carries a rep *count* but no rep boundaries, so they are
 * recovered here. Detection uses a Schmitt trigger on the height trace rather
 * than replaying the firmware state machine: the server sees the whole set at
 * once and can set its gate from the actual range, where the device only ever
 * sees the last few samples.
 *
 * The detected count is reported alongside the device's own so a disagreement
 * is visible instead of silently overwriting one with the other.
 */
public final class Analysis {

    /** Same floor the firmware uses, so both sides reject the same twitches. */
    private static final int MIN_ROM_MM = 50;

    private static final int SMOOTH_WINDOW = 5;

    private Analysis() {
    }

    public static final class Rep {
        public int index;
        public int romMm;
        public int concentricMs;   // valley to peak, the working phase
        public int eccentricMs;    // peak to the next valley, the return
        public double eccentricRatio;
        public int peakVelocityMmS;
        public int startMs;
        public int peakMs;
        public int endMs;
    }

    public static final class Result {
        public int detectedReps;
        public int deviceReps;
        public List<Rep> reps = new ArrayList<>();

        // Fatigue: how the first rep compares with the last.
        public Integer romFirstMm;
        public Integer romLastMm;
        public Integer romDropMm;
        public Double  romDropPct;
        public Integer durFirstMs;
        public Integer durLastMs;
        public Integer durChangeMs;

        // Consistency across the set.
        public Double romMeanMm;
        public Double romStdDevMm;
        public Double romCvPct;      // stddev over mean, comparable between sets
        public Double tempoMeanMs;

        public String note;
    }

    public static Result analyse(byte[] waveform, int deviceReps) {
        Result r = new Result();
        r.deviceReps = deviceReps;

        SampleCodec.Sample[] s = SampleCodec.decode(waveform);
        int n = s.length;
        if (n < 20) {
            r.note = "only " + n + " samples, too few to analyse";
            return r;
        }

        int[] t = new int[n];
        int[] h = new int[n];
        int clock = 0;
        for (int i = 0; i < n; i++) {
            clock += s[i].dtMs;
            t[i] = clock;
            h[i] = s[i].heightMM;
        }
        int[] sm = smooth(h, SMOOTH_WINDOW);

        // Percentiles rather than min/max: one spike should not set the gate.
        int lo = percentile(sm, 10);
        int hi = percentile(sm, 90);
        int span = hi - lo;
        if (span < MIN_ROM_MM) {
            r.note = "height span of " + span + "mm never reaches a rep";
            return r;
        }
        int enter = lo + (int) (span * 0.55);
        int exit  = lo + (int) (span * 0.35);

        // Runs spent above the gate. Hysteresis keeps noise near the threshold
        // from splitting one rep into several.
        List<int[]> tops = new ArrayList<>();
        boolean up = false;
        int runStart = 0;
        for (int i = 0; i < n; i++) {
            if (!up && sm[i] >= enter) {
                up = true;
                runStart = i;
            } else if (up && sm[i] <= exit) {
                up = false;
                tops.add(new int[]{runStart, i});
            }
        }
        if (up) {
            tops.add(new int[]{runStart, n - 1});
        }

        for (int k = 0; k < tops.size(); k++) {
            int a = tops.get(k)[0];
            int b = tops.get(k)[1];

            int peak = a;
            for (int i = a; i <= b; i++) {
                if (sm[i] > sm[peak]) peak = i;
            }
            int from = (k == 0) ? 0 : tops.get(k - 1)[1];
            int valleyBefore = from;
            for (int i = from; i <= a; i++) {
                if (sm[i] < sm[valleyBefore]) valleyBefore = i;
            }
            int to = (k == tops.size() - 1) ? n - 1 : tops.get(k + 1)[0];
            int valleyAfter = b;
            for (int i = b; i <= to; i++) {
                if (sm[i] < sm[valleyAfter]) valleyAfter = i;
            }

            int rom = sm[peak] - sm[valleyBefore];
            if (rom < MIN_ROM_MM) {
                continue;
            }

            Rep rep = new Rep();
            rep.index = r.reps.size() + 1;
            rep.romMm = rom;
            rep.startMs = t[valleyBefore];
            rep.peakMs = t[peak];
            rep.endMs = t[valleyAfter];
            rep.concentricMs = t[peak] - t[valleyBefore];
            rep.eccentricMs = t[valleyAfter] - t[peak];
            rep.eccentricRatio = rep.concentricMs > 0
                    ? round2((double) rep.eccentricMs / rep.concentricMs) : 0;
            rep.peakVelocityMmS = peakVelocity(sm, t, valleyBefore, peak);
            r.reps.add(rep);
        }

        r.detectedReps = r.reps.size();
        if (r.detectedReps == 0) {
            r.note = "no rep cleared the " + MIN_ROM_MM + "mm floor";
            return r;
        }

        Rep first = r.reps.get(0);
        Rep last = r.reps.get(r.detectedReps - 1);
        r.romFirstMm = first.romMm;
        r.romLastMm = last.romMm;
        r.romDropMm = first.romMm - last.romMm;
        r.romDropPct = first.romMm > 0 ? round2(100.0 * r.romDropMm / first.romMm) : null;

        int durFirst = first.concentricMs + first.eccentricMs;
        int durLast = last.concentricMs + last.eccentricMs;
        r.durFirstMs = durFirst;
        r.durLastMs = durLast;
        r.durChangeMs = durLast - durFirst;

        double sum = 0, tempo = 0;
        for (Rep x : r.reps) {
            sum += x.romMm;
            tempo += x.concentricMs + x.eccentricMs;
        }
        double mean = sum / r.detectedReps;
        double var = 0;
        for (Rep x : r.reps) {
            var += (x.romMm - mean) * (x.romMm - mean);
        }
        // Population stddev: this is the whole set, not a sample from a larger one.
        double sd = Math.sqrt(var / r.detectedReps);
        r.romMeanMm = round2(mean);
        r.romStdDevMm = round2(sd);
        r.romCvPct = mean > 0 ? round2(100.0 * sd / mean) : null;
        r.tempoMeanMs = round2(tempo / r.detectedReps);

        if (r.detectedReps != deviceReps) {
            r.note = "detected " + r.detectedReps + " reps, the device counted " + deviceReps;
        }
        return r;
    }

    private static int peakVelocity(int[] h, int[] t, int from, int to) {
        int best = 0;
        for (int i = from; i < to; i++) {
            int dt = t[i + 1] - t[i];
            if (dt <= 0) continue;
            int v = (int) (1000L * (h[i + 1] - h[i]) / dt);
            if (v > best) best = v;
        }
        return best;
    }

    private static int[] smooth(int[] v, int w) {
        int n = v.length;
        int[] out = new int[n];
        int half = w / 2;
        for (int i = 0; i < n; i++) {
            int a = Math.max(0, i - half);
            int b = Math.min(n - 1, i + half);
            long sum = 0;
            for (int j = a; j <= b; j++) sum += v[j];
            out[i] = (int) (sum / (b - a + 1));
        }
        return out;
    }

    private static int percentile(int[] v, int p) {
        int[] c = v.clone();
        java.util.Arrays.sort(c);
        int idx = (int) Math.round((p / 100.0) * (c.length - 1));
        return c[Math.max(0, Math.min(c.length - 1, idx))];
    }

    private static double round2(double d) {
        return Math.round(d * 100.0) / 100.0;
    }
}
