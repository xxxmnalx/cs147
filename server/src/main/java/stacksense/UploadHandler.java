package stacksense;

import io.javalin.http.Context;
import java.util.LinkedHashMap;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Receives one completed set: metadata in the query string, the raw waveform
 * as an application/octet-stream body.
 *
 * Any non-2xx makes the device keep the set buffered and retry, so reject
 * anything that fails validation rather than storing a damaged set.
 */
public final class UploadHandler {

    private static final Logger log = LoggerFactory.getLogger(UploadHandler.class);

    private UploadHandler() {
    }

    public static void handle(Context ctx) {
        int setNumber;
        int reps;
        int baselineMM;
        double restingG;
        int declaredSamples;
        long declaredCrc;
        try {
            setNumber       = requiredInt(ctx, "set");
            reps            = requiredInt(ctx, "reps");
            baselineMM      = requiredInt(ctx, "base");
            restingG        = requiredDouble(ctx, "restg");
            declaredSamples = requiredInt(ctx, "n");
            declaredCrc     = requiredLong(ctx, "crc");
        } catch (IllegalArgumentException e) {
            reject(ctx, e.getMessage());
            return;
        }

        byte[] body = ctx.bodyAsBytes();

        if (body.length % SampleCodec.SAMPLE_BYTES != 0) {
            reject(ctx, "body of " + body.length + " bytes is not a whole number of samples");
            return;
        }

        int actualSamples = body.length / SampleCodec.SAMPLE_BYTES;
        if (actualSamples != declaredSamples) {
            reject(ctx, "n=" + declaredSamples + " but body carries " + actualSamples);
            return;
        }

        long actualCrc = SampleCodec.checksum(body);
        if (actualCrc != declaredCrc) {
            reject(ctx, "crc mismatch: device sent " + declaredCrc + ", server computed " + actualCrc);
            return;
        }

        SampleCodec.Sample[] samples = SampleCodec.decode(body);
        long durationMs = 0;
        for (SampleCodec.Sample s : samples) {
            durationMs += s.dtMs;
        }

        // TODO persist to MySQL once Db is implemented.
        log.info("set={} reps={} baseline={}mm restingG={} samples={} bytes={} span={}ms crc=0x{}",
                setNumber, reps, baselineMM, restingG, actualSamples, body.length,
                durationMs, Long.toHexString(actualCrc));

        Map<String, Object> ok = new LinkedHashMap<>();
        ok.put("status", "ok");
        ok.put("set", setNumber);
        ok.put("samples", actualSamples);
        ok.put("stored", false);
        ctx.status(200).json(ok);
    }

    private static void reject(Context ctx, String reason) {
        log.warn("rejected upload: {}", reason);
        Map<String, Object> err = new LinkedHashMap<>();
        err.put("status", "error");
        err.put("reason", reason);
        ctx.status(400).json(err);
    }

    private static String required(Context ctx, String name) {
        String raw = ctx.queryParam(name);
        if (raw == null || raw.isBlank()) {
            throw new IllegalArgumentException("missing query parameter '" + name + "'");
        }
        return raw;
    }

    private static int requiredInt(Context ctx, String name) {
        String raw = required(ctx, name);
        try {
            return Integer.parseInt(raw);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("'" + name + "' is not an integer: " + raw);
        }
    }

    private static long requiredLong(Context ctx, String name) {
        String raw = required(ctx, name);
        try {
            return Long.parseLong(raw);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("'" + name + "' is not a number: " + raw);
        }
    }

    private static double requiredDouble(Context ctx, String name) {
        String raw = required(ctx, name);
        try {
            return Double.parseDouble(raw);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException("'" + name + "' is not a number: " + raw);
        }
    }
}
