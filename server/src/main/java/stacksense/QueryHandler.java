package stacksense;

import io.javalin.http.Context;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Read side for the dashboard.
 *
 * The waveform is served as the same packed byte stream the device uploaded,
 * not as JSON. Re-encoding 1011 samples into JSON would inflate 5 KB into
 * roughly 30 KB and force a second decoder to exist; instead the browser runs
 * the same 5-byte-stride decode the server does.
 */
public final class QueryHandler {

    private static final Logger log = LoggerFactory.getLogger(QueryHandler.class);

    private static final int DEFAULT_LIMIT = 50;

    private QueryHandler() {
    }

    /** GET /api/sets — newest sets first, metadata only. */
    public static void list(Context ctx) {
        if (!Db.isReady()) {
            ctx.status(503).json(err("database not available"));
            return;
        }
        int limit = DEFAULT_LIMIT;
        String raw = ctx.queryParam("limit");
        if (raw != null && !raw.isBlank()) {
            try {
                limit = Math.max(1, Math.min(500, Integer.parseInt(raw)));
            } catch (NumberFormatException ignored) {
                // fall back to the default rather than rejecting the page load
            }
        }
        List<Db.SetRow> rows = Db.listSets(limit);
        ctx.json(rows);
    }

    /** GET /api/sets/{id}/waveform — raw packed samples, 5 bytes each. */
    public static void waveform(Context ctx) {
        if (!Db.isReady()) {
            ctx.status(503).json(err("database not available"));
            return;
        }
        long id;
        try {
            id = Long.parseLong(ctx.pathParam("id"));
        } catch (NumberFormatException e) {
            ctx.status(400).json(err("id must be a number"));
            return;
        }

        byte[] blob = Db.waveform(id);
        if (blob == null) {
            ctx.status(404).json(err("no set with id " + id));
            return;
        }
        ctx.contentType("application/octet-stream").result(blob);
    }

    /** GET /api/analysis/{id} — per-rep metrics derived from the stored waveform. */
    public static void analysis(Context ctx) {
        if (!Db.isReady()) {
            ctx.status(503).json(err("database not available"));
            return;
        }
        long id;
        try {
            id = Long.parseLong(ctx.pathParam("id"));
        } catch (NumberFormatException e) {
            ctx.status(400).json(err("id must be a number"));
            return;
        }
        Db.SetRow row = Db.getSet(id);
        byte[] blob = Db.waveform(id);
        if (row == null || blob == null) {
            ctx.status(404).json(err("no set with id " + id));
            return;
        }
        ctx.json(Analysis.analyse(blob, row.reps));
    }

    /**
     * DELETE /api/sets/{id}
     *
     * Guarded by a shared key in the X-Admin-Key header. Deletion is the only
     * destructive route and the dashboard is on a public domain, so this fails
     * closed: with ADMIN_KEY unset nothing can be deleted at all.
     */
    public static void remove(Context ctx) {
        String expected = System.getenv("ADMIN_KEY");
        if (expected == null || expected.isBlank()) {
            log.warn("delete refused: ADMIN_KEY is not configured on this server");
            ctx.status(503).json(err("deletion is disabled: no admin key configured"));
            return;
        }
        String supplied = ctx.header("X-Admin-Key");
        if (supplied == null || !constantTimeEquals(supplied, expected)) {
            log.warn("delete refused: bad admin key from {}", ctx.ip());
            ctx.status(401).json(err("wrong password"));
            return;
        }

        if (!Db.isReady()) {
            ctx.status(503).json(err("database not available"));
            return;
        }
        long id;
        try {
            id = Long.parseLong(ctx.pathParam("id"));
        } catch (NumberFormatException e) {
            ctx.status(400).json(err("id must be a number"));
            return;
        }
        if (!Db.deleteSet(id)) {
            ctx.status(404).json(err("no set with id " + id));
            return;
        }
        java.util.Map<String, Object> ok = new java.util.LinkedHashMap<>();
        ok.put("status", "ok");
        ok.put("deleted", id);
        ctx.json(ok);
    }

    /** Compares without leaking the answer through how early it stops. */
    private static boolean constantTimeEquals(String a, String b) {
        byte[] x = a.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        byte[] y = b.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        return java.security.MessageDigest.isEqual(x, y);
    }

    private static java.util.Map<String, Object> err(String reason) {
        java.util.Map<String, Object> m = new java.util.LinkedHashMap<>();
        m.put("status", "error");
        m.put("reason", reason);
        return m;
    }
}
