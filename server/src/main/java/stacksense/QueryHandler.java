package stacksense;

import io.javalin.http.Context;
import java.util.List;

/**
 * Read side for the dashboard.
 *
 * The waveform is served as the same packed byte stream the device uploaded,
 * not as JSON. Re-encoding 1011 samples into JSON would inflate 5 KB into
 * roughly 30 KB and force a second decoder to exist; instead the browser runs
 * the same 5-byte-stride decode the server does.
 */
public final class QueryHandler {

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

    /** DELETE /api/sets/{id} */
    public static void remove(Context ctx) {
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

    private static java.util.Map<String, Object> err(String reason) {
        java.util.Map<String, Object> m = new java.util.LinkedHashMap<>();
        m.put("status", "error");
        m.put("reason", reason);
        return m;
    }
}
