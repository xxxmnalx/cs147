package stacksense;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import io.javalin.json.JsonMapper;
import java.lang.reflect.Type;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class Main {

    private static final Logger log = LoggerFactory.getLogger(Main.class);
    private static final int DEFAULT_PORT = 8080;

    private Main() {
    }

    public static void main(String[] args) {
        int port = DEFAULT_PORT;
        String fromEnv = System.getenv("STACKSENSE_PORT");
        if (fromEnv != null && !fromEnv.isBlank()) {
            port = Integer.parseInt(fromEnv);
        }

        Javalin app = Javalin.create(config -> {
            config.jsonMapper(gsonMapper());
            config.staticFiles.add(staticFiles -> {
                staticFiles.directory = "/public";
                staticFiles.location = Location.CLASSPATH;
            });
            // Log every request, not just the ones a handler claims. Without
            // this a device hitting the wrong path or method 404s silently and
            // looks identical to a device that never connected at all.
            config.requestLogger.http((ctx, ms) ->
                    log.info("{} {} {} <- {} ({} ms)",
                            ctx.method(), ctx.path(), ctx.statusCode(), ctx.ip(), Math.round(ms)));
        });

        boolean dbReady = Db.init();

        app.get("/api/health", ctx -> ctx.result(dbReady ? "ok" : "ok (no database)"));
        app.post("/api/upload", UploadHandler::handle);
        app.get("/api/sets", QueryHandler::list);
        app.get("/api/sets/{id}/waveform", QueryHandler::waveform);
        app.delete("/api/sets/{id}", QueryHandler::remove);

        // 0.0.0.0 so the ESP32 and the EC2 security group can reach it, not just localhost.
        app.start("0.0.0.0", port);
        log.info("StackSense server listening on 0.0.0.0:{}", port);
    }

    /** Javalin defaults to Jackson; this project ships Gson instead. */
    private static JsonMapper gsonMapper() {
        // Default Gson escapes '=' and '&' as \u003d / \u0026, which mangles the
        // query-string echoes in error messages.
        final Gson gson = new GsonBuilder().disableHtmlEscaping().create();
        return new JsonMapper() {
            @Override
            public String toJsonString(Object obj, Type type) {
                return gson.toJson(obj, type);
            }

            @Override
            public <T> T fromJsonString(String json, Type targetType) {
                return gson.fromJson(json, targetType);
            }
        };
    }
}
