package stacksense;

import com.zaxxer.hikari.HikariConfig;
import com.zaxxer.hikari.HikariDataSource;
import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * MySQL access for completed sets.
 *
 * Credentials come from the environment, never from the repository:
 * DB_URL, DB_USER, DB_PASS are supplied by /etc/stacksense.env on the server.
 * If they are absent the server still starts and still accepts uploads, it
 * just cannot persist them, so a database outage never costs a training set.
 */
public final class Db {

    private static final Logger log = LoggerFactory.getLogger(Db.class);

    private static HikariDataSource pool;

    private Db() {
    }

    /** Row shape for the dashboard list. The waveform is fetched separately. */
    public static final class SetRow {
        public long id;
        public int setNumber;
        public int reps;
        public int baselineMm;
        public double restingG;
        public int sampleCount;
        public int durationMs;
        public long checksum;
        public String receivedAt;
    }

    /** Builds the pool. Returns false when the server should run storage-less. */
    public static boolean init() {
        String url  = System.getenv("DB_URL");
        String user = System.getenv("DB_USER");
        String pass = System.getenv("DB_PASS");

        if (url == null || user == null || pass == null) {
            log.warn("DB_URL/DB_USER/DB_PASS not set, uploads will not be persisted");
            return false;
        }

        HikariConfig cfg = new HikariConfig();
        cfg.setJdbcUrl(url);
        cfg.setUsername(user);
        cfg.setPassword(pass);
        // The instance has under 1 GB of RAM and MySQL is capped at 20
        // connections; a small pool is plenty for one device.
        cfg.setMaximumPoolSize(4);
        cfg.setConnectionTimeout(5000);
        cfg.setPoolName("stacksense");

        try {
            pool = new HikariDataSource(cfg);
            try (Connection c = pool.getConnection();
                 Statement s = c.createStatement()) {
                s.execute("SELECT 1");
            }
            log.info("database ready");
            return true;
        } catch (SQLException | RuntimeException e) {
            log.error("database unavailable, uploads will not be persisted: {}", e.getMessage());
            pool = null;
            return false;
        }
    }

    public static boolean isReady() {
        return pool != null;
    }

    /** Inserts one set and returns its generated id, or -1 on failure. */
    public static long insertSet(int setNumber, int reps, int baselineMm, double restingG,
                                 int sampleCount, int durationMs, long checksum, byte[] waveform) {
        if (pool == null) {
            return -1;
        }
        String sql = "INSERT INTO sets "
                + "(set_number, reps, baseline_mm, resting_g, sample_count, duration_ms, checksum, waveform) "
                + "VALUES (?, ?, ?, ?, ?, ?, ?, ?)";

        try (Connection c = pool.getConnection();
             PreparedStatement ps = c.prepareStatement(sql, Statement.RETURN_GENERATED_KEYS)) {
            ps.setInt(1, setNumber);
            ps.setInt(2, reps);
            ps.setInt(3, baselineMm);
            ps.setDouble(4, restingG);
            ps.setInt(5, sampleCount);
            ps.setInt(6, durationMs);
            ps.setLong(7, checksum);
            ps.setBytes(8, waveform);
            ps.executeUpdate();

            try (ResultSet keys = ps.getGeneratedKeys()) {
                return keys.next() ? keys.getLong(1) : -1;
            }
        } catch (SQLException e) {
            log.error("insert failed: {}", e.getMessage());
            return -1;
        }
    }

    /** Newest sets first, without the waveform bytes. */
    public static List<SetRow> listSets(int limit) {
        List<SetRow> out = new ArrayList<>();
        if (pool == null) {
            return out;
        }
        String sql = "SELECT id, set_number, reps, baseline_mm, resting_g, sample_count, "
                + "duration_ms, checksum, received_at FROM sets ORDER BY id DESC LIMIT ?";

        try (Connection c = pool.getConnection();
             PreparedStatement ps = c.prepareStatement(sql)) {
            ps.setInt(1, limit);
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    SetRow r = new SetRow();
                    r.id          = rs.getLong("id");
                    r.setNumber   = rs.getInt("set_number");
                    r.reps        = rs.getInt("reps");
                    r.baselineMm  = rs.getInt("baseline_mm");
                    r.restingG    = rs.getDouble("resting_g");
                    r.sampleCount = rs.getInt("sample_count");
                    r.durationMs  = rs.getInt("duration_ms");
                    r.checksum    = rs.getLong("checksum");
                    r.receivedAt  = String.valueOf(rs.getTimestamp("received_at"));
                    out.add(r);
                }
            }
        } catch (SQLException e) {
            log.error("list failed: {}", e.getMessage());
        }
        return out;
    }

    /** Deletes one set. Returns false when the id was not present. */
    public static boolean deleteSet(long id) {
        if (pool == null) {
            return false;
        }
        try (Connection c = pool.getConnection();
             PreparedStatement ps = c.prepareStatement("DELETE FROM sets WHERE id = ?")) {
            ps.setLong(1, id);
            return ps.executeUpdate() > 0;
        } catch (SQLException e) {
            log.error("delete failed: {}", e.getMessage());
            return false;
        }
    }

    /** Raw packed waveform for one set, or null if the id is unknown. */
    public static byte[] waveform(long id) {
        if (pool == null) {
            return null;
        }
        try (Connection c = pool.getConnection();
             PreparedStatement ps = c.prepareStatement("SELECT waveform FROM sets WHERE id = ?")) {
            ps.setLong(1, id);
            try (ResultSet rs = ps.executeQuery()) {
                return rs.next() ? rs.getBytes(1) : null;
            }
        } catch (SQLException e) {
            log.error("waveform fetch failed: {}", e.getMessage());
            return null;
        }
    }
}
