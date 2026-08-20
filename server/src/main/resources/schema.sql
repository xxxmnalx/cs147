-- StackSense storage.
--
-- One row per completed set. The waveform is kept in exactly the packed form
-- the device sent, so nothing is lost in translation and the decoder on the
-- browser side can be the same one the server uses.

CREATE DATABASE IF NOT EXISTS stacksense
  CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

USE stacksense;

CREATE TABLE IF NOT EXISTS sets (
  id           BIGINT       NOT NULL AUTO_INCREMENT,

  -- The firmware restarts set_number at 1 on every boot, so it is a label
  -- rather than an identity: two different sets can both report set_number 1.
  -- Rows are keyed on the surrogate id above instead.
  -- TODO add device_id once a second board exists. Every row is currently
  -- assumed to come from the same device.
  set_number   INT          NOT NULL,

  reps         INT          NOT NULL,
  baseline_mm  INT          NOT NULL,
  resting_g    FLOAT        NOT NULL,
  sample_count INT          NOT NULL,

  -- Sum of every dt in the waveform. A lower bound on real elapsed time,
  -- because dt saturates at 255 ms per sample.
  duration_ms  INT          NOT NULL,

  -- The value bufferChecksum() produced on the device, already verified
  -- against the body before this row was written.
  checksum     INT UNSIGNED NOT NULL,

  -- Packed Sample records: uint8 dt, int16 height, int16 accel, little endian,
  -- 5 bytes each. MEDIUMBLOB rather than BLOB so that raising MAX_SAMPLES on
  -- the firmware cannot silently truncate at the 64 KB BLOB ceiling.
  waveform     MEDIUMBLOB   NOT NULL,

  received_at  TIMESTAMP    NOT NULL DEFAULT CURRENT_TIMESTAMP,

  PRIMARY KEY (id),
  KEY idx_received (received_at)
) ENGINE=InnoDB;
