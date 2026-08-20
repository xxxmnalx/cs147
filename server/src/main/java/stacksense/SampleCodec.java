package stacksense;

/**
 * Decoder for the packed sample stream the ESP32 posts.
 * Mirrors struct Sample and bufferChecksum() in src/main.cpp.
 */
public final class SampleCodec {

    /** sizeof(struct Sample): uint8 dt + int16 height + int16 accel, packed. */
    public static final int SAMPLE_BYTES = 5;

    private SampleCodec() {
    }

    public static final class Sample {
        public final int dtMs;     // milliseconds since the previous sample, 0-255
        public final int heightMM; // height above the set baseline
        public final int accelMg;  // deviation from resting acceleration, milli-g

        Sample(int dtMs, int heightMM, int accelMg) {
            this.dtMs = dtMs;
            this.heightMM = heightMM;
            this.accelMg = accelMg;
        }
    }

    /**
     * Rotate-xor fold, identical to bufferChecksum() in the firmware.
     * That version folds a uint32_t, so the right shift must be unsigned here
     * and the result is widened to a long to stay positive.
     */
    public static long checksum(byte[] body) {
        int sum = 0;
        for (byte b : body) {
            sum = (sum << 1) ^ (sum >>> 31) ^ (b & 0xFF);
        }
        return sum & 0xFFFFFFFFL;
    }

    public static Sample[] decode(byte[] body) {
        int count = body.length / SAMPLE_BYTES;
        Sample[] out = new Sample[count];
        for (int i = 0; i < count; i++) {
            int o = i * SAMPLE_BYTES;
            out[i] = new Sample(body[o] & 0xFF, int16(body, o + 1), int16(body, o + 3));
        }
        return out;
    }

    /** Signed little-endian 16-bit read, matching the ESP32 byte order. */
    private static int int16(byte[] b, int o) {
        return (short) ((b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8));
    }
}
