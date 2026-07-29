# Release packaging

`package_firmware.py` converts successful CI build output into a self-contained ZIP.
It does not compile firmware.

For ESP-IDF it reads the project's generated `build/flasher_args.json`, preserving
the framework-provided flash offsets. For Arduino it prefers the exported merged
binary and falls back to the standard ESP32-C6 bootloader, partition, and application
offsets when necessary.

Each package contains:

- every relevant binary under `bin/`;
- `manifest.json` with source and framework metadata, offsets, sizes, and SHA-256;
- `flash.sh` and `flash.bat` helpers;
- ESP-IDF's original `flasher_args.json` under `metadata/` when applicable.

The workflows invoke the script after a successful build:

```sh
python releases/package_firmware.py esp-idf \
  --project examples/esp-idf/01_AXP2101 \
  --build-dir examples/esp-idf/01_AXP2101/build \
  --framework-version v5.5.5 \
  --output-dir release-artifacts
```

Generated archives are uploaded as GitHub Actions artifacts and are not committed.
