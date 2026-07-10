# Vendored: QrCodeGenerator (C#)

- Source: https://github.com/manuelbl/QrCodeGenerator (MIT — see QRCODEGEN-LICENSE.txt)
- Version: v2.1.0, pinned commit `d75ad20b2f2c04a41120c3e7c7326842fc837fcc`
- Files: QrCode.cs, QrSegment.cs, QrSegmentAdvanced.cs, BitArrayExtensions.cs,
  DataTooLongException.cs, Graphics.cs, PngBuilder.cs, ReedSolomonGenerator.cs,
  Objects.cs (namespace `Net.Codecrete.QrCodeGenerator`)
- Fetched 2026-07-10 via raw.githubusercontent.com at the pinned SHA.
- Inspected before adoption: no network / process / registry / file access
  (verified by grep, see Phase 5 log). PngBuilder uses System.IO only for
  in-memory MemoryStream/DeflateStream PNG encoding — no filesystem access.
- Used by Experience/Publishing/QrUrlPresenter.cs to render the LFKS download
  URL as a QR texture. Do not modify these files; bump the pin instead.
