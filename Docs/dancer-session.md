# ダンサー収録セッション 操作ガイド（2026-07-14）

トライ&エラーのループ: **F9 収録 → Space（カウントダウン）freeze → F10 で USDZ+QR → 携帯で AR 確認 → Print Export でワンクリック STL**。全部 Play を止めずに回せる。

## キー操作（Game ビューにフォーカスがあること）

| キー | 動作 |
|---|---|
| **F9** | RCSV 収録 開始/停止（`SensorRecorder`、RecordingBase+日時フォルダ） |
| **Space** | live: カウントダウン（既定 5 秒、画面中央に大表示）→ freeze。**Rec 中でも有効**（録画は継続、描画/TSDF/BT だけ静止）。再度 Space で解除は即時、カウントダウン中の Space はキャンセル。playback 中は従来通り即 pause |
| **F10** | `OperatorPublisher`: capture（trail 長は Inspector の `trailSamples`）→ GLB+USDZ 書き出し → LFKS upload → **Display 2・3 の右上に QR**（Display 1 のメイン view は邪魔しない。出し先は `QrOverlay.targetDisplays`）。freeze/pause 中に実行するのが基本 |
| **F11** | QR オーバーレイを消す |

## trail 長の変え方

`OperatorPublisher` の Inspector `trailSamples`（2..32、15 ≈ 0.65 秒 @ ~23Hz BT）。
**freeze/pause したまま値を変えて F10 を押し直せば、同じ瞬間から別 trail 長で再生成できる**（pose リングは常時 32 フレーム保持）。32 超が欲しくなったら `BonePoseHistory.MaxK` 拡張が必要（未対応）。

## 3D プリント書き出し

`Window > Print Export` → **[Fuse → Close → Export STL (ワンクリック)]**。
- 出力: `~/Documents/FloatingVectorsPrints/print_<stamp>.stl`（`targetHeightMm`、既定 300mm、winding 自動補正）
- やり直し: **Restore** で融合前に戻る（表示 volume も戻る）
- パネル右下 **[出力フォルダを開く]** で確認

## 設定・前提

- publish 設定: `Assets/Settings/ExperienceConfig.asset`（**dryRunPublish=false 済み**）を `OperatorPublisher.config` に割当。テスト時に dry run に戻したければここを ON
- LFKS token: `persistentDataPath/lfks-token.txt` 配置済み（期限 2026-10-30）
- USDZ は binary usdc（`~/.venvs/usd` の python 検出済み）
- 失敗時: ファイルは `FloatingVectorsPrints/` に残る → `Docs/lfks-upload-guide.md` の upload.ps1 で手動アップロード
- Rec 中の freeze 中は bodies_main が書かれない（Windows 再生は live BT なので実害なし）

## 検証済み（2026-07-13、playback 4cam で実測）

- F10 一連: dry run 7.3s / 実 LFKS upload 20.8s（URL 取得・QR 表示まで）
- 同一瞬間の trail 15→32 再生成 OK（pause 中の curve 再ビルド確認済み）
- STL チェーン: 6.6s（405k tris / 19MB）
- ライブ 4cam での freeze + Rec 継続は**現地カメラ接続後に要実機確認**（コードパスは playback で未通過）
