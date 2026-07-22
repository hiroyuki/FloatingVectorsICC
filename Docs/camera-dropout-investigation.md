# カメラ取りこぼし調査メモ

作成 2026-07-21（5080機、ライブ4カメラでの体験フロー通しテスト中に観測）。

Play を停止/再開するたびに、4台のうち1台がフレームを出さない状態で起動する現象。
物理層（USB 断）ではなく、起動シーケンスの堅牢性の問題に見える。展示前に対策を入れたい。

## 観測された事実

| 回（時刻） | 落ちた個体 | 顛末 |
|---|---|---|
| 昼 | C6 | `OrbbecException: ob_pipeline_start_with_config(...): DeviceComponentPtr is nullptr [type=12]` — ストリーム開始失敗 |
| 20:32 | （録画3台ぶんのみ） | TSDF 空 → capture 0 三角形 → 「うまくいかなかったみたい　ごめんね」 |
| 22:52 | HJ | 通し中止 |
| 22:56 | C6 | 通し中止 |
| 23:xx | F0 → C6 | Play 入れ直しで最終的に4台復帰 |

- **毎回違う個体が落ちる。** 特定カメラの故障ではない
- OS のイベントログ（過去24時間）に USB の surprise removal / エラーが **1件も記録なし**
  （`Get-WinEvent` で USB|Kernel-PnP|usbxhci の Error/Warning/Critical を確認）。
  ケーブルが物理的に抜けた・バスから落ちた、ではない
- 落ちた個体は `[PointCloudRenderer] <serial>: applySyncConfig=false ...` のログは出る
  ＝ **デバイス列挙はできているのにフレームが来ない**。昼の `DeviceComponentPtr is nullptr`
  （ストリーム開始そのものの失敗）とは別の失敗の仕方
- **Play を入れ直すと直る**（ただし別の1台が落ちることはある）
- USB 構成自体は良好：Renesas USB 3.0 eXtensible Host Controller が4基あり、
  カメラを1台ずつ別コントローラに分散できる

## 波及した被害（なぜ深刻か）

```
カメラ1台が停止
  → TSDFIntegrator が expectedCamCount=4 で4台揃うのを待ち続ける
    → 積分が一度も走らない（TotalIntegrationCount=0）
      → TSDF ボリュームが空
        → Marching Cubes 0 三角形 → capture 失敗
          → 全来場者に「うまくいかなかったみたい　ごめんね」
```

`CameraHealthMonitor` は1台消えても **Fault（全画面赤アラート）を上げなかった**ため、
運用側が体験開始前に気づけなかった。1台の不調が全来場者に波及する構造になっている。

## 原因の推測（未確定）

最有力は **Play/Stop に伴うデバイス再列挙で、確率的に1台掴み損ねている**。

根拠:
- 毎回違う個体が落ちる
- OS 層にエラーが残らない
- 入れ直すと直る
- エディタで Play/Stop を繰り返す状況で頻発

前セッションのデバイス解放が間に合わず、次の起動で掴めない、という筋。
**展示本番（ビルド済みバイナリを1日1回起動）ではこの引き金は大きく減る見込み**だが、
「起きない」と断定はできないので、ソフト側を堅くしておくべき。

## 対策案（優先順・いずれも未実装）

| # | 対策 | 効果 |
|---|---|---|
| 1 | **TSDFIntegrator が欠けたカメラを待たない** — タイムアウト後は生きているカメラだけで積分 | 1台落ちても体験が成立。謝罪画面を防げる |
| 2 | **CameraHealthMonitor が Fault を上げる** — 全画面赤アラート | 運用が体験開始前に気づける |
| 3 | **デバイス open のリトライ** — 失敗時に少し待って再試行 | このエラーは2回目で通ることが多い |

1・2 は「カメラ1台の不調が全来場者に波及する」構造を断つもので、展示前に入れる価値が高い。
3 は起動時の保険。

## 未検証（次にやるべき切り分け）

- ビルド版でも再現するか（エディタ固有か、本番でも起きるか）
- `DeviceComponentPtr is nullptr` と「列挙OKだがフレーム来ず」が同一原因か別か
- OrbbecSDK v2 側にデバイス open のタイムアウト/リトライ API があるか
  （`C:\dev\OrbbecSDK_v2\include\libobsensor\h\Device.h` / `Pipeline.h` を確認）

## 関連コンポーネント（対策の実装先）

- `Assets/Scripts/PointCloud/SensorManager.cs` — `StartLive`、デバイス列挙・パイプライン開始
- `Assets/Scripts/PointCloud/PointCloudRenderer.cs` — 各カメラのフレーム取得
- `Assets/Scripts/PointCloud/CameraHealthMonitor.cs` — Fault 判定（対策2）
- `Assets/Scripts/TSDF/TSDFIntegrator.cs` — `expectedCamCount` の待ち合わせ（対策1）
