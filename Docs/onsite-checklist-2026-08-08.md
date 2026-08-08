# 現場動作確認チェックリスト(2026-08-07〜08 の変更分)

対象コミット: 1e05bc1 / c990a85 / caaccdb / 69301e8 / a837d42 / ead1a72。
恒常的なセットアップ項目は [onsite-checklist.md](./onsite-checklist.md) を参照。
ここは**この 2 日間で入った機能が現場の実機(Windows ビルド + 4 カメラ + 3 ディスプレイ)で
動くかの確認**に絞る。

## 1. Display 1 オペレーター HUD(ビルドで確認 — Control Panel はビルドに無い)

スライダー 6 本(上から History (live) / History (print) / Brightness / Decimate /
Seed Count / Web Curve Tol)+ トグル 4 つ + ボタン 2 つの縦並びになった。

- [ ] スライダー 6 本すべてが表示され、ドラッグで**即座に見た目が変わる**こと
  - History (live): 点群のモーションカーブの時間窓が伸縮する
  - History (print): 値を変えても**ライブ表示は変わらない**(書き出し時のみ使用)。
    Export 後の preview / GLB に反映されているかで確認
  - Brightness: 点群全体の明るさ
  - Decimate: 点群の間引き率(%)
  - Seed Count: モーションカーブの本数
  - Web Curve Tol: ライブ非反映。Export 後のファイルサイズ/形状で確認
- [ ] パネル全体(スライダー+トグル+ボタン+チートシート)が
  **オペレーターディスプレイの縦に収まる**こと(行数が大幅に増えた)
- [ ] **Save settings** → アプリ再起動 → スライダー値が復元されること(PlayerPrefs)
- [ ] **Keep recordings** トグル
  - ON: 体験 1 周(QR まで)後も visitorRecordingRoot にテイクが残る。
    ログに `kept visitor take ... (keep-recordings ON)` が出る
  - OFF(既定): 従来どおり QR / 次回開始の境界でテイクが削除される
  - ON 運用時は**ディスク残量を監視**(1 テイク数 GB、自動削除なし)
- [ ] **Flip Display 3 (X)** トグル
  - ON で Display 3 だけ左右反転し、**Display 2 には影響しない**こと
  - Save ボタン不要で**トグルした瞬間に永続化** → 再起動後も反転が維持されること
  - 反転状態で立体(TSDF メッシュ等)が裏返って見えない(面が黒くならない)こと
- [ ] Tab キーで HUD ⇄ 4 カメラタイルの切替が引き続き動くこと
- [ ] (旧) QR Dry Run トグルは **HUD から削除済み**。dry run の切替は
  ExperienceConfig.dryRunPublish で行う — 本番前に false を確認

## 2. オペレータープレビュー(Display 1 右上 = Display 2 ミラー)

- [ ] Display 1 右上の小窓に**来場者画面(Display 2)のミラー**が出ること
  (従来の単カメラ color feed から変更。戻すのは Inspector の mirrorDisplay2=false)
- [ ] **既知の制限**: ScreenSpaceOverlay の来場者 UI(QR コード・メッセージ)は
  ミラーに**映らない**。QR 表示確認は物理ステージモニターで行うこと
- [ ] Display 2 側が X 反転(MirrorDisplayCamera)している場合でも、ミラー小窓の
  立体が裏返らない(inverted culling 対応)こと
- [ ] 体験フロー 1 周の間、小窓のフレームレート/遅延が運用に耐えること

## 3. Playback 床キャリブ(録画の持ち回り)

- [ ] 現地リグで **Save(収録保存)したテイクの `calibration/` に floor.yaml が
  コピーされている**こと(cameras.yaml と同様の自己完結化)
- [ ] G キー床チューニング(キャリブ UI)で、**playback 中の `_Playback_*` GO も
  追従して動く**こと(ライブリグが無い機でも効く)
- [ ] チューニング保存後、次のテイクロードで床が維持されること
- [ ] floor.yaml が live にも take にも無い場合のみ、シーン既定値に戻り
  **警告ログ `no floor.yaml under ...`** が出ること(黙って古い値を使い続けない)
- [ ] floor.yaml ありのテイク → 無しのテイクに切り替えたとき、
  **前のテイクの床が残らない**こと(fallback に復元される)

## 4. Web 書き出しチューニングループ

Editor(Control Panel)での事前調整と、ビルドでの本番書き出しの両方。

- [ ] (Editor) Window > Control Panel の **「Web export preview (書き出しデータ確認)」**:
  Capture → Preview で書き出し形状がその場に出る(ファイル生成・アップロード無し)。
  Hide preview でライブ表示へ戻る
- [ ] Status 行の **推定 GLB サイズ(est GLB ~x.x MB)** を見ながら
  Web Curve Tol / Decimate / History (print) を調整 → 実 Export のファイルサイズと
  数 % 以内で一致すること
- [ ] (ビルド) HUD の **Export GLB + USDZ** → StatusText が進行 → 完走。
  現地回線で実アップロード + QR → 実機(iPhone/Android)で表示確認
- [ ] GLB ファイル名に `_t<trailSamples>` が付き、意図した History (print) 値で
  書き出されていること

## 5. プリント書き出し(8/7 変更分)

- [ ] **stlFloorCenterOnModel**(既定 ON でシーンに wire 済): トレイルが張り出した
  スカルプチャで床プレートがモデル全体の XZ 中心に来て、**印刷時に倒れにくい
  バランス**になっていること
- [ ] プリントファイル名に **テイクフォルダ名 + フレーム番号のサフィックス**
  (`_<take>_f01234`)が付き、どの収録由来か追えること
- [ ] **freeOrbitInDev**: Experience Mode OFF のとき、ポーズ中でもオービット操作で
  カメラを動かせること。Experience Mode ON(本番)では従来どおり抑制されること

## 実施メモ

- HUD の自動解決(pointCloudView / decimater / motionCurves / display3Mirror)は
  シーンに該当コンポーネントが無いと**その行ごと非表示**になる。行が欠けていたら
  シーン構成を疑う
- Flip Display 3 と Save settings は別系統(前者は即時 PlayerPrefs、後者はボタン押下時)。
  「Save してないのに残る/ Save したのに残らない」と混同しないこと
