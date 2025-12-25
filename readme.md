# 環境構築

- とりあえず、以下のコードを管理者権限のターミナルで実行し、ドライバをOSにいれる
```
Invoke-WebRequest 'https://dl.espressif.com/dl/idf-env/idf-env.exe' -OutFile .\idf-env.exe
.\idf-env.exe driver install --espressif
```
- デバイスマネージャーから、ユニバーサルシリアルデバイスの中の「USB JTAG/serial debug unit」を選ぶ
- "ドライバーの更新"、"コンピュータを参照してドライバーを検索"、"コンピュータ上の利用可能なドライバーの一覧から選択します"の順で選ぶ
- 互換性のあるハードウェアを表示にチェックがついている状態で、モデルに、USBシリアルデバイスという項目を選ぶ
- 次へを押してインストールする

これでシリアルポートに認識されるため、USB CDCが使えるようになり、espflashが使えるようになる