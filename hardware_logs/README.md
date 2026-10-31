# Hardware log sessions

Folder ini khusus untuk bukti dan diagnosis **hardware fisik** (ESP32, TB6600,
motor, AS5600, dan UWB). Ini sengaja terpisah dari `logs/`, yang dipakai oleh
simulator dan dashboard. Isi sesi tidak di-commit agar data eksperimen tidak
tercampur dengan source code.

## Rekam satu sesi

Sambungkan ESP32 lalu jalankan dari root repository:

```powershell
python scripts\capture_hardware_log.py --port COM5 --label uji_lurus
```

Ganti `COM5` sesuai `pio device list`. Tekan `Ctrl+C` setelah pengujian selesai.
Skrip membuat satu folder seperti
`hardware_logs/20261030_143012_uji_lurus/` yang berisi:

- `raw_serial.log` — semua byte serial yang diterima, termasuk pesan boot dan
  error non-JSON;
- `packets.jsonl` — setiap baris serial dalam envelope dengan waktu PC;
  paket JSON firmware berada di properti `packet`;
- `manifest.json` — metadata port, baud rate, mulai/selesai, dan jumlah baris.

Perintah dapat dikirim dan ikut dicatat dengan `--command`, misalnya:

```powershell
python scripts\capture_hardware_log.py --port COM5 --label pid_kiri `
  --command "CMD:status" --command "CMD:destination 11" --command "CMD:start"
```

Perekam membutuhkan `pyserial` (sudah tercantum pada
`dashboard/requirements.txt`):

```powershell
python -m pip install pyserial
```

## Audit sesi

```powershell
python scripts\audit_hardware_log.py hardware_logs\20261030_143012_uji_lurus
```

Auditor menulis `audit_report.md` dan `audit_summary.json` ke folder sesi.
Ia memeriksa kontinuitas telemetri, urutan waktu firmware, state navigasi,
kualitas/freshness posisi, kesehatan AS5600, tracking/saturasi PID roda, dan
deadline control-loop. Hasilnya adalah indikator diagnosis; bukan bukti bahwa
motor atau posisi sudah benar secara fisik. Sertakan folder sesi tersebut saat
meminta audit agar data mentah dan kesimpulannya dapat diperiksa ulang.

Untuk audit file lama dari dashboard, berikan path file `.jsonl` langsung.
