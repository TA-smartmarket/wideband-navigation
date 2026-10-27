# Project Structure

Dokumen ini adalah peta singkat repository Smart Trolley Navigation. Tujuannya
agar setiap file mudah dicari tanpa memindahkan source atau mengubah perilaku
program.

## Struktur utama

```text
smart-trolley-navigation/
|-- navigation_core/   C++17 portable: algoritma navigasi bersama
|   |-- include/navigation/  public API
|   `-- src/                 implementation
|-- firmware/          integrasi ESP32-S3 dan perangkat keras
|   |-- include/             konfigurasi dan interface driver
|   `-- src/                 task, provider, driver, dan telemetry
|-- dashboard/         server kontrol lokal dan UI browser
|   `-- static/              HTML, CSS, dan Three.js
|-- simulator/         simulator desktop berbasis Python/Pygame
|   `-- smart_trolley_sim/   package simulator
|-- config/            konfigurasi map, graph, navigasi, dan simulasi
|-- tests/             test native C++ per subsistem
|-- test_data/         fixture UWB, rute, dan skenario deterministik
|-- scripts/           build, validasi, generator, dan runner
|-- docs/              dokumentasi dan diagram
|-- wokwi/             aset simulasi ESP32 opsional
|-- logs/              output runtime; bukan source
|-- results/           hasil pengujian/simulasi; bukan source
|-- platformio.ini     konfigurasi build utama
|-- README.md          panduan penggunaan
`-- AGENTS.md          aturan review dan perubahan repository
```

## Batas tanggung jawab

| Komponen | Boleh berisi | Tidak boleh berisi |
| --- | --- | --- |
| `navigation_core/` | Graph, Dijkstra, FSM, kontrol, odometry, C ABI | Arduino, GPIO, serial, Wi-Fi, UI |
| `firmware/` | FreeRTOS, ESP32, sensor, motor, NVS, serial/MQTT | Implementasi ulang algoritma navigasi |
| `dashboard/` | FastAPI, WebSocket, serial bridge, editor map, UI | Kontrol motor real-time atau algoritma navigasi kedua |
| `simulator/` | Plant, UWB fault model, renderer, logging | Salinan Dijkstra/FSM Python |
| `config/` | Input konfigurasi yang versioned | Log, cache, atau hasil runtime |
| `tests/` | Test native C++ | Source produksi |
| `test_data/` | Fixture deterministik | Output run ad-hoc |
| `scripts/` | Tooling yang dijalankan dari root | Source runtime firmware/dashboard |

## Cara mencari file

- Algoritma atau tipe publik: `navigation_core/include/navigation/`.
- Implementasi algoritma: `navigation_core/src/`.
- Pin, motor, encoder, dan ESP32: `firmware/include/` dan `firmware/src/`.
- API/server dashboard: `dashboard/main.py`, `dashboard/models.py`, dan
  `dashboard/serial_bridge.py`.
- Tampilan dashboard: `dashboard/static/`.
- Perilaku simulator: `simulator/smart_trolley_sim/`.
- Nilai map dan tuning: `config/`.
- Cara menjalankan dan memvalidasi: `README.md`, `docs/QUICKSTART_ID.md`, dan
  `docs/testing.md`.

## File source dan file generated

File di bawah ini bukan source dan aman untuk tidak ditampilkan di Explorer:

```text
.pio/                  output dan dependency PlatformIO
.venv/, .venv-1/       environment Python lokal
build/                 native shared library hasil build
dashboard/node_modules dependency JavaScript lokal
**/__pycache__/        cache Python
logs/                  log runtime
results/               hasil simulator dan audit
config/dashboard_scene.json  scene kerja dashboard saat runtime
```

Folder tersebut tetap berada di disk. Konfigurasi `.vscode/settings.json`
hanya menyembunyikannya dari Explorer dan pencarian agar source lebih mudah
dibaca.

## Alur dependensi

```text
config + test_data
        |
        v
navigation_core <--- simulator
        |
        +-----------> native tests
        |
        `-----------> firmware ---> ESP32 hardware

dashboard ---> config/map/graph/scene
          `--> serial/WebSocket ---> firmware atau simulation mode
```

`navigation_core/` tetap menjadi satu-satunya sumber kebenaran untuk Dijkstra,
path following, dan navigation FSM.

## Audit kebersihan repository

### Dipertahankan

- Struktur source saat ini sudah memisahkan library, firmware, server/dashboard,
  simulator, konfigurasi, test, dan tooling dengan jelas.
- `logs/` dan `results/` tetap ada dengan `.gitkeep`, sedangkan hasil run
  diabaikan Git.
- Tidak ada source yang dipindahkan, diganti nama, atau dihapus dalam audit ini.

### Kandidat perapian berikutnya

Hal-hal berikut sengaja tidak dilakukan otomatis karena dapat menghapus data
lokal atau mengubah banyak path:

1. Pilih satu dari `.venv/` atau `.venv-1/`, lalu hapus environment yang tidak
   dipakai setelah diverifikasi.
2. Tinjau dua file `docs/.$*.drawio.bkp`; keduanya backup editor yang saat ini
   terlacak Git.
3. Jika nama `dashboard/` ingin diganti menjadi `server/`, lakukan sebagai
   refaktor tersendiri karena perintah, import Python, dokumentasi, dan proses
   startup harus diperbarui bersama-sama.
4. Jangan memindahkan `navigation_core/` ke `lib/` hanya demi estetika sebelum
   memperbarui `platformio.ini`, native build script, simulator binding, dan
   semua dokumentasi yang merujuk path tersebut.

## Pemeriksaan standar

Jalankan dari root repository setelah perubahan struktural:

```bat
python scripts\validate_config.py
python -m unittest dashboard.test_models
pio test -e native
pio run -e esp32-s3-devkitc-1
python simulator\main.py --headless --all
```

